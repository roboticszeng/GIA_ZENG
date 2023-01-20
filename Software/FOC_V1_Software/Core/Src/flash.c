/*
 * flash.c
 *
 *  Created on: Mar 25, 2022
 *      Author: LX
 */

#include "flash.h"
#include "tim.h"
#include "AS5600.h"
#include "gpio.h"

// 读取指定地址的半字(16位数据)
// faddr:读地址
// 返回值:对应数据.
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr)
{
    return *(volatile uint16_t *)faddr;
}
#if STM32_FLASH_WREN // 如果使能了写
// 不检查的写入
// WriteAddr:起始地址
// pBuffer:数据指针
// NumToWrite:半字(16位)数
void STMFLASH_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
    uint16_t i;
    for (i = 0; i < NumToWrite; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WriteAddr, pBuffer[i]); // 每次写入半字
        WriteAddr += 2;                                                       // 地址增加2.
    }
}
// 从指定地址开始写入指定长度的数据
// WriteAddr:起始地址(此地址必须为2的倍数!!)
// pBuffer:数据指针
// NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
#if STM32_FLASH_SIZE < 256
#define STM_SECTOR_SIZE 1024 // 字节
#else
#define STM_SECTOR_SIZE 2048
#endif
uint16_t STMFLASH_BUF[STM_SECTOR_SIZE / 2]; // 最多是2K字节
void STMFLASH_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
    uint32_t secpos;    // 扇区地址
    uint16_t secoff;    // 扇区内偏移地址(16位字计算)
    uint16_t secremain; // 扇区内剩余地址(16位字计算)
    uint16_t i;
    uint32_t offaddr; // 去掉0X08000000后的地址

    if (WriteAddr < STM32_FLASH_BASE || (WriteAddr >= (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE)))
        return; // 非法地址

    HAL_FLASH_Unlock();                       // 解锁
    offaddr = WriteAddr - STM32_FLASH_BASE;   // 实际偏移地址.
    secpos = offaddr / STM_SECTOR_SIZE;       // 扇区地址  0~127 for STM32F103RBT6
    secoff = (offaddr % STM_SECTOR_SIZE) / 2; // 在扇区内的偏移(2个字节为基本单位.)
    secremain = STM_SECTOR_SIZE / 2 - secoff; // 扇区剩余空间大小
    if (NumToWrite <= secremain)
        secremain = NumToWrite; // 不大于该扇区范围
    while (1)
    {
        STMFLASH_Read(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2); // 读出整个扇区的内容
        for (i = 0; i < secremain; i++)                                                                // 校验数据
        {
            if (STMFLASH_BUF[secoff + i] != 0XFFFF)
            {
                led_blink(BLINK_FAST, 2);
                break; // 需要擦除
            }
        }
        if (i < secremain) // 需要擦除
        {
            FLASH_PageErase(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE); // 擦除这个扇区
            FLASH_WaitForLastOperation(FLASH_WAITETIME);                  // 等待上次操作完成
            CLEAR_BIT(FLASH->CR, FLASH_CR_PER);                           // 清除CR寄存器的PER位，此操作应该在FLASH_PageErase()中完成！
                                                // 但是HAL库里面并没有做，应该是HAL库bug！
            for (i = 0; i < secremain; i++) // 复制
            {
                STMFLASH_BUF[i + secoff] = pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2); // 写入整个扇区
            
            
        }
        else
        {
            FLASH_WaitForLastOperation(FLASH_WAITETIME);           // 等待上次操作完成
            STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain); // 写已经擦除了的,直接写入扇区剩余区间.
        }
        if (NumToWrite == secremain)
            break; // 写入结束了
        else       // 写入未结束
        {
            secpos++;                   // 扇区地址增1
            secoff = 0;                 // 偏移位置为0
            pBuffer += secremain;       // 指针偏移
            WriteAddr += secremain * 2; // 写地址偏移(16位数据地址,需要*2)
            NumToWrite -= secremain;    // 字节(16位)数递减
            if (NumToWrite > (STM_SECTOR_SIZE / 2))
                secremain = STM_SECTOR_SIZE / 2; // 下一个扇区还是写不完
            else
                secremain = NumToWrite; // 下一个扇区可以写完了
        }
    };
    HAL_FLASH_Lock(); // 上锁
}
#endif

// 从指定地址开始读出指定长度的数据
// ReadAddr:起始地址
// pBuffer:数据指针
// NumToWrite:半字(16位)数
void STMFLASH_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead)
{
    uint16_t i;
    for (i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr); // 读取2个字节.
        ReadAddr += 2;                                // 偏移2个字节.
    }
}

// 测试用///
// WriteAddr:起始地址
// WriteData:要写入的数据
void Test_Write(uint32_t WriteAddr, uint16_t WriteData)
{
    STMFLASH_Write(WriteAddr, &WriteData, 1); // 写入一个字
}

extern uint16_t ADDR_ARRAY[];

#define FLASH_USER_START_ADDR   ((uint32_t)0x0800f000)          /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ((uint32_t)0x0800fc00)  /* End @ of user Flash area */

void save_config(sdo_typedef *sdo)
{
    uint16_t data_read_flag = DATA_READ_FLAG;

    // 保存配置后所有定时器停止工作，避免中断干扰的问题
    HAL_TIM_Base_Stop_IT(&htim1);
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_TIM_Base_Stop_IT(&htim4);

    uint16_t *point_sdo = (uint16_t *)sdo;
    
    static FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
    HAL_FLASH_Unlock();

    for(int i = 0; i < sizeof(*sdo) / sizeof(uint16_t); i++)
    {
        // sdo所有变量都写入flash
        // STMFLASH_Write(_get_addr_flash(ADDR_ARRAY[i]), &(*(point_sdo++)), 1);
        STMFLASH_Write(_get_addr_flash(ADDR_ARRAY[i]), point_sdo, 1);
        point_sdo++;
    }

    STMFLASH_Write(_get_addr_flash(ADDR_READ_FLAG), &data_read_flag, 1);
}

extern encoder_typedef* oEncoder;
extern state_typedef* oState;
extern config_typedef* oConfig;
extern sdo_typedef* oSdo;
extern pdo_typedef* oPdo;


void read_config(sdo_typedef *sdo)
{
    uint16_t read_flag = 0;
    uint16_t *point_sdo = (uint16_t *)sdo;
    STMFLASH_Read(_get_addr_flash(ADDR_READ_FLAG), &read_flag, 1);
    if (read_flag != DATA_READ_FLAG)
    {
        // 第一次上电的情况
        // 1-SDO全部写零
        for(int i = 0; i < sizeof(*sdo) / sizeof(uint16_t); i++)
        {
            *(point_sdo++) = 0;
        }

        // 2-自动电角度对齐
        calib_elec_angle(oState, oSdo);

        // 3-自动ADC对齐
        calib_adc_offset(oPdo, oSdo);
        
//        sdo->SDO_ELEC_ZERO_POSITION = oConfig->ELEC_ZERO_POSITION;
//        sdo->SDO_ADC0_OFFSET = oConfig->CONST_ADC0_OFFSET;
//        sdo->SDO_ADC1_OFFSET = oConfig->CONST_ADC1_OFFSET;
    }
    else
    {
        // 不是第一次上电的情况
        
        // 1-小灯快闪十下
        for (unsigned int i = 0; i < LED_BLINK_READ_FLASH * 2; i++)
        {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            HAL_Delay(100);
        }

        // 2-读到的FLASH数据写入SDO
        for(int i = 0; i < sizeof(*sdo) / sizeof(uint16_t); i++)
        {
            STMFLASH_Read(_get_addr_flash(ADDR_ARRAY[i]), point_sdo, 1);
            point_sdo++;
        }
    }

}