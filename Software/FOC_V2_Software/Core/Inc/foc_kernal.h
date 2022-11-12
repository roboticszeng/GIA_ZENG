
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FOC_KERNAL_H__
#define __FOC_KERNAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
float _normalizeAngle(float angle);
void setPhaseVoltage(float Uq, float Ud, float angle_elec);
void get_angle_elec(void);
    
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FOC_KERNAL_H__ */

