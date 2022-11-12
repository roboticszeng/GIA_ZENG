clear;clc;

pid.pos.kp = 20.8;
pid.pos.ki = 0;
pid.pos.max = 15;
pid.pos.intmax = 15;
pid.vel.kp = 0.2;
pid.vel.ki = 1.5;
pid.vel.max = 2;
pid.vel.intmax = 1.5;
pid.curq.kp = 0.8;
pid.curq.ki = 40;
pid.curq.max = 0.9;
pid.curq.intmax = 0.5;
pid.curd.kp = 0.2;
pid.curd.ki = 0;
pid.curd.max = 0.2;
pid.curd.intmax = 0.1;


sdo.id = 0;
sdo.enc_zero_pos_h = 0;
sdo.enc_zero_pos_l = 2048;
struct_lib = {'pos', 'vel', 'curq', 'curd'};
for i = 1 : 4
    
[sdo.(['pid_' (struct_lib{i}) '_kp_int']), sdo.(['pid_' (struct_lib{i}) '_kp_dec'])] = ...
    convert_float_to_intdec(pid.(struct_lib{i}).kp, 16);
[sdo.(['pid_' (struct_lib{i}) '_ki_int']), sdo.(['pid_' (struct_lib{i}) '_ki_dec'])] = convert_float_to_intdec(pid.(struct_lib{i}).ki, 16);
sdo.(['pid_' (struct_lib{i}) '_max']) = convert_float_to_intdec(pid.(struct_lib{i}).max, 8);
sdo.(['pid_' (struct_lib{i}) '_intmax']) = convert_float_to_intdec(pid.(struct_lib{i}).intmax, 8);
end

sdo.filt_vel_cutoff_freq = 20;
sdo.filt_curq_cutoff_freq = 20;
sdo.filt_curd_cutoff_freq = 20;

sdo.pos_lb = 0;
sdo.pos_ub = 4096;
sdo.vel_lb = 24000;
sdo.vel_ub = 41000;
sdo.curq_lb = 800;
sdo.curq_ub = 3300;
sdo.curd_lb = 0;
sdo.curd_ub = 4096;

f = fieldnames(sdo);
a = nan(length(f), 1);
for i = 1 : length(f)
    a(i) = sdo.(f{i});
end


std_data = cell(length(f), 1);
rx_data = cell(8, 1);
for i = 1 : length(f)
    rx_data{1} = '55';
    rx_data{2} = num2str(sdo.id);
    rx_data{3} = num2str(dec2hex(round(sdo.(f{i}) / 256)));
    rx_data{4} = num2str(dec2hex(round(mod(sdo.(f{i}), 256))));
    for j = 2 : 4
        if length(rx_data{j}) == 1
            rx_data{j} = ['0', rx_data{j}];
        end
    end
    rx_data{5} = '00';
    rx_data{6} = '00';
    rx_data{7} = '0d';
    rx_data{8} = '0a';
    std_data{i} = 'x|';
    for j = 1 : 8
        std_data{i} = [std_data{i}, ' ', rx_data{j}];
    end
end

time = cell(length(f), 1);
for i = 1 : length(f)
    if i < 10
        time{i} = ['12:08:0' num2str(i)];
    elseif i >= 60
        disp('err');
    else
        time{i} = ['12:08:' num2str(i)];
    end
end

addr = {'0x3000', '0x3500', '0x3502', ...
    '0x3600', '0x3602', '0x3604', '0x3606', '0x3608', '0x360a', ...
    '0x3610', '0x3612', '0x3614', '0x3616', '0x3618', '0x361a', ...
    '0x3620', '0x3622', '0x3624', '0x3626', '0x3628', '0x362a', ...
    '0x3630', '0x3632', '0x3634', '0x3636', '0x3638', '0x363a', ...
    '0x3702', '0x3704', '0x3706', ...
    '0x3800', '0x3802', '0x3804', '0x3806', '0x3808', '0x380a', '0x380c', '0x380e'};
if length(addr) ~= length(f)
    disp('error');
end

outputfilename = 'motor_default_settings.csv';
fileID = fopen(outputfilename, 'w');
fprintf(fileID, '序号,系统时间,时间标识,CAN通道,传输方向,ID号,帧类型,帧格式,长度,数据\r\n');
for i = 1 : length(addr)
    fprintf(fileID, '%d,%s,无,ch1,发送,%s,数据帧,扩展帧,0x08,%s\r\n', ...
        (i-1), time{i}, addr{i}, std_data{i});
end
fclose(fileID);

