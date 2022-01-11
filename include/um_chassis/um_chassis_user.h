/*
 * @Author: DahlMill
 * @Date: 2020-10-01 10:38:45
 * @LastEditors  : Please set LastEditors
 * @LastEditTime : 2021-06-04 19:28:05
 */
#ifndef __UM_CHASSIS_USER_H__
#define __UM_CHASSIS_USER_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
extern "C" {
#include "um_chassis_data.h"
}

#ifdef __cplusplus
extern "C" {
#endif

enum CHASSIS_USER_MODE
{
    CHASSIS_USER_MODE_NONE = 0,
    CHASSIS_USER_MODE_SLAM = 1,
};

#define LOG_SHOW 0

int UM_InitSensorMem2User(void (*Callback)(SensorData_t sensorData), int userMode);
void UM_InitControlMem2User(void);
int UM_WriteControlMem(ControlData_t cd);
void UM_SetImuCallBack(void (*Callback)(IMU_t imu));

#ifdef __cplusplus
}
#endif

#endif