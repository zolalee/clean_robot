/*
 * @Author: DahlMill
 * @Date: 2020-03-04 10:49:34
 * @LastEditTime: 2020-10-02 10:41:48
 * @LastEditors: Please set LastEditors
 * @Description: 
 * @FilePath: /shmYuv/shm_com.h
 */

#ifndef __SHM_COM_H__
#define __SHM_COM_H__

#define MAX_SHM_SIZE 512 * 512 * 2

typedef struct STU_SHM_MAP
{
    int endFlag;                                //0, 未结束； 1， 结束
    char shmSp[MAX_SHM_SIZE];  //空间
} STU_SHM_MAP;

typedef struct STU_SHM_OTHER
{
    int endFlag;                                //0, 未结束； 1， 结束
    char shmSp[64];                        //空间
} STU_SHM_OTHER;

#endif
