/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-12-31 16:06:19
 * @Project      : UM_path_planning
 */

#ifndef BLOCK_PLANNING_H
#define BLOCK_PLANNING_H
#include "common_function/MapFunction.h"
#include "um_chassis/chassisBase.h"
#include "common_function/MotionControl.h"
#include "common_function/ExceptionHanding.h"
#include "navigation_algorithm/RoadPlanning.h"
#include "navigation_algorithm/AlongWall.h"

#pragma once

namespace useerobot{

#define AREA_LENGTH 33

struct boundary
{
    int up;
    int down;
    int left;
    int right;
};
struct Grate
{
  Grid grating;
  int grateSign;
  int ldsSign;
  int grateMax;
  int grateTime;
};

enum DIVB
{
  UP,
  DOWN,
  RIGHT,
  LEFT,
  IDLE,
};



class blockPlanning
{
  public:
      blockPlanning(/* args */);
      ~blockPlanning();
      
      // Maze _maze;

     
      int curAdd;
      int conIndex;
      // mapRange _map;
      DIVB divBound;
      DIVB divWall;
      void SetBound(boundary stu);
      void GetBound(vector <boundary>* arr,boundary* stu);
      void DivArea(Sensor sensor,Grid cur,Trouble trouble);
      void init();
      int FullBound(Grid cur,Trouble trouble);
      int ReMove(Grid cur,DIVB forward);
  private:
    AlongWall _wall;
    chassisBase chassisBlock;
    MotionControl motionBlock;
    RoadPlanning roadBlock;
    Grid aim;
    Grate grate;
    RoadAim _aim;
    Trouble _trouble;
    
    int continues;
    int returnSign;
    int CalGrating(Sensor sensor,Grid cur,int forward);
    int SetInformation(Sensor sensor,Grid cur);
    void BackBound(Grid cur);
    void GetBoundOb(Grid cur,Trouble trouble);
   
    
};


}
#endif