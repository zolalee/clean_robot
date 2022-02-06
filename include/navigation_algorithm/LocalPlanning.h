/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  :
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-18 16:28:13
 * @Project      : UM_path_planning
 */


#pragma once
#ifndef LOCAL_PLANNING_H
#define LOCAL_PLANNING_H

#include <iostream>

#include <algorithm>
#include <Eigen/Core>
#include <vector>
#include <cstdio>
#include <queue>
#include "common_function/MotionControl.h"
#include "common_function/logger.h"
// #include "navigation_algorithm/RoadPlanning.h"

#ifndef __UMUSERMODE__
#define __UMUSERMODE__ CHASSIS_USER_MODE_SLAM
#endif
using namespace std;

//using namespace useerobot;
namespace useerobot
{
extern PRO process;
extern current_pose_t* current_pose;
extern current_pose_t raw_share_pose;
extern CHASSIS_CONTROL GLOBAL_CONTROL;
const float _PI = 3.1415;

struct Config
{
    float max_speed = 0.22;
   
    float min_speed = 0;			

    //float max_yawrate =  90.0 * _Pi / 180.0;
		float max_yawrate =  1.6;

    float max_accel = 4.0;

    float max_dyawrate = 1200.0 * _Pi / 180.0;  

    float v_reso = 0.02;

    //float yawrate_reso = 10 * _Pi / 180.0;
    float yawrate_reso = 0.2;

    float dt = 0.05;
   
    float predict_time = 3;

    float wheel_L = 0.20;
};
   
struct DynamicWindow
{
    float v_max;
    float v_min;
    float w_max;
    float w_min;
    
};
struct RobotData
{
    float x;
    float y;
    float theta;
    float v;
    float w;
};

struct Node
{
	int sl;
	int sr;
	float v_for;
	int v_i;

	// Node(float _v_for, int _v_i)
	// {
	// 	v_for = _v_for;
	// 	v_i = _v_i;
	// }

	friend bool operator < (const Node &a,const Node &b)
	{
		return a.v_for <= b.v_for; 
	} 
};

// RobotData RobotPos;
// DynamicWindow speedWindow;
const Config cfg;
using Trajectory = vector<RobotData>;

using Points = std::vector<std::pair<int, int>>;

class dwa
{
	public:
	
		float lastspeedW;
		int s_left;
		int s_right;
		int maxIndex1;
		int maxIndex2;
		int initDwa = 1;

		Node node;
		Grid stage_aim;
		Grid ready_aim;
		Grid _aim;
		Grid firstPoint;
		chassisBase chassisDwa;
		Grid start;
		Grid end;
		RobotData RobotPos;
		DynamicWindow speedWindow;
		MotionControl motionDwa;
	public:
		dwa(){

		}
		~dwa(){
		}

		void init(){
			initDwa = 1;
			lastspeedW = 0;
			maxIndex1 = 0;
			maxIndex2 = 0;
		}


		//void updataGoalPos(Points& astarPath);
		void updataGoalPos(Grid& cur,Points& astarPath,Grid& aim);
		bool start_path_planner(Sensor sensor,Grid cur,Points& astarArr);

		void WheelTrans(Sensor sensor, Grid cur,RobotData* pos){
				
				pos->v = (sensor.leftw + sensor.rightw) / (2.0 * 1000);
				pos->w = (sensor.leftw - sensor.rightw) / (cfg.wheel_L * 1000);  //rad/s
				
				pos->theta = cur.forward * _Pi / 180;
				pos->x = cur.realx * 0.15;
				pos->y = cur.realy * 0.15;
		}

		void dynamicWindow(DynamicWindow* speed){

				speed->v_max = min(cfg.max_speed, RobotPos.v+cfg.max_accel*cfg.dt);

				//speed->v_min = max(cfg.min_speed, RobotPos.v-cfg.max_accel*cfg.dt);
				// speed->w_max = min(cfg.max_yawrate, RobotPos.w+cfg.max_dyawrate*cfg.dt);
				// speed->w_min = max(-cfg.max_yawrate, RobotPos.w-cfg.max_dyawrate*cfg.dt);  
				speed->v_min = 0;
				speed->w_max = cfg.max_yawrate;
				speed->w_min = -cfg.max_yawrate;
					
		}


		bool PredictTrajectory(float v,float w,Trajectory& traj);
		
	


		//bool StartDwa(Sensor sensor, Grid cur,vector<pair<float, float> >& astarArr);

          
};

}
#endif

