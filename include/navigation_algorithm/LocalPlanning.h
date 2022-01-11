/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  :
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-10 09:29:35
 * @Project      : UM_path_planning
 */


#pragma once
#ifndef LOCAL_PLANNING_H
#define LOCAL_PLANNING_H

#include <algorithm>
#include <Eigen/Core>
#include <vector>
#include <cstdio>
#include "common_function/MotionControl.h"
#include "common_function/logger.h"
// #include "navigation_algorithm/RoadPlanning.h"
using namespace std;
 #ifndef __UMUSERMODE__
#define __UMUSERMODE__ CHASSIS_USER_MODE_SLAM
#endif

namespace useerobot
{
extern current_pose_t* current_pose;
extern current_pose_t raw_share_pose;
extern CHASSIS_CONTROL GLOBAL_CONTROL;

	// enum class RobotType {
	//     circle,
	//     rectangle
	// };
	extern PRO process;

	//dwa parameters
	struct Config
	{
		//Maximum linear velocity (m/s)
	    float max_speed = 0.22;
	    // float min_speed = -0.10;
			float min_speed = 0.0;			

		//Maximum angular velocity
	    float max_yawrate =  90.0f * _Pi / 180.0f;

		//Maximum linear acceleration
	    float max_accel = 4.0f;

		//Maximum angular acceleration
	    float max_dyawrate = 1400.0f * _Pi / 180.0f;
			
			// float max_dyawrate = 700.0f * _Pi / 180.0f;
		//Linear velocity resolution
	    // float v_reso = 0.02f;
			float v_reso = 0.03f;

		//Angular velocity resolution
	    float yawrate_reso = 10 * _Pi / 180.0f;
			// float yawrate_reso = 5 * _Pi / 180.0f;

		//Control cycle
	    // float dt = 0.05;
			float dt = 0.15;

	    float predict_time = 1.0;
			// float predict_time = 0.5;

		//Segment end cost
	    // float to_goal_cost_gain = 1;
			float to_goal_cost_gain = 0.7;
			float to_goal_angle_gain = 0.5;

		//Speed cost
	    float speed_cost_gain = 0.3;
			float speed_w_cost_gain = 0.1;
		// obstacle cost
	    float obstacle_cost_gain =0.05;

		//Astar path end cost
		// float global_goal_cost_gian =0.0001;
		
			float global_goal_cost_gian =0.1;

	    //ROBOT TYPE
	    RobotType robot_type = RobotType::circle;

	    //for circular robot (m)
	    float robot_radius_recordmap = 0.07;

			float robot_radius_dymap = 0.15;

		//Two wheel wheelbase (m)
		float wheel_L = 0.22f;

	    //for rectangular robot
	    float robot_width = 0.15;
	    float robot_length = 0.2;

		float lattice_resolution = 0.05; //m
		uint16_t motion_step = 1;

	};

	struct Control
	{
		//linear velocity
	    float v;

		//angular velocity
	    float w;
	};

	struct RobotData
	{
	    float x;
	    float y;
	    float theta;
	    float v;
	    float w;
	};

	using Trajectory = std::vector<RobotData>;
	using Obstacles = std::vector<std::pair<float, float>>;

    class LocalPlanning
    {
    	private:
        /* data */

   		public:
			LocalPlanning(/* args */);
			~LocalPlanning();
    };

	//Local obstacle avoidance
    class dwa : public LocalPlanning
    {
	    private:
	        struct DynamicWindow
	        {
		        float vmin;
		        float vmax;
		        float yaw_rate_min;
		        float yaw_rate_max;
	        };

		public:
			enum pathPlannerRunState
			{
				Error = 1,
				Success,
				Collide,
				Unreach,
				Idle,
			};

			enum headingaAlignState
			{
				FirstAlignment = 1,
				Run,
				Finish,
				Idel,
			};

			std::shared_ptr<std::thread> Dwa_thread_ = nullptr;
			headingaAlignState heading_align_state;

			//Motion information of current robot
			pathPlannerRunState  dwa_run_state;

			//Motion information of current robot
			RobotData currentRobotData;

			//Current wheel speed obtained from chassis
			move_data curWheelInfo;

			//Convert the angular and linear speed predicted by DWA into wheel speed
			move_data calWheelInfo;

			move_data moveData;
			Obstacles recordmap_obstaclesPos;
			Obstacles dymap_obstaclesPos;
			chassisBase chassisbaseControl;
			struct Config dwaParamConfig;
			Eigen :: Vector2f goalPos = {0, 0};
			std::pair<Control, Trajectory> predInfo;
			uint16_t offset = 0;
			RobotData robotData;
			int dwa_sum_times = 0;
			bool updataGoalPos_index = false;
			MotionControl localplanning_motion;
			Grid local_aim;
			double error_angle;
			bool last_caculate_state = false;
			bool fast_dwa_turn_index = false;
			bool turn_finished_index = true;
			// Maze _maze;
	    
			public:
	        dwa(const int16_t start_point_x, const int16_t start_point_y,
			 								const int16_t end_point_x, const int16_t end_point_y);
	        dwa();

	        ~dwa();

			/**
			 * @description: Convert X and Y coordinates into a set of vectors
			 * @event:
			 * @param {float} x
			 * @param {float} y
			 * @return { vector<std::pair<float, float>>}
			*/
      	    vector<std::pair<float, float>> pointToVector(float x, float y);

			/**
			 * @description: Put the predicted track points into the vector container
			 * @event:
			 * @param {const Trajectory} traj
			 * @return {vector<std::pair<float, float>>}
			*/
       		vector<std::pair<float, float>> trajectorToVector(const Trajectory& traj);

			/**
			 * @description: Calculate the position of the current machine movement
			 * @event:
			 * @param {const RobotData&} x
			 * @param {const Control} u
			 * @param {float} dt
			 * @return {RobotData}
			*/
			RobotData motion(const RobotData& x, const Control& u, float dt);

			/**
			 * @description: A set of linear and angular velocity ranges are calculated according to the current motion state
			 * @event:
			 * @param {const RobotData} x
			 * @param {const Config} cfg
			 * @return {DynamicWindow}
			*/
			DynamicWindow calculateDynamicWindow(const RobotData& x, const Config& cfg) const;

			/**
			 * @description: A set of linear and angular velocity ranges are calculated according to the current motion state
			 * @event:
			 * @param {const RobotData} x
			 * @param {const Config} cfg
			 * @param {const Eigen::Vector2f} goal
			 * @param {const Obstacles&} obs
			 * @param {const Obstacles&} globalPath
			 * @return {pair<Control, Trajectory>}
			*/
	        pair<Control, Trajectory> dwaControl(const RobotData& x, const Config& cfg,
			                     const Eigen::Vector2f& goal, const Obstacles& re_obs, const Obstacles& dy_obs, const Obstacles& globalPath);

			/**
			 * @description: Calculate the distance from the end of the predicted trajectory to the target point
			 * @event:
			 * @param {const Trajectory} traj
			 * @param {const Eigen::Vector2f} goal
			 * @return {pair<Control, Trajectory>}
			*/
	        float calculateToGoalCost(const Trajectory& traj, const Eigen::Vector2f& goal);

			/**
			 * @description: Calculate the distance from the end of the predicted track to the end of the global path
			 * @event:
			 * @param {const Trajectory} traj
			 * @param {const Obstacles} globalPath
			 * @return {float}
			*/
			float calculateToPathDesCost(const Trajectory& traj, const Obstacles& globalPath);

			/**
			 * @description: Predicting the trajectory of robot motion
			 * @event:
			 * @param {const RobotData} x_init
			 * @param {const Control} u
			 * @param {const Config} cfg
			 * @return {Trajectory}
			*/
	        Trajectory predictTrajectory(const RobotData& x_init, const Control& u, const Config& cfg);

			/**
			 * @description: A set of optimal linear velocity and angular velocity are calculated
			 * @event:
			 * @param {const RobotData} x
			 * @param {const DynamicWindow} dw
			 * @param {const Config} cfg
			 * @param {const Eigen::Vector2f} goal
			 * @param {const Obstacles} obs
			 * @param {const Obstacles} globalPath
			 * @return {std::pair<Control, Trajectory>}
			*/
	        std::pair<Control, Trajectory> calculateControlAndTrajectory(
	           							const RobotData& x, const DynamicWindow& dw,const Config& cfg,
	           							const Eigen::Vector2f& goal, const Obstacles& re_obs, const Obstacles& dy_obs,const Obstacles& globalPath);

			/**
			 * @description: Calculate the distance between the predicted trajectory point and the obstacle
			 * @event:
			 * @param {const Trajectory} traj
			 * @param {const Obstacles} obs
			 * @param {const Config} cfg
			 * @return {float}
			*/
	        float calculateObstacleCost(const Trajectory& traj, const Obstacles& re_obs, const Obstacles& dy_obs,
            											const Config& cfg);

			/**
			 * @description: Obtain the current status information of the robot
			 * @event:
			 * @param {}
			 * @return {RobotData}
			*/
			RobotData getCurrentRobotData();

			/**
			 * @description: Start path planner
			 * @event:
			 * @param {const} Starting point x coordinate
			 * @param {const} Starting point y coordinate
			 * @param {const} End x coordinate
			 * @param {const} End y coordinate
			 * @return {pathPlannerRunState} Robot motion state
			*/
			//pathPlannerRunState start_path_planner(const int16_t start_point_x, const int16_t start_point_y,
			 	//							const int16_t end_point_x, const int16_t end_point_y);
			bool start_path_planner(Sensor& sensor, Grid& cur, vector<pair<float, float> >& astarArr);

			/**
			 * @description: Reset path_planner
			 * @event:
			 * @return {void}
			*/
			void reset_path_planner(void);

			/**
	         * @description: Wheel speed is converted to robot motion speed
	         * @event:
	         * @param {move_data}wheelInfo
	         * @param {Config}cfg
	         * @param {double}angle
	         * @param {double}x_axis
	         * @param {double}y_axis
	         * @return {RobotData}
	        */
			RobotData wheelSpd2RobotSpd(const move_data &wheelInfo, const Config &cfg,
														double angle, double x_axis, double y_axis);

			/**
			 * @description: Robot motion speed is converted to wheel speed
			 * @event:
			 * @param {Control&}robotPosInfo
			 * @param {Config}cfg
			 * @return {move_data}
			*/
			move_data robotSpd2WheelSpd(const Control& robotPosInfo, const Config& cfg);

			/**
			 * @description: Adjust the heading of the robot and align it with the direction of the target point
			 * @event:
			 * @param {Eigen::Vector2f&} goalPos
			 * @param {RobotData&} robotPos
			 * @param {Config&} cfg
			 * @return {heading_align_state}
			*/
			headingaAlignState heading_align(Eigen::Vector2f &goalPos, RobotData& robotPos,
															const Config& cfg, float& robotHeading);

			/**
			 * @description: Start heading alignment
			 * @event:
			 * @param {Eigen::Vector2f&} goalPos
			 * @param {RobotData&} robotPos
			 * @param {Config&} cfg
			 * @return {heading_align_state}
			*/
			headingaAlignState heading_align_run(Eigen::Vector2f &goalPos,
												RobotData& robotPos, const Config& cfg, float& robotHeading);

			/**
			 * @description: Update segment end position
			 * @event:
			 * @param {const Obstacles} astarPath
			 * @param {Eigen :: Vector2f} goalPos
			 * @param {RobotData} robotPos
			 * @return {void}
			*/
			void updataGoalPos(Obstacles& astarPath, Eigen :: Vector2f &goalPos, RobotData& robotPos,
											float& angle_, Config& cfg);

			void init();
			Grid CaculateAngle(Grid cur,Grid aim);
			void dwaStart();
      void dwaStop();
			void chassisDwa();
			void call_dwa();
			bool turn_run(Eigen::Vector2f& goalPos,RobotData& robotPos, const Config& cfg, float& robotHeading);
			float calculateAngleGoalCost(const Trajectory& traj, const Eigen::Vector2f& goal);
			
    };

    class aroundColumn : public LocalPlanning
    {
	    private:
	        /* data */

	    public:
	        aroundColumn(/* args */);
	        ~aroundColumn();
    };
}

#endif

