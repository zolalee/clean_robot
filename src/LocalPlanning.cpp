/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  :
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-11 16:06:23
 * @Project      : UM_path_planning
 */
#include "navigation_algorithm/LocalPlanning.h"
#include "navigation_algorithm/RoadPlanning.h"
#include <fstream>
#include <vector>
#include <iostream>
#include <sys/time.h>

using namespace std;
extern double x_ ;			 //Odometer x coordinates
extern double y_ ;			 //Odometer y coordinates
extern float angle_;		 //Odometer angle
extern useerobot::Maze _maze;
float last_wspeed = 0.0;
namespace useerobot
{
		
    extern RoadAim road_aim;
	extern dynamicMapInfo_t* dynamicMap;
		LocalPlanning::LocalPlanning(/* args */)
    {

    }
		LocalPlanning::~LocalPlanning()
		{

		}
    dwa::dwa()
	{
		heading_align_state = FirstAlignment;
		dwa_run_state = Idle;

	}
	dwa::dwa(const int16_t start_point_x, const int16_t start_point_y,
			 								const int16_t end_point_x, const int16_t end_point_y)
    {
		heading_align_state = FirstAlignment;
		dwa_run_state = Idle;



	//	cout << "planningMaze.startPoint.x:" << planningMaze.startPoint.x << "planningMaze.startPoint_y:"<<planningMaze.startPoint.y
	//			<< "planningMaze.endPoint.x:" << planningMaze.endPoint.x << "planningMaze.endPoint.y:"<<planningMaze.endPoint.y <<endl;
	}

    dwa::~dwa()
	{
		heading_align_state = FirstAlignment;
		dwa_run_state = Idle;
	}

    dwa::DynamicWindow dwa::calculateDynamicWindow(const RobotData& x, const Config& cfg) const
    {
        DynamicWindow vs =
		{
            cfg.min_speed,
            cfg.max_speed,
            -cfg.max_yawrate,
            cfg.max_yawrate
        };

        // dwa from motion model
        DynamicWindow vd =
        {
            x.v - cfg.max_accel * cfg.dt,
            x.v + cfg.max_accel * cfg.dt,
            x.w - cfg.max_dyawrate * cfg.dt,
            x.w + cfg.max_dyawrate * cfg.dt
        };

        // final, clip min and max of v and yaw
        DynamicWindow dw =
		{
            std::max(vs.vmin, vd.vmin),
            std::min(vs.vmax, vd.vmax),
            std::max(vs.yaw_rate_min, vd.yaw_rate_min),
            std::min(vs.yaw_rate_max, vd.yaw_rate_max)
        };

        return dw;
    }

    std::pair<Control, Trajectory> dwa::dwaControl(const RobotData& x,
    const Config& cfg, const Eigen::Vector2f& goal, const Obstacles& re_obs, const Obstacles& dy_obs,const Obstacles& globalPath)
    {
	    DynamicWindow dw = calculateDynamicWindow(x, cfg);

	    return calculateControlAndTrajectory(x, dw, cfg, goal, re_obs,dy_obs, globalPath);
    }

    std::pair<Control, Trajectory> dwa::calculateControlAndTrajectory
		(
    				const RobotData& x, const DynamicWindow& dw,
    				const Config& cfg, const Eigen::Vector2f& goal, const Obstacles& re_obs, const Obstacles& dy_obs,const Obstacles& globalPath)
    {
		float min_cost = std::numeric_limits<float>::max();
		Control best_u{0.0f, 0.0f};
		Trajectory best_traj;

   		// try all v and w combinations
    	for (float v = dw.vmin; v < dw.vmax; v += cfg.v_reso)
			{
	        for (float w = dw.yaw_rate_min; w < dw.yaw_rate_max; w += cfg.yawrate_reso)
				{
	            Control u {v, w};
	            Trajectory traj = predictTrajectory(x, u, cfg);

	            float to_goal_cost = cfg.to_goal_cost_gain * calculateToGoalCost(traj, goal);
	            // float speed_cost = cfg.speed_cost_gain * (cfg.max_speed - traj.back().v);
							// float speed_cost = cfg.speed_cost_gain * fabs(x.v - (traj.front().v+traj[1].v+traj[2].v)/3);
							// float speed_v_cost = cfg.speed_w_cost_gain *fabs(cfg.max_yawrate - traj.back().w);
							float speed_cost = cfg.speed_cost_gain * fabs(x.v - traj.front().v);
							float speed_v_cost = cfg.speed_w_cost_gain *fabs(x.w - traj.front().w);
							// float angle_cost = cfg.to_goal_angle_gain*calculateAngleGoalCost(traj,goal) ;
	            float ob_cost = cfg.obstacle_cost_gain * calculateObstacleCost(traj, re_obs, dy_obs,cfg);
							float to_path_des_cost = cfg.global_goal_cost_gian * calculateToPathDesCost(traj, globalPath);
	            float final_cost = to_goal_cost + speed_cost + ob_cost + to_path_des_cost + speed_v_cost;
							
							// float final_cost = to_goal_cost + speed_cost + ob_cost + to_path_des_cost;
	            if (final_cost < min_cost )
							{
	                // FRIZY_LOG(LOG_DEBUG, "DWA cost : to_goal_cost = %f ,speed_cost = %f ,speed_v_cost = %f,ob_cost = %f,to_path_des_cost = %f",to_goal_cost,speed_cost,speed_v_cost,ob_cost,to_path_des_cost);
									min_cost = final_cost;
	                best_u = u;
	                best_traj = traj;
	            }

				//cout << endl;
	          //  cout << " to_goal_cost:"<< to_goal_cost << "    speed_cost:" << speed_cost << "  ob_cost:"<< ob_cost << "  des_cost" << to_path_des_cost << "  final_cost:" << final_cost << "  best_v:"
	          //           << best_u.v << "   best_w:" << best_u.w << "   v :"  << v << "  w: " << w<< endl;
	    	}
	    }

        return {best_u, best_traj};
    }

    Trajectory dwa::predictTrajectory(const RobotData& x_init, const Control& u, const Config& cfg)
    {
		float time = 0.0f;
		Trajectory traj = {x_init};

    	RobotData x = x_init;

	    while (time <= cfg.predict_time)
		{
	        x = motion(x, u, cfg.dt);
	        traj.push_back(x);
	        time += cfg.dt;
	    }

    	return traj;
    }

    float dwa::calculateToGoalCost(const Trajectory& traj, const Eigen::Vector2f& goal)
    {
			float dx = goal.x() - traj.back().x;
    	float dy = goal.y() - traj.back().y;

  	  return sqrt(dx * dx + dy * dy);
    }
	float dwa::calculateAngleGoalCost(const Trajectory& traj, const Eigen::Vector2f& goal)
	{
		// need to add
		return 0.0;
	}
	float dwa::calculateToPathDesCost(const Trajectory& traj, const Obstacles& globalPath)
    {
		 float dx = traj.back().x - globalPath.back().first;
		 float dy = traj.back().y - globalPath.back().second;

		 return sqrt(dx * dx + dy * dy);
	}

    float dwa::calculateObstacleCost(const Trajectory& traj, const Obstacles& re_obs,const Obstacles& dy_obs,
 													const Config& cfg)
    {
	    // cost is inversely proportional to closest obs along path
	    if (cfg.robot_type == RobotType::circle)
			{
	        float min_r = std::numeric_limits<float>::max();

			// Traverse the distance between the trajectory point and the obstacle point
	        for (const RobotData& pose : traj)
					{
	            for ( auto obstacle : re_obs)
							{
	                float dx = obstacle.first - pose.x;
	                float dy = obstacle.second - pose.y;
	                float r = std::hypot(dx, dy);

					// will collide, inf cost 0.15m
	                if (r < cfg.robot_radius_recordmap)
	                {
									FRIZY_LOG(LOG_DEBUG, "THE OBSTACLE IS TOO CLOSE ");
									return std::numeric_limits<float>::infinity();
						
									}

	                min_r = std::min(r, min_r);
	            }
							for ( auto obstacle : dy_obs)
							{
	                float dx = obstacle.first - pose.x;
	                float dy = obstacle.second - pose.y;
	                float r = std::hypot(dx, dy);

					// will collide, inf cost 0.15m
	                if (r < cfg.robot_radius_dymap)
	                {
									FRIZY_LOG(LOG_DEBUG, "THE OBSTACLE IS TOO CLOSE ");
									return std::numeric_limits<float>::infinity();
						
									}

	                min_r = std::min(r, min_r);
	            }
	    		}

	        return 1.0f / min_r;
	  	}
		else if (cfg.robot_type == RobotType::rectangle)
		{
	        //TODO support rectangles
	    }

	    return 0.0f;
    }

    RobotData dwa::motion(const RobotData& x, const Control& u, float dt)
    {
		RobotData x_new;

 		x_new.theta = x.theta + u.w * dt;
    	x_new.x = x.x + u.v * std::cos(x_new.theta) * dt;
   		x_new.y = x.y + u.v * std::sin(x_new.theta) * dt;
    	x_new.v = u.v;
   		x_new.w = u.w;

    	return x_new;
    }

    RobotData dwa::getCurrentRobotData()
    {
        GridPose CurGridPose;
        CurGridPose = GetCurGridPose();

        currentRobotData.x = CurGridPose.i;
        currentRobotData.y = CurGridPose.j;
        currentRobotData.theta = CurGridPose.forward;
    }

    vector<std::pair<float, float>> dwa::pointToVector(float x, float y)
    {
        return { {x, y} };
    }

	RobotData dwa::wheelSpd2RobotSpd(const move_data &wheelInfo, const Config &cfg,
												double angle, double x_axis, double y_axis)
    {
		RobotData newRobotPos;

		newRobotPos.v = (wheelInfo.rightspeed + wheelInfo.leftspeed) / 2.0;
		newRobotPos.w = (wheelInfo.rightspeed - wheelInfo.leftspeed) / cfg.wheel_L;  //rad/s
		newRobotPos.theta = 2*_Pi-angle;
		newRobotPos.x = x_axis;
		newRobotPos.y = y_axis;

		return newRobotPos;
	}

	move_data dwa::robotSpd2WheelSpd(const Control& robotPosInfo, const Config& cfg)
	{
		move_data newWheelInfo;

		newWheelInfo.leftspeed  = robotPosInfo.v - robotPosInfo.w * cfg.wheel_L / 2.0;
		newWheelInfo.rightspeed = robotPosInfo.v + robotPosInfo.w * cfg.wheel_L / 2.0;

		// Output limiting
        if(newWheelInfo.leftspeed > 0.30)
		{
			newWheelInfo.leftspeed = 0.30;
		}

        if(newWheelInfo.leftspeed < -0.3)
		{
			newWheelInfo.leftspeed = -0.3;
		}

        if(newWheelInfo.rightspeed > 0.3)
		{
			newWheelInfo.rightspeed = 0.3;
		}

        if(newWheelInfo.leftspeed < -0.3)
		{
			newWheelInfo.leftspeed = -0.3;
		}

		return newWheelInfo;
	}

    vector<std::pair<float, float>> dwa::trajectorToVector(const Trajectory& traj)
    {
        std::vector<std::pair<float, float>> vec;
        vec.reserve(traj.size());

        for (const auto& pose : traj)
        {
			vec.emplace_back(pose.x, pose.y);
		}

        return vec;
    }

	dwa::headingaAlignState dwa::heading_align_run(Eigen::Vector2f& goalPos,
										RobotData& robotPos, const Config& cfg, float& robotHeading)
	{

		int16_t leftSpd = 0;
		int16_t rightSpd = 0;

		if(turn_finished_index == true)
		{
		double dx = goalPos.x() - robotPos.x;
		double dy = goalPos.y() - robotPos.y;
		error_angle = std::atan2(dy, dx) * 180 / _Pi;
		if (error_angle < 0)
		{
			error_angle += 360.0f;
		}
		error_angle = 360.0f - error_angle;
		FRIZY_LOG(LOG_DEBUG,"dy = %f, dx = %f", dy,dx);
		FRIZY_LOG(LOG_DEBUG,"error_angle: %f", error_angle);
		FRIZY_LOG(LOG_DEBUG,"robotHeading: %f", robotHeading*180.0f /_Pi);
		turn_finished_index = false;

		}
		// robotHeading = 90.0 - robotHeading;
		// if(robotHeading<0)

		// robotHeading = robotHeading+360.0;
		//Counterclockwise rotation
		if(error_angle > robotHeading*180.0f /_Pi && GLOBAL_CONTROL ==  WHEEL_RUN)
		{


				usleep(20* 1000);
				chassisbaseControl.chassisSpeed(120, -120, 1);
				FRIZY_LOG(LOG_DEBUG,"(error_angle - robotHeading*180.0f /_Pi): %f", (error_angle - robotHeading*180.0f /_Pi));
				if(abs(error_angle - robotHeading*180.0f /_Pi) < 5.0)
				{
					
					chassisbaseControl.chassisSpeed(0, 0, 1);
					heading_align_state = Finish;
					return Finish;
				}
		}
		else if(robotHeading*180.0f /_Pi  > error_angle && GLOBAL_CONTROL ==  WHEEL_RUN) //Clockwise rotation
		{

				usleep(20* 1000);
				chassisbaseControl.chassisSpeed(-120, 120, 1);
				FRIZY_LOG(LOG_DEBUG,"(error_angle - robotHeading*180.0f /_Pi): %f", (error_angle - robotHeading*180.0f /_Pi));
				if(abs(robotHeading*180.0f /_Pi  - error_angle) < 5.0)
				{
					heading_align_state = Finish;
      		chassisbaseControl.chassisSpeed(0, 0, 1);
					return Finish;
				}
		
		}

	}
	bool dwa::turn_run(Eigen::Vector2f& goalPos,
										RobotData& robotPos, const Config& cfg, float& robotHeading)
	{


	}
	void dwa::updataGoalPos(Obstacles& astarPath, Eigen :: Vector2f &goalPos, RobotData& robotPos,float& angle_, Config& cfg)
	{
		uint16_t size = astarPath.size();
		goalPos.x() = astarPath[offset+1].first;
		goalPos.y() = astarPath[offset+1].second;

		FRIZY_LOG(LOG_INFO,"astar path size: %d", size);
		FRIZY_LOG(LOG_DEBUG,"goalPos: %f %f", goalPos.x(), goalPos.y());

		
		heading_align_state = heading_align(goalPos, robotPos, cfg, angle_);
		if(heading_align_state != Finish)
		{
			FRIZY_LOG(LOG_INFO,"heading align uncomplete!");
			return;
		}


		// if (abs(robotPos.x - goalPos.x()) + abs(robotPos.y - goalPos.y()) > 0.03
		// 	&& _maze.GetMapState(goalPos.x()/(3*resolution),goalPos.y()/(3*resolution),2) != 2)
		// 	{
				
		// 		return;
		// 	}
		if(updataGoalPos_index == true)
		{
		FRIZY_LOG(LOG_DEBUG,"the offset is %d",offset);
		FRIZY_LOG(LOG_DEBUG,"UPdate for the next goal");
		if((size % cfg.motion_step) == 0 && size > (offset+2))
		{
			offset += cfg.motion_step;

		}
		else
		{
			if(size < (offset+2)&&size == (offset+2))
			{
				FRIZY_LOG(LOG_DEBUG,"update to the astarpath end");
				return;
			}
			if((size - offset) < cfg.motion_step)
			{
				offset += (size - offset);
			}
		}
		updataGoalPos_index = false;
		}


	}

	dwa::headingaAlignState dwa::heading_align(Eigen::Vector2f &goalPos, RobotData& robotPos,
															const Config& cfg, float& robotHeading)
	{
		if ( heading_align_state == FirstAlignment)
		{
			heading_align_state = Run;
		}

		FRIZY_LOG(LOG_DEBUG, "heading_align_state:%d",heading_align_state);

		switch(heading_align_state)
		{
			case Run:
				heading_align_run(goalPos, robotPos, cfg, robotHeading);
				break;

			case Finish:
				break;

			default:
				break;
		}

		// FRIZY_LOG(LOG_DEBUG, "heading_align_state:%d",heading_align_state);
		return heading_align_state;
	}

	void dwa::reset_path_planner(void)
	{

	}
	long long CurrentTime()
    {
        struct timeval time;

        gettimeofday(&time,NULL);

        return ((long long)time.tv_sec * 1000000 + (long long)time.tv_usec);
    }

	void dwa::call_dwa()
	{

	}
	bool dwa::start_path_planner(Sensor& sensor, Grid& cur, vector<pair<float, float> >& astarArr)
	{
			long long startTime ;
			startTime =	CurrentTime();
			recordmap_obstaclesPos.clear();
			dymap_obstaclesPos.clear();
			if(sensor.bump != 0 || sensor.obs != 0)
			{
				init();
				return 1;
			}

			for(int x = cur.x - 2; x <= cur.x + 2; x++)
			{
				for(int y = cur.y - 2; y <= cur.y + 2; y++)
				{
					if(_maze.GetMapState(x, y, 2) == 2)
					{
					// 	if(x>=0&y>=0)
					// 	obstaclesPos.push_back({x*0.15-0.075, y*0.15-0.075});
					// 	if(x>=0&y<0)
					// 	obstaclesPos.push_back({x*0.15-0.075, y*0.15+0.075});
					// 	if(x<0&y>=0)
					// 	obstaclesPos.push_back({x*0.15+0.075, y*0.15-0.075});
					// 	if(x<0&y<0)
					// 	obstaclesPos.push_back({x*0.15+0.075, y*0.15+0.075});

					if(x ==cur.x && y ==cur.y)
					{

					}
					// continue;
					else
					{
						recordmap_obstaclesPos.push_back({x*0.15, y*0.15});
					}
					}
				}
			}
			
			// FRIZY_LOG(LOG_DEBUG,"dynamicMap->size = %d",dynamicMap->size);
			// for(int i = 0;i< dynamicMap->size;i++)
			// {
				
			// 	dymap_obstaclesPos.push_back({dynamicMap->dyMapPointcloud[i].x, dynamicMap->dyMapPointcloud[i].y});
			// }
			FRIZY_LOG(LOG_DEBUG,"recordmap_obstaclesPos.size() = %d",recordmap_obstaclesPos.size());
			// FRIZY_LOG(LOG_DEBUG,"dymap_obstaclesPos.size() = %d",dymap_obstaclesPos.size());
			// robotData.x = raw_share_pose.x;
			// robotData.y = raw_share_pose.y;
			// float cur_forward = cur.forward*_Pi/180;
			
			robotData.x = x_ ;
			robotData.y = y_ ;
			FRIZY_LOG(LOG_INFO,"the astarArr.back().first = %f,astarArr.back().second = %f",astarArr.back().first,astarArr.back().second);
			if(cur.x == road_aim.x && cur.y ==road_aim.y )
			{
				FRIZY_LOG(LOG_INFO,"dwa run finished",astarArr.back().first,astarArr.back().second);
				init();
				return 1;
			}

			if(std::hypot(robotData.x - astarArr.back().first, robotData.y - astarArr.back().second) <= 0.08
				|| (_maze.GetMapState(road_aim.x,road_aim.y,2) == 2 && abs(cur.x - road_aim.x)+abs(cur.y - road_aim.y) < 2))
			{
				{
					local_aim.x = road_aim.x;
					local_aim.y = road_aim.y;
					if(last_caculate_state == false)
					{
						local_aim = CaculateAngle(cur,local_aim);
					 	last_caculate_state = true;
					}
					localplanning_motion.WheelControl(sensor,cur,local_aim);
					FRIZY_LOG(LOG_INFO,"dwa near finished",astarArr.back().first,astarArr.back().second);
					// init();

					return 1;
				}
			}

			// 转换位置和角度

			// 更新段路径结束位置
			updataGoalPos(astarArr, goalPos, robotData,angle_, dwaParamConfig);//cur.forward
			if(heading_align_state != Finish||heading_align_state == FirstAlignment)
			{
				FRIZY_LOG(LOG_WARNING,"do heading run");
				return 1;
			}
			// updataGoalPos(astarArr, goalPos, robotData,cur_forward, dwaParamConfig);
			// FRIZY_LOG(LOG_DEBUG,"updataGoalPos ok");
			FRIZY_LOG(LOG_DEBUG,"cur.x*0.15 = %f ,cur.y*0.15 = %f, goalPos.x() = %f ,goalPos.y() = %f ",cur.x*0.15 , cur.y*0.15, goalPos.x(), goalPos.y());
			// if(hypot(robotData.x - goalPos.x(), robotData.y - goalPos.y())<0.25 && hypot(goalPos.x() - x_, goalPos.y() - y_) > 0.04 )
			// if((hypot(cur.x*0.15 - goalPos.x(), cur.y*0.15 - goalPos.y()) <0.25 && hypot(cur.x*0.15 - goalPos.x(), cur.y*0.15 - goalPos.y())  > 0.04)
			// 	||(hypot(robotData.x - goalPos.x(), robotData.y - goalPos.y())<0.25 && hypot(goalPos.x() - x_, goalPos.y() - y_) > 0.04 ))	
			if(hypot(cur.x*0.15 - goalPos.x(), cur.y*0.15 - goalPos.y()) <0.05)
			{
				fast_dwa_turn_index = false;
			}
			if(fast_dwa_turn_index == true )

			{
					FRIZY_LOG(LOG_DEBUG,"fast dwa road mode");
					FRIZY_LOG(LOG_DEBUG,"hypot(cur.x*0.15 - goalPos.x(), cur.y*0.15 - goalPos.y()) = %f",hypot(cur.x*0.15 - goalPos.x(), cur.y*0.15 - goalPos.y()));
					
					local_aim.x = round_doubel(goalPos.x()*100/15);
					local_aim.y = round_doubel(goalPos.y()*100/15);
					// FRIZY_LOG(LOG_DEBUG,"goalPos.x() = %f , loalPos.y() = %f,int(goalPos.x()*100/15) = %d, int(goalPos.y()*100/15)=%d ",goalPos.x(), goalPos.y(),int(goalPos.x()*100/15),int(goalPos.y()*100/15));					
					// FRIZY_LOG(LOG_DEBUG,"local_aim.x = %d , local_aim.y = %d",local_aim.x, local_aim.y);
					local_aim = CaculateAngle(cur,local_aim);

					localplanning_motion.WheelControl(sensor,cur,local_aim);
					FRIZY_LOG(LOG_DEBUG,"距离终点位置 = %f ",std::hypot(goalPos.x() - x_, goalPos.y() - y_));
					return 1;
			}


			else if (fast_dwa_turn_index == false)
			{
			// 获取当前的两轮速度
			moveData.leftspeed = sensor.leftw / 1000.0;
			moveData.rightspeed = sensor.rightw / 1000.0;

			FRIZY_LOG(LOG_DEBUG,"(sensor.leftw ,sensor.rightw): %f %f", moveData.leftspeed, moveData.rightspeed);
			// 将当前两轮速度转换为机器人的运动速度和角速度

			FRIZY_LOG(LOG_DEBUG,"(cur_x,cur_y,angle_): %f %f %f", x_, y_,angle_*180/_Pi);
			// FRIZY_LOG(LOG_WARNING,"(cur_x,cur_y,cur_theta): %f %f %f", robotData.x, robotData.y,cur_forward);

			robotData = wheelSpd2RobotSpd(moveData, dwaParamConfig, angle_, x_, y_);// cur.forward, cur.x, cur.x
			// robotData = wheelSpd2RobotSpd(moveData, dwaParamConfig, cur_forward, robotData.x, robotData.y);
			// DWA算法预测了一组速度和角速度
			long long _startTime =	CurrentTime();
			// cout<<"dwaControl start time ="<< _startTime<< endl;
			predInfo = dwaControl(robotData, dwaParamConfig, goalPos, recordmap_obstaclesPos,dymap_obstaclesPos, astarArr);
			// cout<<"dwaControl caculate time ="<< _endTime -_startTime<< endl;
			FRIZY_LOG(LOG_DEBUG,"预测速度为　v = %f ,w =%f",predInfo.first.v,predInfo.first.w);
			
			if((predInfo.first.v < 0.02f&&predInfo.first.v > -0.02f &&predInfo.first.w > -0.02f &&predInfo.first.w < 0.02f)||(predInfo.first.v == 0 && last_wspeed == predInfo.first.w ))
			{
				
				dwa_sum_times++;
				if(dwa_sum_times>40)
				{
					if(hypot(cur.x*0.15 - goalPos.x(), cur.y*0.15 - goalPos.y()) <0.25 && hypot(cur.x*0.15 - goalPos.x(), cur.y*0.15 - goalPos.y())  > 0.04 )
					{
							FRIZY_LOG(LOG_DEBUG," set fast dwa road mode");
							fast_dwa_turn_index = true;
					}
					else	
					{
					init();
					FRIZY_LOG(LOG_ERROR,"dwa planer caculate failed, canot reach the aim");
					dwa_sum_times =0;
					return false;
					}
				}
			}
			else
			{
				dwa_sum_times =0;
			}
			
			last_wspeed = predInfo.first.w;
			// 将DWA算法预测的速度和角速度转化为两个轮子的速度
			moveData = robotSpd2WheelSpd(predInfo.first, dwaParamConfig);
			//predInfo.first.v = 0.0f;
			//predInfo.first.w = 0.0f;

			// 将两轮速度发送给底盘
			chassisbaseControl.chassisSpeed(moveData.leftspeed * 1000, moveData.rightspeed * 1000, 1);
			
			FRIZY_LOG(LOG_DEBUG,"moveData.leftspeed = %f ,moveData.rightspeed = %f ,将两轮速度发送给底盘",moveData.leftspeed * 1000,moveData.rightspeed * 1000);
			// 到达分段路径的末端
			FRIZY_LOG(LOG_DEBUG,"距离终点位置 = %f ",std::hypot(goalPos.x() - x_, goalPos.y() - y_));
			}

			// if( hypot(robotData.x - goalPos.x(), robotData.y - goalPos.y())  <= 0.04 )
			// if( hypot(cur.x*0.15 - goalPos.x(), cur.y*0.15 - goalPos.y())  <= 0.04 )
			if(( hypot(cur.x*0.15 - goalPos.x(), cur.y*0.15 - goalPos.y())  <= 0.05 )||( hypot(robotData.x - goalPos.x(), robotData.y - goalPos.y())  <= 0.05 ))
			{
				FRIZY_LOG(LOG_INFO,"update the next destination");
				// exit(0);
				// 更新目标位置
				// 推算航距
				updataGoalPos_index = true;
				
				updataGoalPos(astarArr, goalPos, robotData, angle_, dwaParamConfig);
				// 读取ｓｌａｍ位置
				// updataGoalPos(astarArr, goalPos, robotData, cur_forward, dwaParamConfig);
			}
			//	到达astar路径末端
			// if(std::hypot(robotData.x - astarArr.back().first, robotData.y - astarArr.back().second) <= 0.03)
			// {
				
			// 	// chassisbaseControl.chassisSpeed(0,0,1);
			// 	Grid local_aim;
			// 	local_aim.x = road_aim.x;
			// 	local_aim.y = road_aim.y;
			// 	localplanning_motion.WheelControl(sensor,cur,local_aim);
			// 	FRIZY_LOG(LOG_INFO,"the astarArr.back().first = %f,astarArr.back().second = %f, dwa run finished",astarArr.back().first,astarArr.back().second);
			// 	init();
			// 	// process = PLAN;

			// 	return 1;
			// }
			long long  endTime;
			endTime = CurrentTime();
			cout<<"start_path_planner total time ="<< endTime - startTime<< endl;
			return 1;

	}
	void dwa::dwaStart()
	{
		FRIZY_LOG(LOG_INFO,"dwa thread run ");
		Dwa_thread_ = std::make_shared<std::thread>(&dwa::chassisDwa, this);
	}
	void dwa::dwaStop()
	{
		FRIZY_LOG(LOG_INFO,"dwa thread stop ");
		Dwa_thread_->join();
		Dwa_thread_ = nullptr;

	}
    Grid dwa::CaculateAngle(Grid cur,Grid aim)
    {
        //0
        if (aim.x - cur.x > 0 && aim.y - cur.y == 0)
        {			
            aim.forward = 0;	     
        }
        //90
        else if (aim.x - cur.x == 0 && aim.y - cur.y < 0)
        {
            aim.forward = 90;
        }
        //270
        else if (aim.x - cur.x == 0 && aim.y - cur.y > 0)
        {
            aim.forward = 270;
        }		
        //180					
        else if (aim.x - cur.x < 0 && aim.y - cur.y == 0)
        {
            aim.forward = 180;
        }
        //other angle
        else if (abs(aim.x - cur.x) != 0 && abs(aim.y - cur.y) != 0)
        {
            float_t tempcan = float_t(abs(aim.y - cur.y))/abs(aim.x - cur.x);
            
            if (aim.y - cur.y > 0 && aim.x - cur.x > 0)
            {
                aim.forward = 360 - atan(tempcan) * 180 / 3.14;
            }
            if (aim.y - cur.y > 0 && aim.x - cur.x < 0)
            {
                aim.forward = 180 + atan(tempcan) * 180 / 3.14;
            }
            if (aim.y - cur.y < 0 && aim.x - cur.x < 0)
            {
                aim.forward = 180 - atan(tempcan) * 180 / 3.14;
            }
            if (aim.y - cur.y < 0 && aim.x - cur.x > 0)
            {
                aim.forward = atan(tempcan) * 180 / 3.14;
            }		
            printf("jizhi.%f,%f\n",tempcan,atan(tempcan));						
        }		
        else
        {
            FRIZY_LOG(LOG_DEBUG,"goup");
            aim.forward = cur.forward;
        }

        return aim;
    }
	void dwa::chassisDwa()
	{
		// FRIZY_LOG(LOG_DEBUG,"DWA thread start");
		// while(1)
		// {
    //         if(GLOBAL_CONTROL != WHEEL_RUN ||)
    //         {
    //             // FRIZY_LOG(LOG_DEBUG, "GLOBAL_CONTROL:%d", GLOBAL_CONTROL);
    //             usleep(200 * 1000);
    //             continue;
    //         }	
		// 				else
		// 				{

		// 				}
								
		// }

	}
	void dwa::init()
	{
		offset = 0;
		heading_align_state = FirstAlignment;
		last_caculate_state = false;
		turn_finished_index = true;
	}

	//Column mode
    aroundColumn::aroundColumn(){}
    aroundColumn::~aroundColumn(){}

}

