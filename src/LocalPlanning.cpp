/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  :
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-18 16:30:07
 * @Project      : UM_path_planning
 */

#include "navigation_algorithm/LocalPlanning.h"
#include "navigation_algorithm/RoadPlanning.h"
#include <fstream>
#include <vector>
#include <iostream>
#include <sys/time.h>

using namespace std;

// extern double x_ ;			 //Odometer x coordinates
// extern double y_ ;			 //Odometer y coordinates
// extern float angle_;		 //Odometer angle
extern useerobot::Maze _maze;

namespace useerobot
{

extern RoadAim road_aim;

float last_wspeed = 0.0;

	Grid CaculateAngle(Grid cur,Grid aim)
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
				float tempcan = float(abs(aim.y - cur.y))/abs(aim.x - cur.x);
				
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
				//printf("jizhi.%f,%f\n",tempcan,atan(tempcan));						
		}		
		else
		{
				FRIZY_LOG(LOG_DEBUG,"goup");
				aim.forward = cur.forward;
		}

		return aim;
	}

	// double round(double x)
	// {
  //   return (x > 0.0) ? floor(x + 0.5) : ceil(x - 0.5);
	// }

	inline bool Isfree(int x,int y){
		return _maze.GetMapState(x,y,2) != 2 && _maze.GetMapState(x,y,1) != 4;
	}


	bool caculateLine(Grid& cur,Grid& aim,float& forward){

		//Grid cur = {int(1000*RobotPos.x),int(1000*RobotPos.y)};
		//Grid aim = {int(1000*vf.first),int(1000*vf.second)};


		Grid c1 = {int(1000*cur.realx),int(1000*cur.realy)};
		Grid c2 = {int(1000*aim.x),int(1000*aim.y)};

		float tmpfor = CaculateAngle(c1,c2).forward;

		if (tmpfor >= 315 || tmpfor <= 45){
			float k = tan(tmpfor*_PI/180);

			for (int x = 1;cur.x+x <= aim.x;++x){
				int y = cur.y - round(x*k);
				if (abs(y - road_aim.y) + abs(cur.x+x - road_aim.x) < 2)
					continue;

				if (!Isfree(cur.x+x,y) || !Isfree(cur.x+x,y+1) || !Isfree(cur.x+x,y-1)){
					FRIZY_LOG(LOG_DEBUG,"cut!.%d.%d  %f",cur.x+x,y,tmpfor);
					return false;
				}	
			}
		}

		else if (tmpfor >= 135 && tmpfor <= 225){
			float k = tan(tmpfor*_PI/180);
			for (int x = 1;cur.x-x >= aim.x;++x){
				int y = cur.y + round(x*k);
				if (abs(y - road_aim.y) + abs(cur.x-x - road_aim.x) < 2)
					continue;

				if (!Isfree(cur.x-x,y) || !Isfree(cur.x-x,y+1) || !Isfree(cur.x-x,y-1)){
					FRIZY_LOG(LOG_DEBUG,"cut!.%d.%d  %f",cur.x+x,y,tmpfor);
					return false;
				}	
			}			
		}
		else if (tmpfor > 45 && tmpfor < 135){
			float k = tan(_PI/2 - tmpfor*_PI/180);
			for (int y = 1;cur.y-y >= aim.y;++y){

				int x = cur.x + round(y*k);

				if (abs(cur.y-y - road_aim.y) + abs(x - road_aim.x) < 2)
					continue;	

				if (!Isfree(x,cur.y-y) || !Isfree(x+1,cur.y-y) || !Isfree(x-1,cur.y-y)){
					FRIZY_LOG(LOG_DEBUG,"cut!.%d.%d  %f",cur.x+x,y,tmpfor);
					return false;
				}	
			}			
		}
		else{
			float k = tan(_PI/2 - tmpfor*_PI/180);
			for (int y = 1;cur.y+y <= aim.y;++y){
				int x = cur.x - round(y*k);
				if (abs(cur.y+y - road_aim.y) + abs(x - road_aim.x) < 2)
					continue;	

				if (!Isfree(x,cur.y+y) || !Isfree(x+1,cur.y+y) || !Isfree(x-1,cur.y+y)){
					FRIZY_LOG(LOG_DEBUG,"cut!.%d.%d  %f",cur.x+x,y,tmpfor);
					return false;
				}	
			}			
		}	
		
		forward = tmpfor;
		return true;	
	}


	void dwa::updataGoalPos(Grid& cur,Points& astarPath,Grid& aim)
	{
		
		int size = astarPath.size();
		float dis = 100.0;
		int index = -1;
		for (int i = 0;i < size-1;++i){
			if (fabs(astarPath[i].first-cur.realx) + fabs(astarPath[i].second-cur.realy) < dis){
				dis = fabs(astarPath[i].first-cur.realx) + fabs(astarPath[i].second-cur.realy);
				index = i;
			}
		}
		maxIndex1 = max(maxIndex1,index);
		
		index = -1;
		aim.forward = 1001;
		
		for (int i = size-1; i > maxIndex1 && i >= maxIndex2 && i > 0;--i){
			
			stage_aim = {astarPath[i].first,astarPath[i].second};

			if (!caculateLine(cur,stage_aim,aim.forward)){
				continue;
			}
			else{
				index = i;
				maxIndex2 = max(maxIndex2,index);
				aim.x = stage_aim.x,aim.y = stage_aim.y;
				FRIZY_LOG(LOG_DEBUG,"sss1.%d,%d,%d  max %d %d",index,stage_aim.x,stage_aim.y,maxIndex1,maxIndex2);
				FRIZY_LOG(LOG_DEBUG,"tmpfor.%f  %f  %d,%d",aim.forward,cur.forward,stage_aim.x,stage_aim.y);
				break;
			}
		}

		if (index == -1){
			int ye = maxIndex1 >= maxIndex2 ? maxIndex1+1 : maxIndex2;
			ye = min(ye,size-1);
			ready_aim = {astarPath[ye].first,astarPath[ye].second};
			aim.forward = CaculateAngle(cur,ready_aim).forward;
			aim.x = ready_aim.x,aim.y = ready_aim.y;
			FRIZY_LOG(LOG_DEBUG,"sss2 %d.%d  %f  max %d %d %d",ready_aim.x,ready_aim.y,aim.forward,maxIndex1,maxIndex2,ye);
			FRIZY_LOG(LOG_DEBUG,"tmpfor.%f  %f  %d,%d",aim.forward,cur.forward,ready_aim.x,ready_aim.y);
		}
	}

	bool dwa::start_path_planner(Sensor sensor,Grid cur,Points& astarArr){

			if((cur.x == road_aim.x && cur.y == road_aim.y)
				|| sensor.bump){

				FRIZY_LOG(LOG_DEBUG,"dwa run finished");
				init();				
				return true;
			}

			WheelTrans(sensor,cur,&RobotPos);
			//printf("speed1.%f,%f\n",RobotPos.v,RobotPos.w);
			dynamicWindow(&speedWindow);
			//printf("speed2.%f,%f  %f,%f\n",speedWindow.v_max,speedWindow.v_min,speedWindow.w_max,speedWindow.w_min);

			FRIZY_LOG(LOG_DEBUG,"initDwa.%d",initDwa);

			updataGoalPos(cur,astarArr,_aim);

			switch (initDwa)
			{
			case 1:{
				FRIZY_LOG(LOG_DEBUG,"kaishila");
				start = {cur.x,cur.y};
				
				end = {astarArr.back().first,astarArr.back().second};
				firstPoint = _aim;
				initDwa = 2;
				return true;
			}
			case 0:{
				if (fabs(_aim.forward - cur.forward) < 5 || fabs(_aim.forward - cur.forward) > 355){
					FRIZY_LOG(LOG_DEBUG,"nima1");
					motionDwa.ClearPid();
					firstPoint = _aim;
					initDwa = 2;
				}else if (fabs(_aim.x-cur.realx) + fabs(_aim.y-cur.realy) < 0.5){
					FRIZY_LOG(LOG_DEBUG,"nima3");
					motionDwa.ClearPid();
					firstPoint = _aim;
					firstPoint.forward = cur.forward;
					initDwa = 2;										
				}
				else{
					break;
				}
			}
			case 2:{
				if (firstPoint.x != _aim.x || firstPoint.y != _aim.y){
					FRIZY_LOG(LOG_DEBUG,"nima2");
					initDwa = 0;
					break;
				}
				else{
					if (abs(cur.x-end.x) + abs(cur.y-end.y) <= 1){
						FRIZY_LOG(LOG_DEBUG,"lastlast.%d.%d",end.x,end.y);
						
						motionDwa.WheelControl(sensor,cur,end);
					}else{
						FRIZY_LOG(LOG_DEBUG,"firstPoint.%d.%d  %f",firstPoint.x,firstPoint.y,firstPoint.forward);
						motionDwa.WheelControl(sensor,cur,firstPoint);
					}
					return true;
				}
			}
			default:
				break;
			}
			
			FRIZY_LOG(LOG_DEBUG,"aimfor %d %d %f  %d %d",_aim.x,_aim.y,_aim.forward,maxIndex1,maxIndex2);

			Trajectory best_traj;

			priority_queue<Node> q;


			float k1,k2;
			float det = fabs(cur.forward-_aim.forward) > 180 ? 360-fabs(cur.forward-_aim.forward):fabs(cur.forward-_aim.forward);
			k1 = det/90 - 1;
			k2 = 1.0;


			// if (abs(cur.forward - aimfor) <= 90){
			// 	k1 = -1.0,k2 = 0.5;
			// }else{
			// 	k1 = 1.0,k2 = 0.5;
			// }
			
			for (float v = speedWindow.v_min; v <= speedWindow.v_max; v += cfg.v_reso){
					for (float w = speedWindow.w_min; w <= speedWindow.w_max; w += cfg.yawrate_reso){
					
					Trajectory traj;

					s_left = s_right = 0;
					s_left = static_cast<int>(1000*(v + w * cfg.wheel_L / 2.0));
					s_right = static_cast<int>(1000*(v - w * cfg.wheel_L / 2.0));

					if (!PredictTrajectory(v,w,traj)){
						FRIZY_LOG(LOG_DEBUG,"bbb.%d.%d",s_left,s_right);
						continue;
					}
					
						
					if ((s_left < 120 && s_right < 120 && s_left >= 0 && s_right >= 0)
							|| (s_left + s_right == 0 && abs(s_left) > 130)
							|| s_left > 250 || s_right > 250 || abs(s_left - s_right) > 240)
						continue;


					float v_forward = 0.0;

					int v_index = 60;


					if (_aim.forward < 1000){
						for (int i = 0;i < traj.size();++i){
							
							Grid c1 = {int(1000*traj[i].x/0.15),int(1000*traj[i].y/0.15)};
							Grid c2 = {1000*stage_aim.x,1000*stage_aim.y};

							float tmpfor = CaculateAngle(c1,c2).forward;

							float tmp = fabs(fabs(traj[i].theta*180/_PI - tmpfor) - 180);

							if (tmp > v_forward){
								v_forward = tmp;
								v_index = i;
								if (fabs(v_forward - 180) < 1)
									break;
							}
						}
					}

					node = {s_left,s_right,v_forward,v_index};
					q.push(node);

					// float value = 5*v_forward - v_index
					// + k1*abs(s_left-s_right) + k2*(abs(s_left)+abs(s_right));					
					
					// FRIZY_LOG(LOG_DEBUG,"v: %f  %d  %f  %f s: %d  %d  %f  value: %f"
					// ,v_forward,v_index,traj[v_index].v,traj[v_index].w
					// ,s_left
					// ,s_right
					// ,traj[v_index].theta*180/_PI
					// ,value);
					
					// //float value = 10 - fabs(w) - fabs(w-lastspeedW) - 20 * fabs(0.15-v) + v_forward - v_index;

					// if (value > max_value){
					// 		max_value = value;
					// 		best_traj = traj;
					// }  

				}
			}

			vector<Node> vec;
			const float threshold = 178;
			if (q.empty() || q.top().v_for < threshold){
					FRIZY_LOG(LOG_DEBUG,"fail.%d",best_traj.size());
					init();
					return false;
			}
			else{
				while(!q.empty() && q.top().v_for > threshold){
						vec.push_back(q.top());
						q.pop();
					}
			}
			
			
			float max_value = std::numeric_limits<float>::min(); // > 0
			max_value = -1000;
			for (int i = 0;i < vec.size();++i){

				float value = -vec[i].v_i
				+ k1*abs(vec[i].sl-vec[i].sr) + k2*(vec[i].sl+vec[i].sr);					
				
				FRIZY_LOG(LOG_DEBUG,"v: %f  %d  s: %d  %d  value: %f"
					,vec[i].v_for,vec[i].v_i,vec[i].sl,vec[i].sr,
					value);
				
				//float value = 10 - fabs(w) - fabs(w-lastspeedW) - 20 * fabs(0.15-v) + v_forward - v_index;

				if (value > max_value){
					max_value = value;
					s_left = vec[i].sl;
					s_right = vec[i].sr;
				}  
			}
			 
			FRIZY_LOG(LOG_DEBUG,"last: %d,%d",s_left,s_right);

			// lastspeedW = best_traj.back().w;
			// float lastv = best_traj.back().v;
			// float lastw = best_traj.back().w;
			// s_left = static_cast<int>(1000*(lastv + lastw * cfg.wheel_L / 2.0));
			// s_right = static_cast<int>(1000*(lastv - lastw * cfg.wheel_L / 2.0));
			// FRIZY_LOG(LOG_DEBUG,"last: %f  %f  %d,%d ",lastv,lastw,s_left,s_right);

			chassisDwa.chassisSpeed(s_left,s_right,100);
			return true;

	}

	bool dwa::PredictTrajectory(float v,float w,Trajectory& traj)
	{
			float time = 0.0f;

			// Trajectory traj = {x_init};

			// RobotData x = x_init;

			RobotData _new;
			_new = {RobotPos.x,RobotPos.y,RobotPos.theta};

			while (time <= cfg.predict_time)
			{
				_new.theta += w * cfg.dt;
				if (_new.theta >= 2*_PI) _new.theta -= 2*_PI;
				if (_new.theta < 0) _new.theta += 2*_PI;
				_new.x += v * cfg.dt * cos(_new.theta);
				_new.y -= v * cfg.dt * sin(_new.theta);
				_new.v = v;
				_new.w = w;

				//Grid tmp;

				//tmp.realx = x_new.x,tmp.realy = x_new.y;
				int x1 = _new.x/0.15;
				int y1 = _new.y/0.15;

				int x2 = _new.x/0.15 >= 0 ? x1+1:x1-1;
				int y2 = _new.y/0.15 >= 0 ? y1+1:y1-1;

				// for (int x = -1;x <= 1;++x)
				// {
				// 		for (int y = -1;y <= 1;++y){
				// 				tmp.x += x;
				// 				tmp.y += y;
				// 				if (_maze.GetMapState(tmp.x,tmp.y,2) == 2){
				// 						printf("okk.%f,%f  %f,%f\n",tmp.realx,tmp.realy,x_new.v,x_new.w);
				// 						return false;
				// 				}
				// 		}
				// }
				traj.push_back(_new);
				time += cfg.dt;
			}

			return true;
	}



}