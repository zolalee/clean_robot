/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-07 17:22:04
 * @Project      : UM_path_planning
 */

#include "navigation_algorithm/RoadPlanning.h"
#include "navigation_algorithm/UTurnPlanning.h"

using namespace useerobot;
extern Maze _maze;
namespace useerobot
{   extern bool cleanTaskOverIndex;
    extern PRO process;
    extern vector <Grid> rollOb;   
    extern int justwind;
    extern vector <boundary> boundArr;
    extern boundary limit; 
    extern vector <Grid> boundPoint;
    extern int tmpAdd;
    extern int deta;
    int lasttake;
    static RoadAim Null_aim;
    RoadAim last_road_aim;  
    RoadAim road_aim;
    vector <Grid> deleteOb;
    UTurnPlanning planRoad;

    RoadPlanning::RoadPlanning(/* args */)
    {
    }
   
    RoadPlanning::~RoadPlanning()
    {

    }
    
    void RoadPlanning::init()
    {
        deleteOb.clear();
        roadState = roadIdle;
        astarAgain = 0;
        ewall.x = 1000;
        ewallTime = 0;
        bumpCount = 0;
        dewallSign = 0;    
        call_recharge_index = false;
        
    }    
    void RoadPlanning::SetroadAim(RoadAim aim)
    {
        road_aim.x = aim.x;
        road_aim.y = aim.y;
        road_aim.kind = aim.kind;
        recharge_first_index = 0;
        
    }   

    Grid RoadPlanning::ConAngle(Grid cur,Grid aim)
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
   
    int8_t RoadPlanning::BroadArea(int16_t x,int16_t y)
    {
		int8_t temps = 0;
		for (int i = 1;i < 4;i++)
		{
            for (int j = 0;j < 3;j++)
            {
                if (_maze.GetMapState(x+i,y+j,2) != 0)
                {
                        temps++;
                        break;
                }
            }
		}
		
		for (int i = 1;i < 4;i++)
		{
            for (int j = 0;j < 3;j++)
            {
                if (_maze.GetMapState(x+i,y-j,2) != 0)
                {
                        temps++;
                        break;
                }
            }
		}		
		
		
		for (int i = 1;i < 4;i++)
		{
            for (int j = 0;j < 3;j++)
            {
                if (_maze.GetMapState(x-i,y-j,2) != 0)
                {
                        temps++;
                        break;
                }
            }
		}		
		
		
		for (int i = 1;i < 4;i++)
		{
            for (int j = 0;j < 3;j++)
            {
                if (_maze.GetMapState(x-i,y+j,2) != 0)
                {
                        temps++;
                        break;
                }
            }
		}		
	
		if (temps < 4 && x < limit.up &&  x > limit.down && y < limit.left && y > limit.right)
			return 1;	
		else
			return 0;
    }   
   
    void RoadPlanning::NullRunning(Sensor sensor,Grid cur)
    {
        if (deleteOb.size() > 1 && deleteOb[0].x != 1000)
        {
            FRIZY_LOG(LOG_DEBUG,"shanchu");
            for (int i = 0; i < deleteOb.size();i++)
            {
                FRIZY_LOG(LOG_DEBUG,"shan.%d.%d",deleteOb[i].x,deleteOb[i].y);
                _maze.InputRecord(deleteOb[i].x,deleteOb[i].y,1);
                
            }
            deleteOb[0].x = 1000;
            roadState = roadIdle;
            return;
        }   

        _aim.x = road_aim.x,_aim.y = road_aim.y;
        
        if (_aim.forward == 1000)
        {
            if (Null_aim.x == _aim.x && Null_aim.y == _aim.y && Null_aim.kind == none)
            {
                FRIZY_LOG(LOG_DEBUG,"dewallSign = 1");
                dewallSign = 1;
            }
            else
            {
                FRIZY_LOG(LOG_DEBUG,"dewallSign = 0");
                Null_aim.x = road_aim.x,Null_aim.y = road_aim.y,Null_aim.kind = none;
                dewallSign = 0;
            }
        }

        if (IsWall() == 0 && dewallSign == 0)
        {
            if (_aim.forward == 1000 || cur.x == _aim.x || cur.y == _aim.y)
                _aim = ConAngle(cur,_aim);
            
            motionRoad.WheelControl(sensor,cur,_aim);

            if (sensor.bump != 0)
            {
                printf("impossible!\n");
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                return;
            }
        }
        else
        {
            FRIZY_LOG(LOG_DEBUG,"ewalling.%d.%d.%f.%d",ewall.x,ewall.y,ewall.forward,ewallTime);

            if (abs(cur.x - ewall.x) + abs(cur.y - ewall.y) > 2 
                && BroadArea(cur.x,cur.y) == 1
                && road_aim.kind == searchUnclean)
			{
                printf("zhijietuichu\n");

                do{
                    StopWallFollow();
                    chassisRoad.GetSensor(&sensor);
                    usleep(10 * 1000);
                }while (sensor.leftw != 0 || sensor.rightw != 0); 

                _trouble.type = nothing;
                ewallTime = 0;
                
                road_aim.kind = idle;
                planRoad.PlanSearch(road_aim,-1);
                process = PLAN;
                return;  						
			}
            if ((abs(cur.x - ewall.x) + abs(cur.y - ewall.y) > 15 * sensor.size && ewall.x != 1000)
                || ewallTime > 100 * 3
                || _trouble.type == windcolumn
                || road_aim.kind == searchLast)
            
            {
                _trouble.type = nothing;
                FRIZY_LOG(LOG_DEBUG,"out le.%d.%d",cur.x,cur.y);
                do{
                    StopWallFollow();
                    chassisRoad.GetSensor(&sensor);
                    usleep(10 * 1000);
                }while (sensor.leftw != 0 || sensor.rightw != 0);  

                ewallTime = 0;
                
                if (road_aim.kind == recharge)
                {
                    FRIZY_LOG(LOG_DEBUG,"simida1");
                    roadState = roadIdle;
                    roadDwa.init();
                    return;  
                }
                else if (road_aim.kind == column)
                {
                    deta = cur.addAngle - tmpAdd;
                    for (Grid tmp : boundPoint)
                        tmp.addAngle = tmp.addAngle + deta; 

                    FRIZY_LOG(LOG_DEBUG,"justwind = 1.%d",deta);
                    justwind = 1;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);               
                    process = BOUND;
                    return;                    
                }
                else if (road_aim.kind == searchLast){
                    FRIZY_LOG(LOG_DEBUG,"simida3");
                    planRoad.PlanSearch(road_aim,1);
                    process = PLAN;
                    return;                         
                }
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"the kind.%d",road_aim.kind);      
                    planRoad.PlanSearch(road_aim,0);
                    process = PLAN;
                    return;                       
                }
            
            }
            ewallTime ++;  
            StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);          
        }
        return;
   
    }

    void RoadPlanning::DwaRunning(Sensor sensor,Grid cur)
    {
        vector <Sensor> sss;
        int count1,count2;

        while (count2)
        {
            count2 --;
            chassisRoad.chassisSpeed(sss[count1].leftw,sss[count1].rightw,1);
            if (count2 == 0)
                count1 ++,count2 = 10;
            return;
        }
    }

    #define BUMP_MAX 8
    int RoadPlanning::ArriveRoad(Sensor sensor,Grid cur)
    {
        if (last_road_aim.x != road_aim.x || last_road_aim.y != road_aim.y)
        {
            FRIZY_LOG(LOG_DEBUG,"aim change");
            bumpCount = 0;
            astarAgain = 1;
            roadState = roadIdle;
            Null_aim.kind = idle;
        }
        last_road_aim.x = road_aim.x;
        last_road_aim.y = road_aim.y;

        FRIZY_LOG(LOG_DEBUG,"road_aim.%d.%d.%d.%d.%d",road_aim.x,road_aim.y,road_aim.kind,dewallSign,roadState);

        if (road_aim.kind == searchLast && IsWall() == 2){
            FRIZY_LOG(LOG_DEBUG,"raoing");
        }
        if(road_aim.kind == recharge && hypot(road_aim.x-cur.x,road_aim.y-cur.y)<0.25)
        {
           sum_recharge_times++;

           if(sum_recharge_times >20*60)
           {
                sum_recharge_times = 0;

                FRIZY_LOG(LOG_DEBUG,"can not reach the recharge seat stay around"); 
                chassisRoad.chassisSpeed(0,0,1);
                cleanTaskOverIndex = false;
                call_recharge_index =true;                
                recharge_first_index++;
                FRIZY_LOG(LOG_DEBUG,"recharge_first_index = %d",recharge_first_index);               
           }
           
        }
        if(road_aim.kind == recharge && hypot(road_aim.x-cur.x,road_aim.y-cur.y)>0.25)
        {
            sum_recharge_times = 0;
        }
        if (cur.x == road_aim.x && cur.y == road_aim.y)
        {
            switch (road_aim.kind)
            {
            case recharge:
                FRIZY_LOG(LOG_DEBUG,"finally over");
                chassisRoad.chassisSpeed(0,0,1);
                cleanTaskOverIndex = false;
                call_recharge_index =true;                
                recharge_first_index++;
                FRIZY_LOG(LOG_DEBUG,"recharge_first_index = %d",recharge_first_index);
                break;

            case searchBound:
                FRIZY_LOG(LOG_DEBUG,"daodale2");
                process = BOUND;
                break;

            case searchUnclean:
                FRIZY_LOG(LOG_DEBUG,"daodale1");
                process = PLAN;
                break;

            case searchInside:
                FRIZY_LOG(LOG_DEBUG,"daodale0");
                process = PLAN;
                break;
            

            case backBound:
                FRIZY_LOG(LOG_DEBUG,"daodale3");
                deta = cur.addAngle - tmpAdd;
                for (Grid tmp : boundPoint)
                    tmp.addAngle = tmp.addAngle + deta; 
                process = BOUND;
                break;

            case column:
            {
                FRIZY_LOG(LOG_DEBUG,"daodale4");

                if (rollOb.back().x == cur.x && rollOb.back().y == cur.y)
                {
                    deta = cur.addAngle - tmpAdd;
                    for (Grid tmp : boundPoint)
                        tmp.addAngle = tmp.addAngle + deta; 

                    FRIZY_LOG(LOG_DEBUG,"justwind = 1.%d",deta);
                    // boundArr.push_back(limit);
                    // boundPoint.clear();
                    // process = PLAN;
                    // road_aim.kind = searchInside;
                    // planRoad.PlanSearch(road_aim,-1);  
                    justwind = 1;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);               
                    process = BOUND;
                    return 1;
                }
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"bbqle2");
                    for (int i = 0;i < rollOb.size();i++)
                    {
                        if (rollOb[i].x == cur.x && rollOb[i].y == cur.y)
                        {
                            
                            if (i + 5 < rollOb.size())
                                road_aim.x = rollOb[i+5].x,road_aim.y = rollOb[i+5].y;
                            else
                                road_aim.x = rollOb.back().x,road_aim.y = rollOb.back().y;

                            FRIZY_LOG(LOG_DEBUG,"chulai.%d.%d",road_aim.x,road_aim.y);

                            break;
                        }
                    }
                   
                }
                break;
            }

            case zoning:{
                FRIZY_LOG(LOG_DEBUG,"daodale5");
                process = PLAN;
                break;
            }
            case appoint:
            {
                FRIZY_LOG(LOG_DEBUG,"daodale7");
                process = POINT;
                break;
            }
            case searchLast:{
                FRIZY_LOG(LOG_DEBUG,"daodale6");
                process = PLAN;
                break;                
                //到达假障碍点
                // if (lasttake != IDLE && iswall == 0)
                // {
                //     if (a == x && b == y)
                //     {
                //         printf("5s.\n");
                //         if (bumpstate == 1
                //             || (GetMapState(x,y+1) != 0 && GetMapState(x,y-1) == 0 && GetMapState(x+1,y) == 0 && GetMapState(x-1,y) == 0))
                //         {
                //             s5.x = x,s5.y = y;
                //             printf("s52.%d.%d\n",s5.x,s5.y);						
                //             iswall = 1;	
                //         }
                //         if (GetMapState(x,y+1) == 0)
                //         {
                //             printf("w1\n");
                //             pointTemp.x = x;
                //             pointTemp.y = y + 1;
                //             return pointTemp;					
                //         }
                //         if (GetMapState(x,y-1) == 0)
                //         {
                //             printf("w2\n");
                //             pointTemp.x = x;
                //             pointTemp.y = y-1;
                //             return pointTemp;					
                //         }				
                //         if (GetMapState(x+1,y) == 0)
                //         {
                //             printf("w3\n");
                //             pointTemp.x = x + 1;
                //             pointTemp.y = y;
                //             return pointTemp;					
                //         }		
                //         if (GetMapState(x-1,y) == 0)
                //         {
                //             printf("w4\n");
                //             pointTemp.x = x - 1;
                //             pointTemp.y = y;
                //             return pointTemp;					
                //         }								
                //     }
                //     else
                //     {
                //         printf("sssss5 = 0\n");
                //         sssss5 = 0;
                //     }
                // }



            }           

            default:
                break;
            }

            if (IsWall() != 0)
            {
                do{
                    FRIZY_LOG(LOG_DEBUG,"outroad1");
                    StopWallFollow();
                    chassisRoad.GetSensor(&sensor);
                    usleep(20 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                   
            }
            roadDwa.init();
            roadState = roadIdle;
            return 1;
        }     

        if (sensor.bump || sensor.obs)
        {
  
            if (roadState == roadTrue)
              bumpCount ++;

            switch (road_aim.kind)
            {
            case recharge:
            {
                FRIZY_LOG(LOG_DEBUG,"recharge bump");
                chassisRoad.chassisSpeed(0,0,1);
                break;
            }    
            case searchLast:
            {
                FRIZY_LOG(LOG_DEBUG,"remove6");
                if (abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) > 0 
                     && abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) < sensor.size + 1)
                {

                    FRIZY_LOG(LOG_DEBUG,"5s wall");
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);  

                    roadDwa.init();
                    roadState = roadIdle;

                    return 1;
                }

                if (bumpCount > BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"bumpmax6");
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,1);  
                    process = PLAN;
                    return 1;                     
                }
                break;
            }   

            case searchUnclean:
            {
                FRIZY_LOG(LOG_DEBUG,"remove1");
                if (abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) > 0 
                     && abs(road_aim.x - cur.x) + abs(road_aim.y - cur.y) < sensor.size + 1)
                {

                    printf("into wall\n");

                    if (IsWall() != 0)
                    {
                        do{
                            FRIZY_LOG(LOG_DEBUG,"outroad2");
                            StopWallFollow();
                            chassisRoad.GetSensor(&sensor);
                            usleep(10 * 1000);
                        }while (sensor.leftw != 0 || sensor.rightw != 0);                   
                    }  

                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,2);  
                    process = PLAN;
                    return 1;
                }

                if (bumpCount > BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"bumpmax1");
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,1);  
                    process = PLAN;
                    return 1;                     
                }
                if (IsWall() == 0)
                {
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,0);  
                    process = PLAN;
                    return 1;        
                }          
                break;
            }
            case searchBound: 
            {
                FRIZY_LOG(LOG_DEBUG,"remove2");
                if (bumpCount > BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"bumpmax2");
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,1);  
                    process = PLAN;
                    return 1;                     
                }  
                if (IsWall() == 0)
                {
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,0);  
                    process = PLAN;
                    return 1;        
                }                
                break; 
            }
            case searchInside:
            {
                FRIZY_LOG(LOG_DEBUG,"remove3");
                if (bumpCount > BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"bumpmax3");
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,1);  
                    process = PLAN;
                    return 1;                     
                }                 
                if (IsWall() == 0)
                {
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,-1);  
                    process = PLAN;
                    return 1;        
                } 
            }
            case backBound:
            {
                FRIZY_LOG(LOG_DEBUG,"remove4");

                if (bumpCount > BUMP_MAX)
                {
                    FRIZY_LOG(LOG_DEBUG,"manle5");
                    roadDwa.init();
                    roadState = roadIdle;
                    boundArr.push_back(limit);
                    boundPoint.clear();
                    road_aim.kind = searchInside;
                    planRoad.PlanSearch(road_aim,-1);  
                    process = PLAN;
                    return 1;                     
                }   

                if (IsWall() == 0)
                {
                    roadDwa.init();
                    roadState = roadIdle;
                    planRoad.PlanSearch(road_aim,-1);  
                    process = PLAN;
                    return 1;        
                } 
                break;
            }
            case column:
            {
                deta = cur.addAngle - tmpAdd;
                for (Grid tmp : boundPoint)
                    tmp.addAngle = tmp.addAngle + deta; 

                FRIZY_LOG(LOG_DEBUG,"justwind = 1.%d",deta);
                justwind = 1;
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);               
                process = BOUND;
                return 1;
            }

            case zoning:{
                FRIZY_LOG(LOG_DEBUG,"zoning remove");
                chassisRoad.chassisSpeed(0,0,1);
                //process = PLAN;
                break;
            }
            case appoint:
            {
                FRIZY_LOG(LOG_DEBUG,"remove7");
                chassisRoad.chassisSpeed(0,0,1);
                // process = POINT;
                break;
            }
            default:
                break;
            }
            
            switch (roadState)
            {
            case roadTrue:
            {
            
                FRIZY_LOG(LOG_DEBUG,"roadbump1");
                roadDwa.init();
                roadState = roadIdle;
                return 1;
            }
            case roadFalse:    
            {
                if (IsWall() == 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"roadbump2");
                    dewallSign = 1;
                    ewall = cur,ewallTime = 0; 
                } 
                break;
            }
            default:
                break;
            }
        }

  
        return 0;
    } 

    void RoadPlanning::StartRoad(Sensor sensor,Grid cur,Trouble trouble)
    {
        _trouble = trouble;

        RobotType sss;

        if (ArriveRoad(sensor,cur))
            return;


        switch (roadState)
        {
        case roadIdle:
            //平滑曲线
            FRIZY_LOG(LOG_DEBUG,"astar step1");
            _aim.forward = 1000;
            ewallTime = 0,ewall = cur;

            if (road_aim.kind == searchBound || road_aim.kind == searchUnclean)
                astarArr = roadAstar.astarLength(cur.x,cur.y,road_aim.x,road_aim.y,sss,1,1);
            else if(road_aim.kind ==appoint || road_aim.kind ==zoning ||road_aim.kind == recharge)
                astarArr = roadAstar.astarLength(cur.x,cur.y,road_aim.x,road_aim.y,sss,1,2);
            else
                astarArr = roadAstar.astarLength(cur.x,cur.y,road_aim.x,road_aim.y,sss,1,0);                
            // astarArr.push_back({cur.x*0.15,cur.x*0.15});
            // astarArr.push_back({road_aim.x*0.15,road_aim.y*0.15});
            FRIZY_LOG(LOG_DEBUG,"astar step2");
            if (astarArr.empty())
            {
                if (astarAgain && (road_aim.kind == searchBound || road_aim.kind == searchUnclean)){
                    
                    astarArr = roadAstar.astarLength(cur.x,cur.y,road_aim.x,road_aim.y,sss,1,0);

                    if (astarArr.empty()){
                        FRIZY_LOG(LOG_DEBUG,"yiranshibai");
                        astarAgain = 0;
                        roadState = roadFalse;                        
                    }
                    else{
                        FRIZY_LOG(LOG_DEBUG,"chenggongle");
                        astarAgain = 1;
                        roadState = roadTrue;                           
                    } 
                }
                else{
                    FRIZY_LOG(LOG_DEBUG,"astar fail");
                    roadState = roadFalse;
                }
            }
            else
            {
                FRIZY_LOG(LOG_DEBUG,"astar success");
                // roadDwa._maze = _maze;
                roadState = roadTrue;
            }
            break;

        case roadTrue:

            FRIZY_LOG(LOG_DEBUG,"start run1");
            if (!roadDwa.start_path_planner(sensor,cur,astarArr))
            {
                FRIZY_LOG(LOG_DEBUG,"turanshibai");
                roadState = roadFalse;
            }
            else
            {
                FRIZY_LOG(LOG_DEBUG,"dwa ok");
                Null_aim.kind = idle;
            }
            
            FRIZY_LOG(LOG_DEBUG,"dwa over");
            
            break;

        case roadFalse:
           
            FRIZY_LOG(LOG_DEBUG,"start run2");
            NullRunning(sensor,cur);
            
            break;
        default:
            break;
        }
    }

}    