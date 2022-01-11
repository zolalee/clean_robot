/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-10-25 19:56:21
 * @Project      : UM_path_planning
 */


#include "common_function/MotionControl.h"
#include "common_function/logger.h"
#include "navigation_algorithm/AlongWall.h"


//
#include <Eigen/Dense>


namespace useerobot
{
    // robotData.speed = 0.1;
    // robotData.rotateangle = 0.0;
    static Grid lastaim;
    static wheel_pid wheelPid;
    wheel_mode wheel_state;
    MotionControl::MotionControl(/* args */)
    {
        // FRIZY_LOG(LOG_INFO, "MotionControl::MotionControl interface ");
    }
    
    MotionControl::~MotionControl()
    {
    }
    void MotionControl::chassisMotionInit()
    {
        FRIZY_LOG(LOG_INFO, "start to init the motion control ");
    }
    

    void MotionControl::WheelPid(wheel_speed* Speed,Grid cur,Grid aim)
    {
        
        float_t K1 = 0.5;
        float_t K2 = 0.01;
        float_t K3 = 0.2;
        
        int aimagg = aim.forward * 10;
        int iagg = cur.forward * 10;

        if (abs(aimagg - iagg) > 1800) 
        {
            if (iagg > aimagg)
            {
                iagg = -1 * (3600 - iagg);
            }
            else
            {
                iagg = iagg + 3600;
            }			
        }

        //比例
        wheelPid.acc1 = aimagg - iagg;
        
        //积分
        wheelPid.acc2 = wheelPid.acc1 + wheelPid.acc2;
        
        //微分
        wheelPid.acc3 = wheelPid.acc1 - wheelPid.lastacc1;
        wheelPid.lastacc1 = wheelPid.acc1;

        //总误差
        wheelPid.acc = K1 * wheelPid.acc1 + K2 * wheelPid.acc2 + K3 * wheelPid.acc3;
        
        printf("accerror1.%f,accerror2.%f,accerro3.%f,accerror.%f\n",K1*wheelPid.acc1, K2 * wheelPid.acc2, K3*wheelPid.acc3, wheelPid.acc);

        Speed->left = Speed->left + wheelPid.acc;
        Speed->right = Speed->right - wheelPid.acc;
        

    }
    void MotionControl::ClearPid()
    {
        wheelPid.acc2 = 0;
        wheelPid.acc3 = 0;
        wheelPid.lastacc1 = 0;
    }

    void MotionControl::WheelBack()
    {
        robotchassiscontrol.chassisSpeed(-150,-150,1);

    }

    void MotionControl::WheelControl(Sensor sensor,Grid cur,Grid aim)
    { 
        int force_left = 0;
        if (aim.forward == 2000 || aim.forward == 2090 || aim.forward == 2180 || aim.forward == 2270){
            FRIZY_LOG(LOG_DEBUG,"-2000");
            force_left = 1;
            aim.forward -= 2000;
        }
       
        FRIZY_LOG(LOG_DEBUG,"aim.%d,%d,%f,wheel.%d,%d",aim.x,aim.y,aim.forward,sensor.leftw,sensor.rightw);
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
                
        if (lastaim.x != aim.x || lastaim.y != aim.y || lastaim.forward != aim.forward)
        {
            FRIZY_LOG(LOG_DEBUG,"update");
            wheel_state = forward;
        }
        lastaim = aim;
        
        //自选期间不足判断
        if (wheel_state != left && wheel_state != right)
        {
            //停止
            if (cur.x == aim.x && cur.y == aim.y && cur.forward == aim.forward)
            {
                FRIZY_LOG(LOG_DEBUG,"stop...");
                wheel_state = stop;
            }
            //自旋
            else if (fabs(cur.forward - aim.forward) > 5 && fabs(cur.forward - aim.forward) < 355)
            {

                if (force_left){
                    wheel_state = left;   
                }
                else if (cur.forward - aim.forward > 180 || (cur.forward - aim.forward < 0 && cur.forward - aim.forward > -180))
                {	
                    FRIZY_LOG(LOG_DEBUG,"turnright");
                    wheel_state = right;          
                }
                else{
                    FRIZY_LOG(LOG_DEBUG,"turnleft");
                    wheel_state = left;              
                }
            }
            //直行
            else
                wheel_state = forward;
        }
         
        //轮控状态机    
        if (wheel_state == stop)
        {
            ClearPid();
            StopWallFollow();
            // robotchassiscontrol.chassisSpeed(0,0,1);
        }

        if (wheel_state == right || wheel_state == left)
        {
            ClearPid();
            printf("right || left\n");
            if (fabs(cur.forward - aim.forward) < 4 || fabs(cur.forward - aim.forward) > 356)
            {

                if (sensor.leftw != 0 || sensor.rightw != 0)
                {
                    printf("brake\n");
                    robotchassiscontrol.chassisSpeed(0,0,1);
                    return;
                }
                else
                {
                    printf("OK1\n");
                    wheel_state = forward;
                    return;
                }
            }
            else
            {

                int speed = 120;

                int deta = fabs(cur.forward - aim.forward) >= 330 ? (360 - fabs(cur.forward - aim.forward)):fabs(cur.forward - aim.forward);
                
                if (deta <= 30)
                    speed = speed - 4*(30 - fabs(deta));
                
                if (wheel_state == left)
                     wheelSpeed.left = -1 * speed,wheelSpeed.right = speed;
                if (wheel_state == right)
                    wheelSpeed.left = speed,wheelSpeed.right = -1 * speed;
                FRIZY_LOG(LOG_DEBUG,"zixuan.%d,%d",wheelSpeed.left,wheelSpeed.right);
                robotchassiscontrol.chassisSpeed(wheelSpeed.left,wheelSpeed.right,1);
                return;
            } 
            
        }
        
        if (wheel_state == forward)
        {
            
            wheelSpeed.left = 250;
            wheelSpeed.right = 250;
            WheelPid(&wheelSpeed,cur,aim);
            FRIZY_LOG(LOG_DEBUG,"zhixing.%d.%d",wheelSpeed.left,wheelSpeed.right);
            robotchassiscontrol.chassisSpeed(wheelSpeed.left,wheelSpeed.right,1);

        }
    }


    bool MotionControl::wheelControl(wheel_mode &mode,move_data &data)
    {
        FRIZY_LOG(LOG_INFO, "start to execute wheel motion ");
        robotcontrolMode = mode;
        robotData = data;
        // targetpose = robotData.movepose;
        memset(&robotControldata,0,sizeof(robotControldata));
        robotControldata.cmd = CHASSIS_CMD_ROAD;
        switch (robotcontrolMode)
        {
        case 0:
        {
            FRIZY_LOG(LOG_INFO, "start to execute forwad move ");
            robotchassiscontrol.MakeChassisGoStraight(robotData.speed,robotData.movepose);
            
            break;
        }
        case 1:
        {
            FRIZY_LOG(LOG_INFO, "start to execute back move ");
            robotchassiscontrol.MakeChassisGoStraight(-robotData.speed,robotData.movepose);
            
            break;
        }
            
        case 2:
        {   
            FRIZY_LOG(LOG_INFO, "start to execute turn left move ");
            robotchassiscontrol.MakeChassisTurnLeft(robotData.speed, robotData.rotateangle);
            break;
        }
            
        case 3:
        {
            FRIZY_LOG(LOG_INFO, "start to execute turn right move ");
            robotchassiscontrol.MakeChassisTurnright(robotData.speed, robotData.rotateangle);
            break;
        }
        case 4:
        {
            FRIZY_LOG(LOG_INFO, "start to go around ");
            robotchassiscontrol.MakeChassisGoTurn(robotData.leftspeed,robotData.rightspeed,robotData.times);
            break;
        }
        
        default:
            break;
        }
        return true;
    }
    bool MotionControl::chassisRecharge()
    {
        FRIZY_LOG(LOG_INFO, "start to do chassis recharge");
        if(robotchassiscontrol.MakeBaseRecharge()==0)
        {
            FRIZY_LOG(LOG_INFO, "successful done chassis recharge");
            return true;
        }
        
        else
        {
            FRIZY_LOG(LOG_INFO, "fail done chassis recharge");
            return false;
        }
    }
    bool MotionControl::chassisWallAlong(uint8_t mode,int direction)
    {
        FRIZY_LOG(LOG_INFO, "start to do chassis wallalong");
        if(robotchassiscontrol.MakeBaseAlongWall(mode,direction)==0)
        {
            FRIZY_LOG(LOG_INFO, "successful done chassis wallalong");
            return true;
        }
        
        else
        {
            FRIZY_LOG(LOG_INFO, "fail done chassis wallalong");
            return false;
        }
    }
    bool MotionControl::pathTransformtoOrder(list<Point *> &path)
    {
        FRIZY_LOG(LOG_INFO, "start to transform path to order");
        while(path.empty())
        {
            currentpose = GetCurGridPose();
            // 取path的第一个位置进行路径转控制指令
            auto &p = path.front();
            // targetpose.i = p->x; 
            // targetpose.i = p->y;
            // targetpose.forward = 0.0;
            //要移动到的目标点
            robotData.movepose.i = p->x;
            robotData.movepose.j = p->y;
            robotData.movepose.forward = 0.0;
            // robotData.speed = 0.25;
            auto angle = currentpose.forward;// 需要确定是弧度还是度数？
            if (p->x == currentpose.i && p->y == currentpose.j)
            {
                FRIZY_LOG(LOG_INFO, "don not need to move, currentpose = path.front()");
            }

            else
            {   
                // 路径点在第一象限             
                if(p->x > currentpose.i && p->y >= currentpose.j)
                {
                    auto target_orientation = atan((p->y - currentpose.j)/(p->x - currentpose.i));
                    auto D_value = (target_orientation - angle);
                    robotData.rotateangle = fabs(D_value);
                    if (D_value > 0)
                    {
                        robotcontrolMode = left;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_one;
                    }
                    if (D_value < 0)
                    {
                        robotcontrolMode = right;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_one;
                    }
                    if(D_value == 0)
                    {
                        Goforward_quadrant_one:
                        robotcontrolMode = forward;
                        if(wheelControl(robotcontrolMode,robotData))
                            FRIZY_LOG(LOG_INFO, "Goforward_quadrant_one done well");
                        else
                            return false;
                    }

                }
                // 路径点在第四象限 
                if(p->x >= currentpose.i && p->y < currentpose.j)
                {
                    auto target_orientation = atan((p->y - currentpose.j)/(p->x - currentpose.i))+2*_Pi;
                    auto D_value = (target_orientation - angle);
                    robotData.rotateangle = fabs(D_value);
                    if (D_value > 0)
                    {
                        robotcontrolMode = left;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_four;

                    }
                    if (D_value < 0)
                    {   
                        robotcontrolMode = right;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_four;
                    }
                    if(D_value == 0)
                    {
                        Goforward_quadrant_four:
                        robotcontrolMode = forward;
                        if(wheelControl(robotcontrolMode,robotData))
                            FRIZY_LOG(LOG_INFO, "Goforward_quadrant_four done well");
                        else 
                            return false;
                    }
                }
                // 路径点在第二象限
                if(p->x <= currentpose.i && p->y > currentpose.j)
                {
                    auto target_orientation =_Pi + atan((p->y - currentpose.j)/(p->x - currentpose.i));
                    auto D_value = (target_orientation - angle);
                    robotData.rotateangle = fabs(D_value);
                    if (D_value > 0)
                    {
                        robotcontrolMode = left;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_two;
                    }
                    if (D_value < 0)
                    {
                        robotcontrolMode = right;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_two;
                    }
                    if(D_value == 0)
                    {
                        Goforward_quadrant_two:
                        robotcontrolMode = forward;
                        if(wheelControl(robotcontrolMode,robotData))
                            FRIZY_LOG(LOG_INFO, "Goforward_quadrant_two done well");
                        else
                            return false;

                    }
                }
                // 路径点在第三象限
                if(p->x < currentpose.i && p->y <= currentpose.j)
                {
                    auto target_orientation = fabs(atan((p->y - currentpose.j)/(p->x - currentpose.i)))+_Pi;
                    auto D_value = (target_orientation - angle);
                    robotData.rotateangle = fabs(D_value);
                    if (D_value > 0)
                    {
                        robotcontrolMode = left;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_tree;
                    }
                    if (D_value < 0)
                    {
                        robotcontrolMode = right;
                        wheelControl(robotcontrolMode,robotData);
                        goto Goforward_quadrant_tree;
                    }
                    if(D_value == 0)
                    {
                        Goforward_quadrant_tree:
                        robotcontrolMode = forward;
                        if(wheelControl(robotcontrolMode,robotData))
                            FRIZY_LOG(LOG_INFO, "Goforward_quadrant_tree done well");
                        else
                            return false;
                    }
                }                        
            }
            path.pop_front(); 
        }
        return true;
    }
}

