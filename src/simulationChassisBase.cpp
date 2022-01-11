/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-07-26 14:44:08
 * @LastEditTime : 2021-08-14 10:15:17
 * @Project      : UM_path_planning
 */
#include "um_chassis/chassisBase.h"

#include <std_msgs/String.h>
#include <stdio.h>
#include "ros_lib/geometry_msgs/Pose2D.h"
#include "ros_lib/std_msgs/String.h"
#include "ros_lib/sensor_msgs/NavSatFix.h"
#include "ros_lib/nav_msgs/GetMap.h"
#include "ros_lib/nav_msgs/OccupancyGrid.h"
#include "ros_lib/ros.h"
#include <unistd.h>
geometry_msgs::Pose2D control_msg;
ros::Publisher speedcontrol("wheel_speed", &control_msg);

ros::NodeHandle nh;
sensor_msgs::NavSatFix gps_value;
char *rosSrvrIp = "192.168.3.54";
double GPSPosition[3];
int map_lenth,map_height;
GridPose current_robot_pose;
ros::ServiceClient<nav_msgs::GetMap::Request,nav_msgs::GetMap::Response> map_client("static_map");
// ros::ServiceClient map_client;
nav_msgs::GetMap get_map_srv;
nav_msgs::GetMap::Request map_req;
nav_msgs::GetMap::Response map_res;
useerobot::Maze map_maze;
namespace useerobot{
chassisBase *chassisBase::m_Instance = nullptr;

chassisBase *chassisBase::getInstance()
{
        if (m_Instance == nullptr){
                m_Instance = new chassisBase();
        }
        return m_Instance;
        // nh = new (ros::NodeHandle);
}
chassisBase::chassisBase()
{

};
chassisBase::~chassisBase()
{
        
};
int chassisBase::MakeChassisGoStraight(int16_t speed,GridPose targetPose)
{
        FRIZY_LOG(LOG_INFO, "start to chassis go straight ");
        //printf("Speed = %d; mode:goSTRAIGHT")
        robotPose = GetSimulationCurGridPose();
        while(fabs(sqrt((robotPose.i - targetPose.i)*(robotPose.i - targetPose.i)+(robotPose.j - targetPose.j)*(robotPose.j - targetPose.j)) )> variance)
        {
           chassisSpeed(speed,goSTRAIGHT);
           robotPose = GetSimulationCurGridPose();     
        }
}       
int chassisBase::MakeChassisTurnLeft(int16_t speed, int angle)
{
        robotPose = GetSimulationCurGridPose();
        targetAngle = robotPose.forward + angle*_Pi/180;
        //how to rotato angle??? to do
        while(fabs(robotPose.forward - targetAngle) > Diff_angle)
        {chassisSpeed(speed,turnLEFT);
        robotPose = GetSimulationCurGridPose();
        }
        FRIZY_LOG(LOG_INFO, "Turn %d left done ",angle);
}
int chassisBase::MakeChassisTurnright(int16_t speed, int angle)
{
        //how to rotato angle??? to do
        robotPose = GetSimulationCurGridPose();
        targetAngle = robotPose.forward + angle*_Pi/180;
        while(fabs(robotPose.forward - targetAngle) > Diff_angle)
        {
        chassisSpeed(speed,turnRIGHT);
        robotPose = GetSimulationCurGridPose();
        }
        FRIZY_LOG(LOG_INFO, "Turn %d right done ",angle);
}
int chassisBase::MakeChassisGoTurn(int16_t leftspeed,int16_t rightspeed,int16_t times)
{
        FRIZY_LOG(LOG_INFO, "start to excute go turn ");
        targetRobotPose = GetSimulationCurGridPose();
        for(int i =0 ;i <times;i++)
        {
              chassisSpeed(leftspeed,rightspeed,1);
              robotPose = GetSimulationCurGridPose();
              while(sqrt((robotPose.i - targetRobotPose.i)*(robotPose.i - targetRobotPose.i)+(robotPose.j - targetRobotPose.j)*(robotPose.j - targetRobotPose.j))>0.1)//判断条件太生硬
              {
                   chassisSpeed(leftspeed,rightspeed,1);   
              }
        }
}

int chassisBase::MakeBaseRecharge()
{
        FRIZY_LOG(LOG_INFO, "start to excute base Recharge ");
        chassisRecharge();
}
int chassisBase::MakeBaseEscapeJail()
{
        FRIZY_LOG(LOG_INFO, "start to excute base escapejail ");
        escapeJail();
}
int chassisBase::MakeBaseAlongWall(uint8_t mode,int direction)
{
        if(direction == right_allwalldirection){
                alongWall(mode,RightAlongwall);
        }
        if(direction == left_allwalldirection){
                alongWall(mode,LeftAlongwall);
        }
}
int chassisBase::MakeSmartRecharge()
{

}
int chassisBase::chassisSpeed(int16_t speed,int way)
{       
        
        int16_t leftWhell = 0;
        int16_t rightWhell = 0;
        memset(&controlData,0,sizeof(controlData));

        if(way == goSTRAIGHT){
                leftWhell = speed;
                rightWhell = speed;  
        }
        //how to rotato angle??? to do
        if (way == turnLEFT){
                leftWhell = -speed;
                rightWhell = speed;
        }
        //how to rotato angle??? to do
        if(way == turnRIGHT){
                leftWhell = speed;
                rightWhell = -speed;
        }

        
        controlData.cmd = CHASSIS_CMD_ROAD;
        controlData.len = 10;

        controlData.data[0] =0xFF;
        controlData.data[1] =0xFE;
        controlData.data[2] =0x42;
        controlData.data[3] =0x05;
        if(leftWhell < 0 ){
                leftWhell = leftWhell + 0xFF;
        }
        if(rightWhell < 0 ){
                rightWhell = rightWhell + 0xFF;
        }
        controlData.data[4] = (leftWhell & 0x00FF);
        controlData.data[5] = (leftWhell & 0xFF00) >> 8;
        controlData.data[6] = (rightWhell & 0x00FF);
        controlData.data[7] = (rightWhell & 0xFF00) >> 8;
        controlData.data[8] = NORMAL_MODE;
        
        //calc 校验
        for(int i = 0; i < controlData.len -1; i++){
                controlData.data[9] ^= controlData.data[i];
        }
        //共享内存写入 umweritemem ;
        UM_WriteControlMem(controlData);
        return 0;
}

int chassisBase::chassisSpeed(int16_t leftspeed,int16_t rightspeed,int mode)
{
        
        memset(&controlData,0,sizeof(controlData));
        controlData.cmd = CHASSIS_CMD_ROAD;
        controlData.len = 10;

        controlData.data[0] =0xFF;
        controlData.data[1] =0xFE;
        controlData.data[2] =0x42;
        controlData.data[3] =0x05;
        if(leftspeed < 0 ){
                leftspeed = leftspeed + 0xFFFF;
        }
        if(rightspeed < 0 ){
                rightspeed = rightspeed + 0xFFFF;
        }
        controlData.data[4] = (leftspeed & 0x00FF);
        controlData.data[5] = (leftspeed & 0xFF00) >> 8;
        controlData.data[6] = (rightspeed & 0x00FF);
        controlData.data[7] = (rightspeed & 0xFF00) >> 8;
        controlData.data[8] = mode; //1 :normal mode 2:accelerate mode 
        //calc 校验
        for(int i = 0; i < controlData.len -1; i++){
                controlData.data[9] ^= controlData.data[i];
        }
        //共享内存写入 umweritemem ;
        UM_WriteControlMem(controlData);
        return 0;        

}
void gpsCallback(const geometry_msgs::Pose2D &pose_value)
{
        FRIZY_LOG(LOG_INFO, "the pose_value  is %f , %f ,%f",pose_value.x,pose_value.y,pose_value.theta);
        current_robot_pose.i = int(pose_value.x);
        current_robot_pose.j = int(pose_value.y);
        current_robot_pose.forward = pose_value.theta;
        
}
void _getSimulationMap(const nav_msgs::OccupancyGrid &map_value)
{
        FRIZY_LOG(LOG_INFO, "start to get simulation map");

        // nh.serviceClient(map_client);
        // nh.spinOnce();
        // map_client = nh.serviceClient<nav_msgs::GetMap>("static_map");
        // map_client.call(map_req, map_res);
        // map_client.call(get_map_srv);
        
        FRIZY_LOG(LOG_INFO, "succeessful call simulation map");
        FRIZY_LOG(LOG_INFO, "map_res.map.info.height = %d",map_value.info.height);
        if(map_value.info.height!=0)
        {
                map_maze.setMaze(map_value.info.height,map_value.info.width);
                        for(int i =0; i<map_maze.cols;i++)
                {
                        for(int j =0; j<map_maze.rows;j++)
                        {
                        // maze_temp.Map[i][j]->n = tempstr[i*tempstr[0]+j+2];
                        // new Point(i,j,tempstr[i*tempstr[0]+j+2]);
                        map_maze.Map[i][j]= new Point(i,j,map_value.data[i*map_res.map.info.width+j]);
                        }
                }
                //return map_maze;
        }
        else
        {
                FRIZY_LOG(LOG_ERROR, "GET SIMULATION MAP FAILED");
                exit(0);
        }

}

ros::Subscriber<geometry_msgs::Pose2D> sub_pose("/cleanrobot/robot_pose",&gpsCallback);
ros::Subscriber<nav_msgs::OccupancyGrid> sub_map("/map",&_getSimulationMap);
void chassisBase::simulationInit()
{
        FRIZY_LOG(LOG_INFO, "init the simulation ros node ");
        nh.initNode(rosSrvrIp);
        nh.advertise(speedcontrol);
        nh.subscribe(sub_pose);
        // nh.subscribe(sub_map);
        sleep(1);
        while(!nh.connected()) 
        nh.spinOnce();

}
GridPose chassisBase::GetSimulationCurGridPose()
{
        return current_robot_pose;
}
// Maze chassisBase::getSimulationMap()
// {
//         map_maze_temp = map_maze;
//         return map_maze_temp;
// }
Maze chassisBase::getSimulationMap()
{
        FRIZY_LOG(LOG_INFO, "start to get simulation map");

        nh.serviceClient(map_client);
        // nh.spinOnce();
        // map_client = nh.serviceClient<nav_msgs::GetMap>("static_map");
        map_client.call(map_req, map_res);
        // map_client.call(get_map_srv);
        FRIZY_LOG(LOG_INFO, "succeessful call simulation map");
        FRIZY_LOG(LOG_INFO, "map_res.map.info.height = %d",map_res.map.info.height);
        if(map_res.map.info.height!=0)
        {
                map_maze_temp.setMaze(map_res.map.info.height,map_res.map.info.width);
                        for(int i =0; i<map_maze_temp.cols;i++)
                {
                        for(int j =0; j<map_maze_temp.rows;j++)
                        {
                        // maze_temp.Map[i][j]->n = tempstr[i*tempstr[0]+j+2];
                        // new Point(i,j,tempstr[i*tempstr[0]+j+2]);
                        map_maze_temp.Map[i][j]= new Point(i,j,map_res.map.data[i*map_res.map.info.width+j]);
                        }
                }
                return map_maze_temp;
        }
        else
        {
                FRIZY_LOG(LOG_ERROR, "GET SIMULATION MAP FAILED");
                exit(0);
        }
}

int chassisBase::chassisSimulationSpeed(double leftspeed,double rightspeed,int mode)
{
        FRIZY_LOG(LOG_INFO, "start to excute  chassisSimulationSpeed");
        // char *rosSrvrIp = const_cast<char*>(_rosSrvrIp.c_str());
        // nh.initNode(rosSrvrIp);
        // nh.advertise(speedcontrol);

        FRIZY_LOG(LOG_INFO, "start to assignment speed value");
        // control_msg.layout.dim = 
        // control_msg.data=(float*)malloc(sizeof(float)*2);
        control_msg.x = leftspeed;
        control_msg.y = rightspeed;
        control_msg.theta = 0.0;
        
        // FRIZY_LOG(LOG_INFO, "data_length is set");
        // FRIZY_LOG(LOG_INFO, "leftspeed is set %f",leftspeed);
        // FRIZY_LOG(LOG_INFO, "leftspeed is set %f",control_msg.x);

        // cout<<"the control_msg is "<< control_msg.data <<endl;
        FRIZY_LOG(LOG_INFO, "successful to assignment speed value");
        // nh.subscribe(sub_gps);
        // while(1)       
        // {
        usleep(500000);        
        speedcontrol.publish(&control_msg);

        nh.spinOnce();



        // }

}
int chassisBase::chassisRoadFIX()
{
        
}
int chassisBase::escapeJail()
{

        memset(&controlData,0, sizeof(controlData));
        controlData.len = 6;
        controlData.data[0] = 0xFF;
        controlData.data[1] = 0xFE;
        controlData.data[2] = 0x4C;
        controlData.data[3] = 0x01;
        controlData.data[4] = 0x00;
        for(int i = 0; i < controlData.len -1; i++){
                controlData.data[5] ^= controlData.data[i]; 
        }
        //write
        UM_WriteControlMem(controlData);
        return 0;
}
int chassisBase::chassisRecharge()
{
        //标准回冲

        memset(&controlData, 0, sizeof(controlData));
        
        controlData.cmd = CHASSIS_CMD_ROAD;
        controlData.len = 6;
        controlData.data[0] = 0xFF;
        controlData.data[1] = 0xFE;
        controlData.data[2] = 0x4E;
        controlData.data[3] = 0x01;
        controlData.data[4] = 0x01;

        for(int i = 0; i < controlData.len -1;i++){
                controlData.data[5] ^= controlData.data[i]; 
        }

        //umweritemem 
        // UM_WriteControlMem(controlData);
        return 0;
}

int chassisBase::alongWall(uint8_t mode,uint8_t direction)
{

        memset(&controlData, 0, sizeof(controlData));
        controlData.cmd = CHASSIS_CMD_ROAD;
        controlData.len = 10;
        controlData.data[0] = 0xFF;
        controlData.data[1] = 0xFE;
        controlData.data[2] = 0x73;
        controlData.data[3] = 0x05;
        controlData.data[4] = mode;
        controlData.data[5] = 00;
        controlData.data[6] = 00;
        controlData.data[7] = 150;
        controlData.data[8] = direction;

        for(int i = 0; i < controlData.len -1; i++){
                controlData.data[9] ^=controlData.data[i];
        }
        //unwerite
        // UM_WriteControlMem(controlData);
        return 0;
}
}
