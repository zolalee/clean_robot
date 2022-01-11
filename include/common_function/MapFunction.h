/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-08 20:39:20
 * @LastEditTime : 2022-01-08 10:00:06
 * @Project      : UM_path_planning
 */
#pragma once
#ifndef MAP_FUNCTION_H
#define MAP_FUNCTION_H

#include "lidar/lidar_shm.h"
#include <vector>
#include <list>
#include <cstdlib>
#include <stdint.h>
#include "rpc_core/core/share_mem.h"
#include <thread>
#include <algorithm>
#define MAP_WIDTH   (601)
#define MAP_HEIGHT  (601)
#define MAP_DATA_LIMIT (MAP_WIDTH*MAP_HEIGHT)
using namespace std;
using forbidenInfo = std::vector<std::pair<float, float>>;
namespace useerobot{
#define SIZE 250
#define map_clos 601
#define map_rows 601
#define MAX_BLOCK_NBR 8
const double resolution = 0.05;
// for circular robot
const float robot_radius = 0.15; // m
struct Sensor
{
    uint8_t batvolume;

    float laserL;
    float laserM;
    float laserR;

    int bump;
    int obs;
    int cliff;
    int leftw;
    int rightw;
    int cleanMode;
    int robotMode;
    int power;
    int virwall;

    int robotRadius;
    int pointSize; 
    int size;
    int leftAlongWall;
    int rightAlongWall;

    int leftCliff;
    int rightCliff;
    int midCliff;

    int leftOmnibearingSlow;
    int leftOmnibearingTurn;
    int rightOmnibearingSlow;
    int rightOmnibearingTurn;
    int midOmnibearingSlow;
    int midOmnibearingTurn;

    int leftOmnibearingOn;
    int leftOmnibearingOff;
    int midOmnibearingOn;
    int midOmnibearingOff;
    int rightOmnibearingOn;
    int rightOmnibearingOff;

    int rechargeSign;
    
    int XAngle;
    int YAngle;
    int ZAngle;

    int XAcc;
    int YAcc;
    int ZAcc;
    int addAngle;

    float leftWheelElec;
    float rightWheelElec;

    int zGyroOriginal;    

    int leftVir;
    int rightVir;
    int leftFrontVir;
    int rightFrontVir;
    int leftBehindVir;
    int rightBehindVir;
    int magnVirWall;

    int leftInfrared;
    int rightInfrared;
    int leftFrontInfrared;
    int rightFrontInfrared;
    int leftSideBrushElectricity;
    int rightSideBrushElectricity;
};
using NormalizeConer = std::pair<float, float>;
using Sine_cose_math = std::pair<float, float>;
struct Grid
{
    int x;
    int y;
    float forward;
    int correct_index;
    int addAngle;
    int dx;
    int dy;
    float realx;
    float realy;             
    bool operator == (const Grid p)
    {
        return (this->x == p.x) && (this->y == p.y);
    }
}; 
enum DataState
{
    IS_OLD = 0, //数据未更新
    IS_NEW = 1, //新一帧数据
};
struct share_pos
{
        DataState isNew; //是否为新一帧
        double x;
        double y;
        double heading;
};
typedef struct share_map
{
  int map_width;
  int map_height;
  double resolution;
  double offset_x; // map_offset_X
  double offset_y; // map_offset_Y
  uint8_t data[MAP_DATA_LIMIT];
}share_map_t;

typedef struct dynamicPoint{
  double x;
  double y;
}dynamicPoint_t;

typedef struct dynamicMapInfo{
  double buf;
  int size; 
  dynamicPoint_t dyMapPointcloud[450]; 
}dynamicMapInfo_t;

typedef struct allDirectionPoint{
  double theta;
  double x;
  double y;
}allDirectionPoint_t;

typedef struct allDirectionPointInfo{ 

  int size;
  allDirectionPoint_t point[450];
  
}allDirectionPointInfo_t;

// for rectangular robot
const float robot_width = 0.15;
const float robot_length = 0.2;
enum class RobotType {
    circle,
    rectangle
};
struct cellIndex
{
    int row;
    int col;
    double theta; //{0, 45,90,135,180,225,270,315}角度信息
}; 
struct pointCoordinate
{
    int x;
    int y;
};
struct blockCorner
{
    pointCoordinate topLeftCorner;  //区域左上角坐标
    pointCoordinate topRightCorner; //区域右上角坐标
    pointCoordinate bottomLeftCorner; //区域左下角坐标
    pointCoordinate bottomRightCorner; //区域右下角坐标
};
struct block
{
    uint8_t isNew;
    dynamicPoint topLeftCorner;  //区域左上角坐标
    dynamicPoint topRightCorner; //区域右上角坐标
    dynamicPoint bottomLeftCorner; //区域左下角坐标
    dynamicPoint bottomRightCorner; //区域右下角坐标
};

struct ProhibitedBlock
{
    int forbid_mode;   //禁区模式（0：禁扫拖 1：禁扫 2：禁拖）
    // 禁区块信息
	block Block[MAX_BLOCK_NBR];
};
// 与路径进行 桩电桩前80cm位姿\ 充电桩位姿 \ 禁区脚点 \  划区脚点 \ 定点位姿 通信接口

typedef struct share_path_planning_interface

{
 	// 充电桩位姿
    share_pos chargingPilePos;

	// 充电桩前XXcm位姿, 默认80cm
    share_pos frontOfChargingPile;

	// 指定点位姿
    share_pos selectedPointPos;	

	// 划区块位姿
    block selectedBlock[MAX_BLOCK_NBR];
    // 禁区块
    ProhibitedBlock prohibitedBlock;

}share_path_planning_t;

struct Planning_info
{
    share_pos charger_seat_position;
    share_pos charger_front_position;
    share_pos cleanPointPos;
    block cleanBlock[MAX_BLOCK_NBR];
    ProhibitedBlock forbidenArea;
    pointCoordinate blockCenter;
    
};


enum cornerIdex
{
    _topLeftCorner = 0, 
    _topRightCorner = 1,    
    _bottomLeftCorner = 2, 
    _bottomRightCorner = 3,
};
struct mapRange
{
  int xmax;
  int xmin;
  int ymax;
  int ymin;
};
struct Transformation
{
    float positive;
    float inverse;
};
enum PRO
{
    PLAN = 0,
    BOUND,
    ROAD,
    POINT,
};
enum POINTSTATE{
        NOSWEEP = 0,    //未清扫&&没障碍
        ISSWEEP = 1,    //已清扫
        OBSTACLE = 2,   //有障碍
        UNKNOW = 3,     //未知区域
};

// typedef struct _Point
// {
//     int x,y; //点坐标，这里为了方便按照 C++的数组来计算，x 代表横排，y代表竖列
//     int F,G,H; //F=G+H
//     struct _Point *parent; //parent 的坐标
//     _Point(int _x,int _y):x(_x),y(_y),F(0),G(0),H(0),parent(NULL){}  //变量初始化
// }Point;

class Point
    {
    public:
        Point();
        Point(int x, int y, int n);
        Point(const Point &p);
        bool operator== (const Point &p);
        bool isClose = 0;
        bool isOpen = 0;
        int x, y, n;//点坐标，这里为了方便按照 C++的数组来计算，x 代表横排，y代表竖列, n 代表状态
        int F, G, H;//F=G+H
        Point *parent; //parent 的坐标
    };


class Maze
    {
    public:
        Maze();
        Maze(int rows, int cols);
        Maze(const Maze& m);
        ~Maze();

        void setMaze(int rows, int cols);
        int8_t VirBound(int16_t x,int16_t y);
        void RecordMap(Sensor sensor,Grid cur);
        void InputRecord(int x,int y,int state);
        int GetMapState(int x,int y,int mapkind);
        void globalMapUpdate(); //全局地图更新
        void localMapUpdate();//局部地图更新
        void caculateMapState(int i,int j,uint8_t *map_buf);
        void mapUpdateStart();
        void mapService();
        void setMapArea();
        void setForbindenInfo(bool type);
        void setGlobalMapState(int x,int y,int state);
        NormalizeConer coordinatenormalize(block &_block,Sine_cose_math &rotate_date);
        NormalizeConer antCoordinatenormalize(block &_block,NormalizeConer &_Coner,Sine_cose_math &rotate_date);
        Sine_cose_math caculateSineCose(block &_block);
        int cols, rows;
        vector<vector<Point*> > Map;
        vector<vector<Point*> > recordMap;
        vector<Grid> filter;
        Point startPoint, endPoint;
        std::shared_ptr<std::thread> mapupdate_thread_ = nullptr;
        int times;
        int mapArea;
        Planning_info current_planning_info;
        forbidenInfo forbidenPoint;
        Transformation forbbiden_transformation;
        int remap_sum = 0;
        
    };

class MapFunction
    {
    private:
        /* data */
        GridPose current_pos;
        
    public:
        MapFunction(/* args */);
        ~MapFunction();
        bool getVirtualWall();
        void fittingMap();

        vector <cellIndex> pathArrary;

        // Maze recordMaze;
    };            
}
#endif
