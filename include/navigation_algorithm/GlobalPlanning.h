/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-10-20 11:22:31
 * @Project      : UM_path_planning
 */

#ifndef  GLOBAL_PLANNNING_H
#define  GLOBAL_PLANNNING_H
#include <list>
#include <vector>
#include <math.h>
#include "common_function/MapFunction.h"

#include "common_function/logger.h"
#include <boost/heap/d_ary_heap.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#pragma once
using namespace std;
using namespace cv;
#define MAP_SIZE 250

extern useerobot::Maze _maze;

namespace useerobot
{
   
    template<class T>
    struct compare_state {
        bool operator()(T a1, T a2) const {
            double f1 = a1->g + a1->h;      
            double f2 = a2->g + a2->h;
            if ((f1 >= f2 - 0.000001) && (f1 <= f2 + 0.000001))
                return a1->g < a2->g;
            return f1 > f2;
        }
    };
    struct astarState; 
    using astarPtr = std::shared_ptr<astarState>;
    using astarQueue = boost::heap::d_ary_heap<astarPtr, boost::heap::mutable_<true>,
                                                    boost::heap::arity<2>, 
                                                    boost::heap::compare<compare_state<astarPtr> >>;

   // astarQueue priorityQ;

    struct astarState {

        int id{-1};

        int parentId{-1};

        short x{0}, y{0};

        astarQueue::handle_type heap_key;
 
        int g{std::numeric_limits<int>::max()};
     
        float h{};
        
        bool opened = false;

        bool closed = false;
    
        astarState(int id, short x, short y): x(x), y(y) , id(id){}

    };     





    const int kCost1 = 10; //直移一格消耗
    const int kCost2 = 14; //斜移一格消耗


//全局搜索算法基类
    class GlobalPlanning
    {
    private:
        /* data */

    public:
        GlobalPlanning();
        ~GlobalPlanning();
        virtual Point* findPath(Point &startPoint, Point &endPoint, RobotType &robotShape) = 0;
        virtual list<Point*> getPath(Point *result) = 0;
        virtual vector<Point*>getSurroundPoints(const Point *point,const RobotType &robotShape) const = 0;
        virtual bool isCanReach(const Point &start, const Point &end, const Point *target,const RobotType &robotShape) = 0;

        int step;
        int keyStep;
    };
// A*启发式搜索类，继承于全局算法类
    class aStar : public GlobalPlanning
    {
    private:
        /* data */
        list<Point*> openlist;
        list<Point*> closelist;

        //机器人型状
        int outLineIndex  = 0;
        int outLineIndexCeil = 1 ;
        int moveLength = 1;
        int failTimes = 0;
        int pathType = 0;
    public:
        aStar();
        ~aStar();


        //dijkstra
        //boundary astarBound;
        const int xrows = 250;
        const int ycols = 250;
        int connect;
        astarQueue priorityQ;
        
        vector<astarPtr> _hm;
        vector<bool> _seen;

        vector<vector<short>> _ns;

        inline int CoordTrans(short x, short y);
        inline bool IsFree(short x, short y) const;
        void Dfs(int x,int y);
        void GetSucc(astarPtr &curr, vector<int> &succ_ids,vector<int> &succ_costs);
        void Dijkstra(short xStart,short yStart,vector<vector<Point2i>>& all_path);
        void recoverPath(astarPtr node, int start_id,vector<vector<Point2i>>& all_path);


        /**
         * @description: 启发式搜索的核心代码，用于寻找路径，传入的是起点和终点的引用
         * @event: 
         * @param {Point} &startPoint
         * @param {Point} &endPoint
         * @return {*}
         */
        Point* findPath(Point &startPoint, Point &endPoint, RobotType &robotShape);

        /**
         * @description: 用于得到最佳路径
         * @event: 
         * @param {Point} *result
         * @return {*}
         */        
        list<Point*> getPath(Point *result);

        /**
         * @description: 用于得到某个点周围能够到达的所有点
         * @event: 
         * @param {const Point} *point
         * @return {*}
         */        
        vector<Point*>getSurroundPoints(const Point *point ,  const RobotType &robotShape) const ;

        /**
         * @description: 用于判断某个点是否能够到达目标点
         * @event: 
         * @param {const Point} *point
         * @param {const Point} *target
         * @return {*}
         */        
        bool isCanReach(const Point &start, const Point &end, const Point *target, const RobotType &robotShape) ;

        /**
         * @description: 用于判断某个点是否在某个列表中
         * @event: 
         * @param {list<Point*>} thisList
         * @param {const Point} *point
         * @return {*}
         */        
        
        Point* isInList(list<Point*> thisList, const Point *point) const;

        /**
         * @description: 用于得到openlist中f值最小的点
         * @event: 
         * @param {*}
         * @return {*}
         */                
        Point* getLeastFPoint();
        
        /**
         * @description: 将Astar路径转化为dwa所需路径
         * @event: 
         * @param {*}
         * @return {*}
         */       
        vector<pair<int, int>> astarLength(int x, int y, int m, int n,RobotType &robotShape, int flag, int type);

        pair<int, int> trans(int x, int y);
        pair<int, int> retrans(int x, int y);
        
        /**
         * @description: Astar初始化
         * @event: 
         * @param {*}
         * @return {*}
         */
        void initAstar();

        /**
         * @description: 判断当前点周围是否存在障碍物
         * @event: 
         * @param {*}
         * @return {*}
         */
        bool judgeBarrier(Point *curPoint);
        
        /**
         * @description: 判断两点间是否能直接到达
         * @event: 
         * @param {*}
         * @return {*}
         */
        bool judgePath(Point *starPoint,Point *endPoint);

        /**
         * @description: 计算两点间角度
         * @event: 
         * @param {*}
         * @return {*}
         */        
        double calcAngle(int sx, int sy, int ex, int ey);

        /**
         * @description: 计算Y斜率
         * @event: 
         * @param {*}
         * @return {*}
         */        
        std::pair<double, int> calcYSlop(int sx, int sy, int ex, int ey);
        
        /**
         * @description: 计算X斜率
         * @event: 
         * @param {*}
         * @return {*}
         */        
        std::pair<double, int> calcXSlop(int sx, int sy, int ex, int ey);

        /**
         * @description: 对得到的路径进行优化
         * @event: 
         * @param {*}
         * @return {*}
         */
        void optimizePath(list<Point*> &path);

        int calcG(Point *tempStart, Point *point, int moveLength);
        int calcH(Point *point, Point *endPoint);
        int calcF(Point *point);
    };  

    class idaStar : public GlobalPlanning
    {
    public:
        idaStar();
        ~idaStar();

        Point* findPath(Point &startPoint, Point &endPoint, RobotType &robotShape);
        list<Point*> getPath(Point *result);
        vector<Point*>getSurroundPoints(const Point *point) const ;
        bool isCanReach(const Point *point, const Point *target) const ;
        Point* idaSerach(Point *point, Point &endPoint, int maxF);

        int calcG(Point *point);
        int calcH(Point *point, Point *endPoint);
        int calcF(Point *point);
    };     
}

#endif
