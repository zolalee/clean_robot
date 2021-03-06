/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-19 21:09:53
 * @Project      : UM_path_planning
 */

#include <sys/time.h>
#include <iostream>
#include "navigation_algorithm/GlobalPlanning.h"
#include "navigation_algorithm/BlockPlanning.h"
#include <list>
#include <iomanip>
#include <time.h>

long long GetCurrentTime()

{
        struct timeval time;

        gettimeofday(&time,NULL);

        return (long long)time.tv_sec*1000 + (long long)time.tv_usec/1000;
}


namespace useerobot
{       

        extern boundary limit; 

        GlobalPlanning::GlobalPlanning()
        {
                step = -1;
                keyStep = 11;   
        }
        
        GlobalPlanning::~GlobalPlanning()
        {
        }

        //A*类的构造函数
        aStar::aStar() : GlobalPlanning() {
            _hm.resize(4*xrows*ycols);
            _seen.resize(4*xrows*ycols,false);
        }

        aStar::~aStar() {}

        

    inline int aStar::CoordTrans(short x, short y){

        if (x >= 0 && y >= 0)
            return x + y * ycols;
        else if (x < 0 && y > 0)
            return xrows*ycols + abs(x) + y * ycols;
        else if (x < 0 && y < 0)
            return 2*xrows*ycols + abs(x) + abs(y) * ycols;
        else
            return 3*xrows*ycols + x + abs(y) * ycols;
    }


    void aStar::Dfs(int x,int y){

        if ( x >= xrows || x <= -xrows || y >= ycols || y <=-ycols
            || x >= limit.up || x <= limit.down || y >= limit.left || y <= limit.right)
            return;

        int new_id = CoordTrans(x, y);
      
        //防止重复遍历
        if (!_seen[new_id]) {
            _seen[new_id] = true;
            _hm[new_id] = std::make_shared<astarState>(new_id, x, y);
        }
        else{
            //其中的open的必须要变成清除掉,以便close
            if (_hm[new_id]->opened)
                _hm[new_id]->opened = false;
        }

        if (_maze.GetMapState(x,y,2) != 0 || connect > 500
            || _hm[new_id]->closed)
            return;
        
        ++connect;
        _hm[new_id]->closed = true;

        //CloseArr.push_back(tmp);
        Dfs(x+1,y);
        Dfs(x,y+1);
        Dfs(x-1,y);
        Dfs(x,y-1);
    }

    inline bool aStar::IsFree(short x, short y) const {

        return _maze.GetMapState(x,y,2) != 2 && x > -xrows && x < xrows && y > -ycols && y < ycols;
    }

    void aStar::GetSucc(astarPtr &curr, vector<int> &succ_ids,
                                vector<int> &succ_costs) {

        for (const auto &d : _ns) {
            short new_x{short(curr->x + d[0])};
            short new_y{short(curr->y + d[1])};


            if((d[0] == 1 && d[1] == 1 && 
                    _maze.GetMapState(new_x-1,new_y,2) == 2 && _maze.GetMapState(new_x,new_y-1,2) == 2)
                || (d[0] == 1 && d[1] == -1 && 
                    _maze.GetMapState(new_x-1,new_y,2) == 2 && _maze.GetMapState(new_x,new_y+1,2) == 2)
                || (d[0] == -1 && d[1] == 1 && 
                    _maze.GetMapState(new_x+1,new_y,2) == 2 && _maze.GetMapState(new_x,new_y-1,2) == 2)
                || (d[0] == -1 && d[1] == -1 && 
                    _maze.GetMapState(new_x+1,new_y,2) == 2 && _maze.GetMapState(new_x,new_y+1,2) == 2)) 
            {
                FRIZY_LOG(LOG_DEBUG,"brabra:%d, %d",new_x,new_y);
                continue;
            }

            if (!IsFree(new_x, new_y))
                continue;

            //printf("ccc\n");

            int new_id = CoordTrans(new_x, new_y);

            if (!_seen[new_id]) {
                _seen[new_id] = true;
                _hm[new_id] = std::make_shared<astarState>(new_id, new_x, new_y);
                
                //hm_[new_id]->h = GetHeur(new_x, new_y);
            }
            // else{
            //   if (hm_[new_id]->closed)
            //     continue;
            // }

            succ_ids.push_back(new_id);

            int tmpG = abs(d[0]) + abs(d[1]) > 1 ? 14:10;

            succ_costs.push_back(tmpG);
        }
    }

    

    void aStar::recoverPath(astarPtr node, int start_id,vector<vector<Point2i>>& all_path) {

        //VectorPoint2i path_;

        vector<Point2i> path;
        
        //std::vector <std::pair<int,int>> path;
        
        path.emplace_back(node->x, node->y);
        while (node && node->id != start_id) {

            node = _hm[node->parentId];
            path.emplace_back(node->x, node->y);
        }
        
        reverse(begin(path),end(path));
        all_path.push_back(path);

        for (auto it = path.begin();it!= path.end();++it){
            printf("path:%d,%d,size:%d all:%d",it->x,it->y,path.size(),all_path.size());
            printf("\n");
        }
        
        //path_.swap(path);
    }

    void aStar::Dijkstra(short xStart, short yStart,vector<vector<Point2i>>& all_path){

        _hm.clear();
        _seen.clear();
        priorityQ.clear();

        for (short x = -1; x <= 1; x++) {
            for (short y = -1; y <= 1; y++) {
                if (x == 0 && y == 0)
                    continue;
                _ns.emplace_back(std::vector<short>{x, y});
            }
        }

        int start_id = CoordTrans(xStart, yStart);
        //
        astarPtr currNode_ptr = std::make_shared<astarState>(astarState(start_id, xStart, yStart));
        currNode_ptr->g = 0;

        //currNode_ptr->g = cMap_[start_id];
        //currNode_ptr->h = GetHeur(xStart, yStart);

        currNode_ptr->heap_key = priorityQ.push(currNode_ptr);

        currNode_ptr->opened = true;

        _hm[currNode_ptr->id] = currNode_ptr;
        _seen[currNode_ptr->id] = true;

        int expand_iteration = 0;

        while (true) {

            expand_iteration++;
            currNode_ptr = priorityQ.top();
            priorityQ.pop();

            if (_maze.GetMapState(currNode_ptr->x,currNode_ptr->y,2) == 0
                && !currNode_ptr->closed
                && currNode_ptr->x < limit.up && currNode_ptr->x > limit.down
                && currNode_ptr->y < limit.left && currNode_ptr->y > limit.right){

                printf("arrive.%d.%d\n",currNode_ptr->x,currNode_ptr->y);

                connect = 0;

                Dfs(currNode_ptr->x,currNode_ptr->y);

                printf("connect.%d\n",connect);

                recoverPath(currNode_ptr,start_id,all_path);

            }
            
            currNode_ptr->closed = true; 
            // if (currNode_ptr->id == goal_id) {
            //   if (verbose_)
            //     printf("Goal Reached!!!!!!\n\n");
            //   break;
            // }
            vector<int> succ_ids;
            vector<int> succ_costs;

            //
            GetSucc(currNode_ptr, succ_ids, succ_costs);

            for (int i = 0; i < (int)succ_ids.size(); ++i) {

                astarPtr &child_ptr = _hm[succ_ids[i]];
                //printf("sss.%d,%d\n",currNode_ptr->x,currNode_ptr->y);

                float tentative_g_val = currNode_ptr->g + succ_costs[i];

                if (tentative_g_val < child_ptr->g) {

                    child_ptr->parentId = currNode_ptr->id; 

                    child_ptr->g = tentative_g_val;          

                    if (child_ptr->opened && !child_ptr->closed){

                        priorityQ.increase(child_ptr->heap_key); // update heap
                    }       
                    else if (child_ptr->opened && child_ptr->closed){
                        FRIZY_LOG(LOG_DEBUG,"laji!");
                        return;
                    } 
                    else if (!child_ptr->closed){

                        child_ptr->heap_key = priorityQ.push(child_ptr);
                        child_ptr->opened = true;
                    }
                } 
            } 

            if (priorityQ.empty()) {
                return;
            }
        }
    }  



        int aStar::calcG(Point *tempStart,Point *point, int moveLength)
        {
                int thisG = (abs(point->x - tempStart->x) + abs(point->y - tempStart->y)) == moveLength ? (kCost1 * moveLength) : (kCost2 * moveLength);
                int parentG = point->parent == NULL ? 0 : point->parent->G;
                return thisG + parentG;
        }

        int aStar::calcH(Point *point, Point *endPoint)
        {	
                int x = point->x;
                int y = point->y;
                int tmp = 0;
                if (_maze.recordMap[x+1][y]->n == 2 || _maze.recordMap[x-1][y]->n == 2 || _maze.recordMap[x][y+1]->n == 2 || _maze.recordMap[x][y-1]->n == 2 
                 || _maze.recordMap[x+1][y+1]->n == 2 || _maze.recordMap[x-1][y-1]->n == 2 || _maze.recordMap[x+1][y-1]->n == 2 || _maze.recordMap[x-1][y+1]->n == 2)
                {
                        tmp = 300;
                }
                if ((_maze.recordMap[x+2][y]->n == 2 && _maze.recordMap[x+2][y+1]->n == 2 && _maze.recordMap[x+2][y-1]->n == 2)
                    || (_maze.recordMap[x-2][y]->n == 2 && _maze.recordMap[x-2][y+1]->n == 2 && _maze.recordMap[x-2][y-1]->n == 2)
                    || (_maze.recordMap[x][y+2]->n == 2 && _maze.recordMap[x+1][y+2]->n== 2 && _maze.recordMap[x-1][y+2]->n == 2)
                    || (_maze.recordMap[x][y-2]->n == 2 && _maze.recordMap[x+1][y-2]->n == 2 && _maze.recordMap[x-1][y-2]->n == 2))
                {
                        tmp = 100;
                }
                
                if (_maze.Map[x+1][y]->n == 4 || _maze.Map[x-1][y]->n == 4 || _maze.Map[x][y+1]->n == 4 || _maze.Map[x][y-1]->n == 4 
                 || _maze.Map[x+1][y+1]->n == 4 || _maze.Map[x-1][y-1]->n == 4 || _maze.Map[x+1][y-1]->n == 4 || _maze.Map[x-1][y+1]->n == 4)
                {
                        tmp = 300;
                }
                if ((_maze.Map[x+2][y]->n == 4 && _maze.Map[x+2][y+1]->n == 4 && _maze.Map[x+2][y-1]->n == 4)
                    || (_maze.Map[x-2][y]->n == 4 && _maze.Map[x-2][y+1]->n == 4 && _maze.Map[x-2][y-1]->n == 4)
                    || (_maze.Map[x][y+2]->n == 4 && _maze.Map[x+1][y+2]->n== 4 && _maze.Map[x-1][y+2]->n == 4)
                    || (_maze.Map[x][y-2]->n == 4 && _maze.Map[x+1][y-2]->n == 4 && _maze.Map[x-1][y-2]->n == 4))
                {
                        tmp = 100;
                }

                //曼哈顿距离
                // if(_maze.recordMap[point->x][point->y]->n == 1)
                    return ((abs(point->x - endPoint->x)+abs(point->y - endPoint->y)) * 10 + tmp);
                // else 
                    // return ((abs(point->x - endPoint->x)+abs(point->y - endPoint->y)) * 20 + tmp);
                //欧几里得距离 任意方向可运动
                //return sqrt((point->x - endPoint->x)*(point->x - endPoint->x) + (point->y - endPoint->y)*(point->y - endPoint->y))*kCost1;
        }
        int aStar::calcF(Point *point)
        {
                return point->G + point->H;
        }

        Point* aStar::getLeastFPoint()
        {
                if (!openlist.empty())
                {
                        auto resPoint = openlist.front();
                        for (auto &point : openlist)
                        {
                                if (point->F <= resPoint->F)
                                        resPoint = point;
                        }
                        return resPoint;
                }
                return nullptr;

        }

        Point *aStar::findPath(Point &startPoint, Point &endPoint, RobotType &robotShape)
        {
                int opentimes = 0;
                int closetimes = 0;
                if(robotShape == RobotType::circle )
                {
                        outLineIndex = 0;// 计算机器人本体拟合的轮廓占据栅格半径；
                        // outLineIndex += 1;
                        moveLength = 1;
                        outLineIndexCeil = ceil(outLineIndex);
                }
                moveLength = 1;
                if(startPoint.x + moveLength > _maze.rows || startPoint.x - moveLength < 0 ||
                   endPoint.y + moveLength > _maze.cols || endPoint.y - moveLength < 0)
                {
                        FRIZY_LOG(LOG_INFO,"astar error, the point it's not allowed");
                        return nullptr;
                }
                if(robotShape == RobotType::rectangle ) // 此处需要重新设计
                { /*need to do */}
                openlist.push_back(&startPoint);
                if(failTimes > 3)
                    FRIZY_LOG(LOG_DEBUG, "astar fail 3 times");
                while (!openlist.empty())
                {
                        Point* curPoint = getLeastFPoint();
                        openlist.remove(curPoint);  //从open内删除
                        curPoint->isOpen = 0;
                        curPoint->isClose = 1;  //进入关闭列表标志位c
                        closetimes++;
                        // 对当前节点周围相邻步长的8个点进行处理
                        for(int x = curPoint->x-moveLength; x <= curPoint->x+moveLength; x+=moveLength)
                        {
                                for(int y = curPoint->y-moveLength; y <= curPoint->y+moveLength; y+=moveLength)
                                {
                                        if(x == curPoint->x && y == curPoint->y)
                                                continue;
                                        if(pathType != 2)
                                        {
                                            if(!isCanReach(startPoint, endPoint, _maze.recordMap[x][y], robotShape))
                                                    continue;
                                            auto target = _maze.recordMap[x][y];
                                            //不允许穿过两个障碍点之间的点
                                            if(((target->x - curPoint->x == 1 && target->y - curPoint->y == 1) && 
                                            (_maze.recordMap[target->x-1][target->y]->n == OBSTACLE && _maze.recordMap[target->x][target->y-1]->n == OBSTACLE))

                                            || ((target->x - curPoint->x == 1 && target->y - curPoint->y == -1) &&
                                            (_maze.recordMap[target->x-1][target->y]->n == OBSTACLE && _maze.recordMap[target->x][target->y+1]->n == OBSTACLE))

                                            || ((target->x - curPoint->x == -1 && target->y - curPoint->y == 1) &&
                                            (_maze.recordMap[target->x+1][target->y]->n == OBSTACLE && _maze.recordMap[target->x][target->y-1]->n == OBSTACLE))

                                            || ((target->x - curPoint->x == -1 && target->y - curPoint->y == -1) &&
                                            (_maze.recordMap[target->x+1][target->y]->n == OBSTACLE && _maze.recordMap[target->x][target->y+1]->n == OBSTACLE))
                                            ) 
                                            {
                                                FRIZY_LOG(LOG_DEBUG, "in 2 bra center:%d, %d", target->x, target->y);
                                                continue;
                                            }
                                            // if(!isInList(openlist, target))
                                            if(!target->isOpen)
                                            {
        
                                                if(curPoint->x == target->x && curPoint->y == target->y)
                                                    FRIZY_LOG(LOG_DEBUG, "tar:%d,%d", target->x, target->y);
                                                    
                                                target->parent = curPoint;
                                                target->G = calcG(curPoint, target, moveLength);
                                                target->H = calcH(target, &endPoint);
                                                target->F = calcF(target);
                                                openlist.push_back(target);
                                                opentimes+=1;
                                                target->isOpen = 1;
                                            }
                                            else
                                            {
                                                    
                                                int tempG = calcG(curPoint, target, moveLength);
                                                if(tempG < curPoint->G)
                                                {
                                                target->parent = curPoint;
                                                target->G = tempG;
                                                target->F = calcF(target);
                                                }
                                            }
                                        }
                                        else
                                        {
                                            if(!isCanReach(startPoint, endPoint, _maze.Map[x][y], robotShape))
                                                    continue;
                                            auto target = _maze.Map[x][y];
                                            //不允许穿过两个障碍点之间的点
                                            if(((target->x - curPoint->x == 1 && target->y - curPoint->y == 1) && 
                                            (_maze.Map[target->x-1][target->y]->n == OBSTACLE && _maze.Map[target->x][target->y-1]->n == OBSTACLE))

                                            || ((target->x - curPoint->x == 1 && target->y - curPoint->y == -1) &&
                                            (_maze.Map[target->x-1][target->y]->n == OBSTACLE && _maze.Map[target->x][target->y+1]->n == OBSTACLE))

                                            || ((target->x - curPoint->x == -1 && target->y - curPoint->y == 1) &&
                                            (_maze.Map[target->x+1][target->y]->n == OBSTACLE && _maze.Map[target->x][target->y-1]->n == OBSTACLE))

                                            || ((target->x - curPoint->x == -1 && target->y - curPoint->y == -1) &&
                                            (_maze.Map[target->x+1][target->y]->n == OBSTACLE && _maze.Map[target->x][target->y+1]->n == OBSTACLE))
                                            ) 
                                            {
                                                continue;
                                            }
                                            // if(!isInList(openlist, target))
                                            if(!target->isOpen)
                                            {
        
                                                    if(curPoint->x == target->x && curPoint->y == target->y)
                                                        FRIZY_LOG(LOG_DEBUG, "tar:%d,%d", target->x, target->y);
                                                    target->parent = curPoint;
                                                    target->G = calcG(curPoint, target, moveLength);
                                                    target->H = calcH(target, &endPoint);
                                                    target->F = calcF(target);
                                                    openlist.push_back(target);
                                                    opentimes+=1;
                                                    target->isOpen = 1;
                                            }
                                            else
                                            {
                                                    
                                                    int tempG = calcG(curPoint, target, moveLength);
                                                    if(tempG < curPoint->G)
                                                    {
                                                            target->parent = curPoint;
                                                            target->G = tempG;
                                                            target->F = calcF(target);
                                                    }
                                            }
                                        }
                                        Point *resPoint = isInList(openlist,&endPoint);
                                        if (resPoint)
                                        {
                                                failTimes = 0;
                                                FRIZY_LOG(LOG_DEBUG , "closetimes:%d", closetimes);
                                                FRIZY_LOG(LOG_INFO, "aStar::findPath successful");
                                                return resPoint;
                                        }
                                        
                                }
                        }
                        //计算次数限制
                        if(closetimes > 2000)
                        {
                                FRIZY_LOG(LOG_DEBUG, "closetimes > 2000, break");
                                break;
                        }

                }
                failTimes ++;
                FRIZY_LOG(LOG_ERROR, "aStar::findPath failed");
                return nullptr;
        }

        list<Point*> aStar::getPath(Point *result)
        {
                // FRIZY_LOG(LOG_INFO, "start to get path in astar model");
                list<Point*> path;
                while (result)
                {	
                        path.push_front(result);
                        result = result->parent;
                }
                openlist.clear();
                FRIZY_LOG(LOG_INFO, " Get path in astar model successful");
                return path;
        }

        Point* aStar::isInList(list<Point*> thisList, const Point *point) const
        {
                for (auto p : thisList)
                        if (p->x == point->x && p->y == point->y)
                                return p;
                return NULL;
        }

        //对周围8个点判断能否到达
        bool aStar::isCanReach(const Point &start, const Point &end, const Point *target, const RobotType &robotShape) 
        {       
                if(target->x == end.x && target->y == end.y)
                    return true;
                if(target->x-outLineIndex < 0 || target->x+outLineIndex > _maze.rows || target->y-outLineIndex < 0 || 
                   target->y+outLineIndex > _maze.cols || target->isClose == 1 ||
                   _maze.recordMap[target->x][target->y]->n == OBSTACLE || _maze.Map[target->x][target->y]->n == 4)
                {
                        // FRIZY_LOG(LOG_DEBUG, "recordMap:(%d,%d):%d", target->x, target->y, _maze.recordMap[target->x][target->y]->n);
                        // FRIZY_LOG(LOG_DEBUG, "Map:(%d,%d):%d", target->x, target->y, _maze.Map[target->x][target->y]->n);
                        return false;
                }
                //寻路失败次数超过３次，降低路径要求
                // if(failTimes <= 3 && !pathType)
                // {
                //     if(abs(target->x - start.x) >= 2 || abs(target->y - start.y) >= 2)
                //     {
                //         for(int x = target->x - 1; x <= target->x + 1; x++)
                //         {
                //             for(int y = target->y - 1; y <= target->y + 1; y++)
                //             {
                //                 if(_maze.recordMap[x][y]->n == OBSTACLE)
                //                 {
                //                     return false;
                //                 }
                //             }
                //         }
                //     }
                // }
                if(pathType == 1)
                {
                    if(_maze.recordMap[target->x][target->y]->n == 1)
                        return true;
                    else 
                        return false;
                }
                else if(!pathType)
                {
                    if(_maze.recordMap[target->x][target->y]->n == 1 || _maze.recordMap[target->x][target->y]->n == 0)
                        return true;
                    else 
                        return false;
                }
                else if(pathType == 2)
                {
                    if(_maze.Map[target->x][target->y]->n == 2 || _maze.recordMap[target->x][target->y]->n == 2)
                        return false;
                    else 
                        return true;
                } 
        }                        

        vector<Point*> aStar::getSurroundPoints(const Point *point, const RobotType &robotShape) const
        {
                vector<Point*> surroundPoints;
                // for(int x = point->x-2; x <= point->x+2; x+=2)
                // {
                //         for(int y = point->y-2; y <= point->y+2; y+=2)
                //         {
                //                 if(isCanReach(point, maze.Map[x][y], robotShape))
                //                         {
                //                                 surroundPoints.push_back(maze.Map[x][y]);
                //                                  FRIZY_LOG(LOG_INFO, "surroundPoints size:%d",surroundPoints.size());
                //                         }
                //         }
                // }
                return surroundPoints;
        }


        


        pair<int, int> aStar::trans(int x, int y)
        {
                int temp;
                pair<int, int> ret;
                if(x < 0)
                        x = abs(x) + MAP_SIZE;
                else 
                        x = MAP_SIZE - x;
                if(y < 0)
                        y = abs(y) + MAP_SIZE;
                else
                        y = MAP_SIZE - y;
                ret = {x, y};
                return ret;
        }
        pair<int, int> aStar::retrans(int x, int y)
        {
                int temp;
                pair<int, int> ret;
                if(x > MAP_SIZE || x == MAP_SIZE)
                        x = MAP_SIZE - x;
                else 
                        x = abs(x - MAP_SIZE);
                if(y > MAP_SIZE || y == MAP_SIZE)
                        y = MAP_SIZE - y;
                else 
                        y = abs(y - MAP_SIZE);
                ret = {x, y};
                return ret;
        }
        vector<pair<int, int>>aStar::astarLength(int x, int y, int m, int n,RobotType &robotShape, int flag, int type)
        {

                list<Point*> path;
                pathType = type;
                initAstar();
                FRIZY_LOG(LOG_DEBUG, "flag:%d  pathType:%d", flag, pathType);
                // FRIZY_LOG(LOG_DEBUG,"start calculate astarLength");
                vector<pair<int, int>> dwapath;
                vector<pair<int, int>> retpath;
                if(x == m && y == n)
                {
                        FRIZY_LOG(LOG_ERROR,"the endPoint is same as the starPoint");
                        return dwapath;
                }
                pair<int, int> start = trans(x, y);
                pair<int, int> des = trans(m, n);
                FRIZY_LOG(LOG_DEBUG,"startX,startY:%d,%d", x, y);
                FRIZY_LOG(LOG_DEBUG,"destX,destY:%d,%d", m, n);
                FRIZY_LOG(LOG_DEBUG,"after transform startX, startY: %d, %d", start.first, start.second);
                FRIZY_LOG(LOG_DEBUG,"after transform destX, destY: %d, %d", des.first, des.second);
                pair<int, int> tmp;
                Point startPoint(start.first, start.second, 0);
                Point endPoint(des.first, des.second, 0);
                long long start_time = GetCurrentTime();
                Point *destination = findPath(startPoint, endPoint, robotShape);
                if(destination == nullptr)
                {
                        FRIZY_LOG(LOG_ERROR,"can't find the path");
                        return dwapath;
                }
                path = getPath(destination);
                long long cost_time = GetCurrentTime() - start_time;
                FRIZY_LOG(LOG_INFO, "find path cost %lld ms", cost_time);
                if(pathType != 2)
                {
                    for(auto p : path)
                    {
                        FRIZY_LOG(LOG_INFO,"astar point:%d %d, n:%d",p->x, p->y, _maze.recordMap[p->x][p->y]->n);
                    }
                }
                if (!flag)
                {
                        for(auto p : path)
                        {
                                tmp = retrans(p->x, p->y);
                                FRIZY_LOG(LOG_DEBUG,"aStar path point retransform: %d, %d", tmp.first, tmp.second);
                                retpath.push_back(tmp);
                        }
                        for(auto q : retpath)
                        {
                                dwapath.push_back({q.first, q.second});
                        }
                        
                }
                else
                {
                        for(auto i : path)
                        {
                                tmp = retrans(i->x, i->y);
                                FRIZY_LOG(LOG_DEBUG,"aStar path point retransform: %d, %d", tmp.first, tmp.second);
                                // retpath.push_back(tmp);
                        }
                        optimizePath(path);
                        for(auto p : path)
                        {
                            FRIZY_LOG(LOG_INFO,"optimize astar point:%d %d, n:%d",p->x, p->y, _maze.recordMap[p->x][p->y]->n);
                            for(int x=p->x-1; x <= p->x+1; x++)
                            {
                                for(int y=p->y-1; y <= p->y+1; y++)
                                { 
                                    if(_maze.recordMap[x][y]->n == OBSTACLE)
                                        FRIZY_LOG(LOG_DEBUG,"near point:%d %d obstacle",x,y);
                                }
                            }
                        }
                        for(auto p : path)
                        {
                                tmp = retrans(p->x, p->y);
                                FRIZY_LOG(LOG_DEBUG,"aStar optimize path point  retransform: %d, %d", tmp.first, tmp.second);
                                retpath.push_back(tmp);
                        }
                        for(auto q : retpath)
                        {
                                dwapath.push_back({q.first, q.second});
                        }
                }              
                pathType = 0;
                return dwapath;
                
        }

        void aStar::initAstar()
        {
                int i, j;
                for(i=0; i < _maze.rows; i++)
                {
                        for(j=0; j < _maze.cols; j++)
                        {
                                _maze.recordMap[i][j]->isClose = 0;
                                _maze.recordMap[i][j]->isOpen = 0;
                                _maze.recordMap[i][j]->parent = nullptr;
                                if(pathType == 2)
                                {
                                    _maze.Map[i][j]->isClose = 0;
                                    _maze.Map[i][j]->isOpen = 0;
                                    _maze.Map[i][j]->parent = nullptr;
                                }
                        }
                }
                if(!openlist.empty())
                {
                        openlist.clear();
                }
                FRIZY_LOG(LOG_INFO,"aStar init successful");
        }       

        bool aStar::judgeBarrier(Point *curPoint)
        {
                int num = 0;
                // FRIZY_LOG(LOG_INFO,"judgeBarrier curpoint:%d,%d",curPoint->x,curPoint->y);
                if(!pathType)
                {
                    // for(int x=curPoint->x - 1; x<=curPoint->x + 1; x++)
                    // {
                    //         for(int y=curPoint->y - 1; y<=curPoint->y + 1; y++)
                    //         {
                                    // FRIZY_LOG(LOG_INFO,"x:%d.y:%d,n:%d",x,y,_maze.recordMap[x][y]->n);
                                    // if(_maze.recordMap[x][y]->n == OBSTACLE || _maze.Map[x][y]->n == 4)
                                    if(_maze.recordMap[curPoint->x][curPoint->y]->n == OBSTACLE || _maze.Map[curPoint->x][curPoint->y]->n == 4)
                                    {
                                            // FRIZY_LOG(LOG_DEBUG,"OBSTACLE IS : %d, %d", curPoint->x, curPoint->y);
                                            return false;
                                    }
                    //         }
                    // }
                }
                else if(pathType == 1)
                {
                    // for(int x=curPoint->x - 1; x<=curPoint->x + 1; x++)
                    // {
                    //         for(int y=curPoint->y - 1; y<=curPoint->y + 1; y++)
                    //         {
                    //                 if(_maze.recordMap[x][y]->n != 1)
                                    if(_maze.recordMap[curPoint->x][curPoint->y]->n != 1 || _maze.Map[curPoint->x][curPoint->y]->n == 4)
                                    {
                                            // FRIZY_LOG(LOG_DEBUG,"OBSTACLE IS : %d, %d", curPoint->x, curPoint->y);
                                            return false;
                                    }
                    //         }
                    // }
                }
                else if(pathType == 2)
                {
                    // for(int x=curPoint->x - 1; x<=curPoint->x + 1; x++)
                    // {
                    //         for(int y=curPoint->y - 1; y<=curPoint->y + 1; y++)
                    //         {
                                    // if(_maze.Map[x][y]->n == 2 || _maze.Map[x][y]->n == 4 || _maze.recordMap[x][y]->n == OBSTACLE)
                                    if(_maze.Map[curPoint->x][curPoint->y]->n == 2 ||
                                     _maze.Map[curPoint->x][curPoint->y]->n == 4 ||
                                      _maze.recordMap[curPoint->x][curPoint->y]->n == OBSTACLE)
                                    {
                                            return false;
                                    }
                    //         }
                    // }
                }
                return true;
        }

        //y = k * x + b;
        std::pair<double, int> aStar::calcYSlop(int sx, int sy, int ex, int ey)
        {
                cout<<setiosflags(ios::fixed)<<setprecision(6);
                std::pair<float,float> slop;
                float k;
                int b;
                int tmp;
                float tmpx;
                float y = ey - sy;
                float x = ex - sx;
                k = y / x;
                tmpx = k * ex;
                tmp = round(tmpx);
                b = ey - tmp;     
                slop.first = k;
                slop.second = b;
                return slop;

        }



        //x = k * y + b;
        std::pair<double, int> aStar::calcXSlop(int sx, int sy, int ex, int ey)
        {
                cout<<setiosflags(ios::fixed)<<setprecision(6);
                std::pair<float,float> slop;
                float k;
                int b;
                int tmp;
                float tmpy;
                float y = ey - sy;
                float x = ex - sx;
                k = x / y;
                tmpy = k * ey;
                tmp = round(tmpy);
                b = ex - tmp;
                slop.first = k;
                slop.second = b;
                return slop;

        }
        
        double aStar::calcAngle(int sx, int sy, int ex, int ey)
        {

                float y = abs(ey - sy);
                float x = abs(ex - sx);
                float temp = y / x;
                cout<<setiosflags(ios::fixed)<<setprecision(2);
                double angle =  atan(temp)*180 / 3.14;
                return angle;
        }
        
        bool aStar::judgePath(Point *starPoint,Point *endPoint)
        {       
                if(starPoint->x == endPoint->x && starPoint->y == endPoint->y)
                    return false;
                if((abs(endPoint->x - starPoint->x) == 0 && abs(endPoint->y - starPoint->y) % moveLength == 0) ||
                  (abs(endPoint->x - starPoint->x) % moveLength == 0 && abs(endPoint->y - starPoint->y) == 0))
                {
                        // FRIZY_LOG(LOG_INFO, "angle:0");
                        if(endPoint->x > starPoint->x && endPoint->y == starPoint->y)
                        {       
                                for(int x=starPoint->x; x<=endPoint->x; x++)
                                {
                                        // if(x == starPoint->x)
                                        //         continue;
                                        if(!judgeBarrier(_maze.recordMap[x][starPoint->y]))       
                                                return false;   
                                        if(!judgeBarrier(_maze.recordMap[x][starPoint->y+1]))       
                                                return false;   
                                        if(!judgeBarrier(_maze.recordMap[x][starPoint->y-1]))       
                                                return false;
                                }
                                return true;
                        }
                        else if(endPoint->x < starPoint->x && endPoint->y == starPoint->y)
                        {       
                                for(int x=starPoint->x; x>=endPoint->x; x--)
                                {
                                        // if(x == starPoint->x)
                                        //         continue;
                                        if(!judgeBarrier(_maze.recordMap[x][starPoint->y]))       
                                                return false;   
                                        if(!judgeBarrier(_maze.recordMap[x][starPoint->y+1]))       
                                                return false;   
                                        if(!judgeBarrier(_maze.recordMap[x][starPoint->y-1]))       
                                                return false;   
                                }
                                return true;
                        }
                        else if(endPoint->x == starPoint->x && endPoint->y > starPoint->y)
                        {       
                                for(int y=starPoint->y; y<=endPoint->y; y++)
                                {
                                        // if(y == starPoint->y)
                                        //         continue;
                                        if(!judgeBarrier(_maze.recordMap[starPoint->x][y]))       
                                                return false;
                                        if(!judgeBarrier(_maze.recordMap[starPoint->x+1][y]))       
                                                return false;
                                        if(!judgeBarrier(_maze.recordMap[starPoint->x-1][y]))       
                                                return false;
                                }
                                return true;
                        }
                        else if(starPoint->y == endPoint->y && endPoint->y < starPoint->y)
                        {       
                                for(int y=starPoint->y; y>=endPoint->y; y--)
                                {
                                        // if(y == starPoint->y)
                                        //         continue;
                                        if(!judgeBarrier(_maze.recordMap[starPoint->x][y]))
                                                return false;
                                        if(!judgeBarrier(_maze.recordMap[starPoint->x+1][y]))       
                                                return false;
                                        if(!judgeBarrier(_maze.recordMap[starPoint->x-1][y]))       
                                                return false;
                                }
                                return true;
                        }
                }
                //对角线上的点，终点起点xy坐标取余步长为0，除以步长结果相等
                // else if((abs(endPoint->x - starPoint->x)%moveLength == 0 && abs(endPoint->y - starPoint->y)%moveLength == 0) && 
                //          (abs(endPoint->x - starPoint->x)/moveLength == abs(endPoint->y - starPoint->y)/moveLength))
                else if(abs(endPoint->x - starPoint->x) == abs(endPoint->y - starPoint->y))
                {
                        // FRIZY_LOG(LOG_INFO,"angle:45");
                        if(endPoint->x > starPoint->x && endPoint->y > starPoint->y)
                        {       
                                int x = starPoint->x;
                                int y = starPoint->y;
                                // x++;
                                // y++;
                                for(x; x<=endPoint->x; x++)
                                {
                                        
                                        if(!judgeBarrier(_maze.recordMap[x][y]))
                                        {
                                                return false;
                                        }
                                        y++;
                                }
                                return true;
                        }
                        else if(endPoint->x > starPoint->x && endPoint->y < starPoint->y)
                        {       
                                int x = starPoint->x;
                                int y = starPoint->y;
                                // x++;
                                // y--;
                                for(x; x<=endPoint->x; x++)
                                {
                                        if(!judgeBarrier(_maze.recordMap[x][y]))
                                        {
                                                return false;
                                        }
                                        y--;
                                }
                                return true;
                        }
                        else if(endPoint->x < starPoint->x && endPoint->y > starPoint->y)
                        {       
                                int x = starPoint->x;
                                int y = starPoint->y;
                                // x--;
                                // y++;
                                for(x; x>=endPoint->x; x--)
                                {
                                        if(!judgeBarrier(_maze.recordMap[x][y]))
                                        {
                                                return false;
                                        }
                                        y++;
                                }
                                return true;
                        }
                        else if(endPoint->x < starPoint->x && starPoint->y < endPoint->y)
                        {       
                                int x = starPoint->x;
                                int y = starPoint->y;
                                // x--;
                                // y--;
                                for(x; x>=endPoint->x; x--)
                                {
                                        if(!judgeBarrier(_maze.recordMap[x][y]))
                                        {
                                                return false;
                                        }
                                        y--;
                                }
                                return true;
                        }
                }
                //其他角度
                else
                {
                        // FRIZY_LOG(LOG_INFO, "angle:other");
                        //斜率k + 截距b
                        std::pair<float, int> temp;
                        double k, angle;
                        int b;       
                        angle = ((double)abs(endPoint->y - starPoint->y) / (double)abs(endPoint->x - starPoint->x));
                        /* 处理时，倾斜角大于45度角的情况（此时起点与终点间纵向距离大于横向距离）使用纵向遍历，
                        而对倾斜角小于45度 的情况（此时起点与终点间横向距离大于纵向距离）使用横向遍历，这样就不会漏掉任何一个点了*/
                        if(endPoint->x > starPoint->x && endPoint->y > starPoint->y)
                        {       
                                //横向遍历
                                if(angle < 1)
                                {
                                        temp = calcYSlop(starPoint->x, starPoint->y, endPoint->x, endPoint->y);
                                        k = temp.first;
                                        b = temp.second;
                                        float y;
                                        for(int x=starPoint->x; x<=endPoint->x; x++)
                                        {
                                                // if(x == starPoint->x)
                                                //         continue;
                                                y = k * x + b;
                                                y = round(y);
                                                if(!judgeBarrier(_maze.recordMap[x][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x][y+1]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x][y-1]))
                                                {
                                                        return false;
                                                }
                                        }
                                        return true;
                                }
                                //纵向遍历
                                if(angle > 1)
                                {
                                        temp = calcXSlop(starPoint->x, starPoint->y, endPoint->x, endPoint->y);
                                        k = temp.first;
                                        b = temp.second;
                                        float x;
                                        for(int y=starPoint->y; y<=endPoint->y; y++)
                                        {
                                                // if(y == starPoint->y)
                                                //         continue;
                                                x = k * y + b;
                                                x = round(x);
                                                if(!judgeBarrier(_maze.recordMap[x][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x+1][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x-1][y]))
                                                {
                                                        return false;
                                                }
                                        }
                                        return true;
                                }
                        }
                        else if(endPoint->x > starPoint->x && endPoint->y < starPoint->y)
                        {
                                if(angle < 1)
                                {
                                        temp = calcYSlop(starPoint->x, starPoint->y, endPoint->x, endPoint->y);
                                        k = temp.first;
                                        b = temp.second;
                                        float y;
                                        for(int x=starPoint->x; x<=endPoint->x ;x++)
                                        {
                                                // if(x == starPoint->x)
                                                //         continue;
                                                y = k * x + b;
                                                y = round(y);
                                                if(!judgeBarrier(_maze.recordMap[x][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x][y+1]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x][y-1]))
                                                {
                                                        return false;
                                                }
                                        }
                                        return true;
                                }
                                if(angle > 1)
                                {
                                        temp = calcXSlop(starPoint->x, starPoint->y, endPoint->x, endPoint->y);
                                        k = temp.first;
                                        b = temp.second;
                                        float x;
                                        for(int y=starPoint->y; y>=endPoint->y; y--)
                                        {
                                                // if(y == starPoint->y)
                                                //         continue;
                                                x = k * y + b;
                                                x = round(x);
                                                if(!judgeBarrier(_maze.recordMap[x][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x+1][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x-1][y]))
                                                {
                                                        return false;
                                                }
                                        }
                                        return true;
                                }
                        }
                        else if(endPoint->x < starPoint->x && endPoint->y > starPoint->y)
                        {
                                if(angle < 1)
                                {
                                        temp = calcYSlop(starPoint->x, starPoint->y, endPoint->x, endPoint->y);
                                        
                                        k = temp.first;
                                        b = temp.second;
                                        float y;
                                        for(int x=starPoint->x; x>=endPoint->x; x--)
                                        {
                                                // if(x == starPoint->x)
                                                //         continue;
                                                y = k * x + b;
                                                y = round(y);
                                                if(!judgeBarrier(_maze.recordMap[x][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x][y+1]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x][y-1]))
                                                {
                                                        return false;
                                                }
                                        }
                                        return true;
                                }
                                if(angle > 1)
                                {
                                        temp = calcXSlop(starPoint->x, starPoint->y, endPoint->x, endPoint->y);
                                        k = temp.first;
                                        b = temp.second;
                                        float x;
                                        for(int y=starPoint->y; y<=endPoint->y; y++)
                                        {
                                                // if(y == starPoint->y)
                                                //         continue;
                                                x = k * y + b;
                                                x = round(x);
                                                if(!judgeBarrier(_maze.recordMap[x][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x+1][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x-1][y]))
                                                {
                                                        return false;
                                                }
                                        }
                                        return true;
                                }
                        }
                        else if(endPoint->x < starPoint->x && endPoint->y < starPoint->y)
                        {
                                if(angle < 1)
                                {
                                        temp = calcYSlop(starPoint->x, starPoint->y, endPoint->x, endPoint->y);
                                        k = temp.first;
                                        b = temp.second;
                                        float y; 
                                        for(int x=starPoint->x; x>=endPoint->x; x--)
                                        {
                                                // if(x == starPoint->x)
                                                //         continue;
                                                y = k * x + b;
                                                y = round(y);
                                                if(!judgeBarrier(_maze.recordMap[x][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x][y+1]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x][y-1]))
                                                {
                                                        return false;
                                                }
                                        }
                                        return true;
                                }
                                if(angle > 1)
                                {
                                        temp = calcXSlop(starPoint->x, starPoint->y, endPoint->x, endPoint->y);
                                        k = temp.first;
                                        b = temp.second;
                                        float x;
                                        for(int y=starPoint->y; y>endPoint->y; y--)
                                        {
                                                // if(y == starPoint->y)
                                                //         continue;
                                                x = k * y + b;
                                                x = round(x);
                                                if(!judgeBarrier(_maze.recordMap[x][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x+1][y]))
                                                {
                                                        return false;
                                                }
                                                if(!judgeBarrier(_maze.recordMap[x-1][y]))
                                                {
                                                        return false;
                                                }
                                        }
                                        return true;
                                }
                        }
                }
        }
        
        void aStar::optimizePath(list<Point*> &path)
        {
                FRIZY_LOG(LOG_DEBUG, "start optimizePath");
                list<Point*>::iterator temp;
                list<Point*>::iterator it = path.begin();
                list<Point*>::iterator next = ++path.begin();
                list<Point*> tempPath;
                int flag = 0;
                long long start_time, cost_time;
                start_time = GetCurrentTime();
                while(1)
                {
                    if(next == path.end())
                    {
                        if(flag == 0)
                        {
                            if(it != path.end())
                            {    
                                it++;
                                if(it == path.end())
                                    break;
                                else
                                {
                                    next = it;
                                    next ++;
                                    if(next == path.end())
                                        break;
                                }
                            }
                        }
                        else
                        {
                            if((*temp)->x == path.back()->x && (*temp)->y == path.back()->y)
                                {
                                    tempPath.remove(*temp);
                                }
                            break;
                        }
                    }
                    // FRIZY_LOG(LOG_DEBUG, "it(%d %d), next(%d %d)", (*it)->x, (*it)->y, (*next)->x, (*next)->y);
                    if(judgePath(*it, *next))
                    {
                        flag = 1;
                        tempPath.push_back(*next);
                        temp = next;
                        next++;
                    }
                    else
                    {  
                        if(flag == 1)
                        {
                            it = temp;
                            tempPath.remove(*temp);
                        }
                        else
                        {
                            next++;
                        }
                        flag = 0;
                    }
                }
                cost_time = GetCurrentTime() - start_time;
                FRIZY_LOG(LOG_INFO, "optimize path cost %lld ms", cost_time);
                for(auto p : tempPath)  
                {
                    path.remove(p);
                }
                // while(1)
                // {
                    // if(flag == 1)
                    // {
                    //         next++;
                    //         if(next == path.end())
                    //         {
                    //                 tempPath.remove(*temp);
                    //                 break;
                    //         }
                    // }
                    // FRIZY_LOG(LOG_DEBUG,"it:(%d,%d) next:(%d,%d) tmpPath.size:%d",(*it)->x, (*it)->y, (*next)->x, (*next)->y, tempPath.size());
                    // if(judgePath(*it, *next))
                    // {
                    //         temp = next;
                    //         tempPath.push_back(*next);
                    //         flag = 1;
                    // }
                    // else
                    // {
                    //         if(tempPath.empty())
                    //         {
                    //             FRIZY_LOG(LOG_ERROR,"optimize failed1");
                    //             return;
                    //         }
                    //         tempPath.remove(*temp);
                    //         if((*it)->x == (*temp)->x && (*it)->y == (*temp)->y)
                    //         {
                    //             for(auto p : tempPath)
                    //             {
                    //                     path.remove(p);
                    //             }
                    //             FRIZY_LOG(LOG_ERROR,"optimize failed2");
                    //             break;
                    //         }
                    //         it = temp;
                    //         flag = 0;
                    // }
                // }
                // for(auto p : tempPath)  
                // {
                //         path.remove(p);
                // }
                FRIZY_LOG(LOG_DEBUG, "optimizePath finished");
        }












//DIA*类的构造函数      还存在BUG？？   
        idaStar::idaStar() : GlobalPlanning() {}
        idaStar::~idaStar() {}

        int idaStar::calcG(Point *point)
        {
                int thisG = 1;
                int parentG = point->parent == NULL ? 0 : point->parent->G;
                return thisG + parentG;
        }

        int idaStar::calcH(Point *point, Point *endPoint)
        {	
                //曼哈顿距离
                return abs(point->x - endPoint->x) + abs(point->y - endPoint->y)*kCost1;
                //欧几里得距离
                //return sqrt((point->x - endPoint->x)*(point->x - endPoint->x) + (point->y - endPoint->y)*(point->y - endPoint->y))*kCost1;
        }
        
        int idaStar::calcF(Point *point)
        {
                return point->G + point->H;
        }
        Point* idaStar::findPath(Point &startPoint, Point &endPoint, RobotType &robotShape)
        {
                //计算起点到终点的f值，设为初始估价量
                startPoint.G = calcG(&startPoint);
                startPoint.H = calcH(&startPoint, &endPoint);
                startPoint.F = calcF(&startPoint);
                int startStep = startPoint.F;

                Point *result;
                while ((result = idaSerach(&startPoint, endPoint, startStep)) == NULL)
                {
                        //判断是否地图已全部遍历
                        bool noPath = true;
                        for (int i = 0; i < _maze.rows; i++)
                        {
                                for (int j = 0; j < _maze.cols; j++)
                                {
                                        if (_maze.Map[i][j]->n == 0)
                                                noPath = false;
                                }
                        }
                        if (noPath)
                                return NULL;

                        //在此估价量下找不到终点，则将估价量++
                        startStep++;
                        //
                        step = -1;
                        for (int i = 0; i < _maze.rows; i++)
                        {
                                for (int j = 0; j < _maze.cols; j++)
                                {
                                        _maze.Map[i][j]->parent = NULL;
                                        if (_maze.Map[i][j]->n < 0)
                                                _maze.Map[i][j]->n = 0;
                                }
                        }
                }

                return result;
        }
        Point* idaStar::idaSerach(Point *point, Point &endPoint, int maxF)
        {
                //若该点的F值>预设的最大F值，则返回
                if (point->F > maxF)
                        return NULL;	

                //如果到达终点则返回终点的指针
                if (point->x == endPoint.x && point->y == endPoint.y)
                        return point;

                //将这个点进行标记，并用计数器step表示遍历的是第几个点
                if (point->x != _maze.startPoint.x || point->y != _maze.startPoint.y)
                {
                        if (point->n == 0)
                        {
                                point->n = step;
                                step--;
                        }
                }

                //得到当前点周围能到达的所有点
                vector<Point*> surroundPoints = getSurroundPoints(point);
                //遍历能够到达的所有点
                for (auto &target : surroundPoints)
                {
                        //
                        if (target->parent == NULL)
                        {
                                target->parent = point;
                                target->G = calcG(target);
                                target->H = calcH(target, &endPoint);
                                target->F = calcF(target);
                        }
                        else
                        {
                                int tempG = point->G + 1;
                                if (tempG < target->G)
                                {
                                        target->parent = point;
                                        target->G = tempG;
                                        target->F = calcF(target);
                                }
                        }

                        Point *result;
                        //递归
                        result = idaSerach(target, endPoint, maxF);
                        if (result != NULL)
                                return result;
                }
                return NULL;
        }
        list<Point*> idaStar::getPath(Point *result)
        {
                keyStep = 11;
                list<Point*> path;
                while (result)
                {
                        if ((result->x != _maze.startPoint.x || result->y != _maze.startPoint.y) && (result->x != _maze.endPoint.x || result->y != _maze.endPoint.y))
                        {
                                result->n = keyStep;
                                keyStep++;
                        }
                        path.push_front(result);
                        result = result->parent;
                }
                return path;
        } 
        vector<Point*> idaStar::getSurroundPoints(const Point *point) const
        {
                vector<Point*> surroundPoints;

                int dx[4] = { 0,-1,0,1 };
                int dy[4] = { 1,0,-1,0 };

                for (int i = 0; i<4; i++)
                {
                        int x = point->x + dx[i];
                        int y = point->y + dy[i];
                        if (isCanReach(point, _maze.Map[x][y]))
                        {
                                surroundPoints.push_back(_maze.Map[x][y]);
                        }

                }

                return surroundPoints;
        } 
        bool idaStar::isCanReach(const Point *point, const Point *target) const
        {
                if ((target->n == 0 || target->n == 3) && target->x >= 1 && target->x <= _maze.rows && target->y >= 1 && target->y <= _maze.cols)
                        return true;
                return false;
        }              
}

