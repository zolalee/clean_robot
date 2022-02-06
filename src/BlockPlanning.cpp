/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-07 17:40:21
 * @Project      : UM_path_planning
 */

#include "navigation_algorithm/BlockPlanning.h"

#include "navigation_algorithm/UTurnPlanning.h"
#include "common_function/MapFunction.h"

extern useerobot::Maze _maze;
namespace useerobot
{
    int deta = 0;
    int tmpAdd = 0;
    int justwind = 0;
    UTurnPlanning planBlock;
    extern PRO process;
    mapRange _map;
    vector <Grid> boundPoint;
    vector <Grid> rollOb;
    vector <boundary> boundArr;
    boundary limit = {1000,-1000,1000,-1000};    
    
    int laxi = 0;
    blockPlanning::blockPlanning(/* args */)
    {
    }


    blockPlanning::~blockPlanning()
    {
      
    }
    void blockPlanning::init()
    {
        vector <boundary>().swap(boundArr);
        vector <Grid>().swap(boundPoint);
        limit = {1000,-1000,1000,-1000};
        _map = {-1000,1000,-1000,1000};
        laxi = 0;
        divBound = DOWN;
        divWall = IDLE;
    }

    #define GRATE_V 600
    //计算光栅
    int blockPlanning::CalGrating(Sensor sensor,Grid cur,int forward)
    {
        
        int16_t templetf = sensor.leftw;
        int16_t tempright = sensor.rightw;
        float_t tempangle = ((360 - cur.forward) * 3.1415)/180;
        
        grate.grating.x = grate.grating.x + ((templetf + tempright) * cos(tempangle)) / 2;
        
        grate.grating.y = grate.grating.y + ((templetf + tempright) * sin(tempangle)) / 2;

        grate.grateTime ++;

        FRIZY_LOG(LOG_DEBUG,"grating.x == %d,grating.y == %d,time.%d,grating.dx == %d,grating.dy == %d,"
            ,grate.grating.x,grate.grating.y,grate.grateTime,grate.grating.dx,grate.grating.dy);
        
        if (forward == 1)
        {
          if (abs(grate.grating.x) > GRATE_V && grate.grateSign != 2)
          {
              FRIZY_LOG(LOG_DEBUG,"grating1");
              grate.grateSign = 1;
              if (abs(grate.grating.x) > grate.grateMax)
                grate.grateMax = abs(grate.grating.x);	
          }
          if (abs(grate.grating.dx - cur.dx) > 200 && grate.grateTime > 10)
          {
            FRIZY_LOG(LOG_DEBUG,"LDS1");
            grate.ldsSign = 1;
          }
          
          if (grate.grateSign == 1 && grate.ldsSign == 1
             && (abs(grate.grating.x) < 200 || abs(grate.grating.x) < (grate.grateMax/3)))
          {
            FRIZY_LOG(LOG_DEBUG,"grate1");
            grate.grateSign = 2;
          }
        }
        else
        {
          if (abs(grate.grating.y) > GRATE_V && grate.grateSign != 2)
          {
              FRIZY_LOG(LOG_DEBUG,"grating2");

              grate.grateSign = 1;
              if (abs(grate.grating.y) > grate.grateMax)
                grate.grateMax = abs(grate.grating.y);	
          }
          if (abs(grate.grating.dy - cur.dy) > 200 && grate.grateTime > 10)
          {
            FRIZY_LOG(LOG_DEBUG,"LDS2");
            grate.ldsSign = 1;
          }
          if (grate.grateSign == 1 && grate.ldsSign == 1
             && (abs(grate.grating.y) < 200 || abs(grate.grating.y) < (grate.grateMax/3)))
          {
            FRIZY_LOG(LOG_DEBUG,"grate2");
            grate.grateSign = 2;
            
          }
        }

        if (grate.grateSign == 2){

            return 1;
        }
            
        else{

            return 0;
        
        }
    }
            


    //设置一圈的数据
    int blockPlanning::SetInformation(Sensor sensor,Grid cur)
    {
        // if (boundPoint.size() == 0)
        //     StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
        

        if (boundPoint.size() == 0){
            FRIZY_LOG(LOG_DEBUG,"kaishile.%d",cur.addAngle);
            blockArrive = 0;
            raoCount = 0;
            conIndex = 0;
            deta = 0;
        }

        //重新划定区域
        if (boundPoint.size() == 0 && boundArr.size() != 0)
        {
            
            limit = {1000,-1000,1000,-1000};
            justwind = 0;
            int temps = 0;
            
            for (int i = 0;i < boundArr.size();i++)
            {
                if (cur.x == boundArr[i].up && cur.y > boundArr[i].right && cur.y < boundArr[i].left
                  && _maze.GetMapState(cur.x+1,cur.y,2) == 0 && _maze.GetMapState(cur.x-1,cur.y,2) != 0)
                {
                    temps = 1;
                    limit.up = cur.x + AREA_LENGTH;
                    limit.down = cur.x; 

                    limit.left = boundArr[i].left;
                    limit.right = boundArr[i].right;

                }
                if (cur.x == boundArr[i].down && cur.y > boundArr[i].right && cur.y < boundArr[i].left
                  && _maze.GetMapState(cur.x-1,cur.y,2) == 0 && _maze.GetMapState(cur.x+1,cur.y,2) != 0)
                {
                    temps = 1;
                    limit.up = cur.x;
                    limit.down = cur.x - AREA_LENGTH;

                    limit.left = boundArr[i].left;
                    limit.right = boundArr[i].right;                    
                }
                if (cur.y == boundArr[i].left && cur.x > boundArr[i].down && cur.x < boundArr[i].up
                  && _maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x,cur.y-1,2) != 0)
                {
                    temps = 1;
                    limit.left = cur.y + AREA_LENGTH;
                    limit.right = cur.y;

                    limit.up = boundArr[i].up;
                    limit.down = boundArr[i].down;
                }
                if (cur.y == boundArr[i].right && cur.x > boundArr[i].down && cur.x < boundArr[i].up
                  && _maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x,cur.y+1,2) != 0)
                {
                    temps = 1;
                    limit.left = cur.y;
                    limit.right = cur.y - AREA_LENGTH;

                    limit.up = boundArr[i].up;
                    limit.down = boundArr[i].down;
                }                             
            }

            FRIZY_LOG(LOG_DEBUG,"new.%d.%d.%d.%d",limit.up,limit.down,limit.left,limit.right);

            if (temps == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"fk");
                process = PLAN;
                _aim.kind = searchBound;
                planBlock.PlanSearch(_aim,-1);
                return 1;
            }
        }
       
        //将一圈的坐标数据存入数组
        auto it = find(boundPoint.begin(),boundPoint.end(),cur);
        if (it == boundPoint.end())
        {
            FRIZY_LOG(LOG_DEBUG,"cur.%d.%d // %f//%d//%d"
            ,cur.x,cur.y,cur.forward,cur.addAngle,boundPoint.size()); 

            if (IsWall() == 0
                && (_maze.GetMapState(cur.x+1,cur.y,2) == 1 && _maze.GetMapState(cur.x-1,cur.y,2) == 1 
					&& _maze.GetMapState(cur.x,cur.y+1,2) == 1 && _maze.GetMapState(cur.x,cur.y-1,2) == 1
					&& _maze.GetMapState(cur.x+1,cur.y+1,2) == 1 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 1
					&& _maze.GetMapState(cur.x-1,cur.y+1,2) == 1 && _maze.GetMapState(cur.x-1,cur.y-1,2) == 1))
            {
                FRIZY_LOG(LOG_DEBUG,"continuous1");
                continues ++;
            }
            else if (IsWall() != 0
				&& (cur.x < limit.down || cur.x > limit.up || cur.y < limit.right || cur.y > limit.left))
            {
                FRIZY_LOG(LOG_DEBUG,"continuous2");
                continues ++;
            }
            else
            {
                continues = 0;
            }
        
            boundPoint.push_back(cur);
        }
        
        
        
        FRIZY_LOG(LOG_DEBUG,"start.%d.%d.%d",boundPoint[0].x,boundPoint[0].y,boundPoint[0].addAngle);

        FRIZY_LOG(LOG_DEBUG,"last_start.%d.%d.%d",last_start.x,last_start.y,last_start.addAngle);
        
        //获取地图的最大最小值
        if (cur.x > _map.xmax) _map.xmax = cur.x;
        if (cur.x < _map.xmin) _map.xmin = cur.x;
        if (cur.y > _map.ymax) _map.ymax = cur.y;
        if (cur.y < _map.ymin) _map.ymin = cur.y;


		if (limit.left == 1000 && limit.up == 1000 && laxi != 5)
		{
            if (boundPoint[0].x < cur.x - 8 && laxi == 0)
                    laxi = 2;	
                            
            if (boundPoint[0].x > cur.x + 8 && laxi == 0)
                    laxi = 1;
                
            if (boundPoint[0].y < cur.y - 8 && laxi == 0)
                    laxi = 4;	
            
            if (boundPoint[0].y > cur.y + 8 && laxi == 0)
                    laxi = 3;
                        
            printf("laxi.%d.%d\n",laxi,cur.addAngle - boundPoint[0].addAngle);
            
            if (laxi == 1)
            {
                if (boundPoint[0].x == cur.x && cur.addAngle - boundPoint[0].addAngle > 100)
                {
                    laxi = 0;
                    limit.up = cur.x;
                    limit.down = cur.x - AREA_LENGTH;
                }
                if (cur.x > boundPoint[0].x)
                    laxi = 5;
            }
            
            if (laxi == 2)
            {
                if (boundPoint[0].x == cur.x && cur.addAngle - boundPoint[0].addAngle > 100)
                {
                    laxi = 0;
                    limit.down = cur.x;
                    limit.up = cur.x + AREA_LENGTH;

                }
                if (cur.x < boundPoint[0].x)
                    laxi = 5;
            }			
            if (laxi == 3)
            {
                if (boundPoint[0].y == cur.y && cur.addAngle - boundPoint[0].addAngle > 100)
                {
                    laxi = 0;
                    limit.left = cur.y;
                    limit.right = cur.y - AREA_LENGTH;
                }
                if (cur.y > boundPoint[0].y)
                    laxi = 5;
            }
            if (laxi == 4)
            {
                if (boundPoint[0].y == cur.y && cur.addAngle - boundPoint[0].addAngle > 100)
                {
                    laxi = 0;
                    limit.right = cur.y;
                    limit.left = cur.y + AREA_LENGTH;
                }
                if (cur.y < boundPoint[0].y)
                    laxi = 5;
            }		
		}


        //前方是之前的边界,不能再走了,需要立刻给出边界
        if (boundArr.size() > 0)
        {
            for (int i = 0;i < boundArr.size();i++)
            {
                if (limit.left == 1000 && (_maze.GetMapState(cur.x+1,cur.y,2) != 0 || _maze.GetMapState(cur.x-1,cur.y,2) != 0))
                {
                    if (boundArr[i].left == cur.y)
                    {
                        if (cur.forward > 0 && cur.forward < 180)
                        {
                        FRIZY_LOG(LOG_DEBUG,"exx1\n");
                        limit.left = cur.y + AREA_LENGTH;
                        limit.right = cur.y; 
                        break;     
                        }                  
                    }
                    
                    if (boundArr[i].right == cur.y)
                    {

                        if (cur.forward < 360 && cur.forward > 180)
                        {
                        FRIZY_LOG(LOG_DEBUG,"exx2\n");
                        limit.left = cur.y;
                        limit.right = cur.y - AREA_LENGTH;    
                        break; 
                        }                   
                    }
                }
                if (limit.up == 1000 && (_maze.GetMapState(cur.x,cur.y+1,2) != 0 || _maze.GetMapState(cur.x,cur.y-1,2) != 0))
                {
                    if (boundArr[i].up == cur.x)
                    {
                        if (cur.forward > 90 && cur.forward < 270)
                        {
                        
                        FRIZY_LOG(LOG_DEBUG,"exx3\n");
                        limit.up = cur.x + AREA_LENGTH;
                        limit.down = cur.x; 
                        break;   
                        }                    
                    }
                    if (boundArr[i].down == cur.x)
                    {
                        if (cur.forward < 90 || cur.forward > 270)
                        {

                            FRIZY_LOG(LOG_DEBUG,"exx4\n");
                            limit.up = cur.x;
                            limit.down = cur.x - AREA_LENGTH;    
                            break;  
                        }                  
                    }
                }
   
            }
        }

        if (IsWall() == 0 && cur.x == limit.up && cur.y > limit.right && cur.y < limit.left && divBound == UP
            && _maze.GetMapState(cur.x+1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x-1,cur.y+1,2) != 0
            && _maze.GetMapState(cur.x+1,cur.y+2,2) != 0 && _maze.GetMapState(cur.x-1,cur.y+2,2) != 0
            && last_start == boundPoint[0]
            // && !planBlock.CheckFor(cur,DOWN)
            // && (cur.x != boundPoint[0].x || cur.y != boundPoint[0].y)
            )  
            {
                last_start.x = 1000;
                FRIZY_LOG(LOG_DEBUG,"exin1\n");
                limit.left = cur.y;
                limit.right = cur.y - AREA_LENGTH;
            } 
        if (IsWall() == 0 && cur.x == limit.down && cur.y > limit.right && cur.y < limit.left && divBound == DOWN
            && _maze.GetMapState(cur.x-1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x+1,cur.y-1,2) != 0
            && _maze.GetMapState(cur.x-1,cur.y-2,2) != 0 && _maze.GetMapState(cur.x+1,cur.y-2,2) != 0
            && last_start == boundPoint[0]
            // && !planBlock.CheckFor(cur,UP)
            // && (cur.x != boundPoint[0].x || cur.y != boundPoint[0].y)
            )  //
            {
                last_start.x = 1000;
                FRIZY_LOG(LOG_DEBUG,"exin2\n");
                limit.right = cur.y;
                limit.left = cur.y + AREA_LENGTH;
            } 
        if (IsWall() == 0 && cur.y == limit.left && cur.x > limit.down && cur.x < limit.up && divBound == LEFT
            && _maze.GetMapState(cur.x-1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x-1,cur.y-1,2) != 0
            && _maze.GetMapState(cur.x-2,cur.y+1,2) != 0 && _maze.GetMapState(cur.x-2,cur.y-1,2) != 0
            && last_start == boundPoint[0]
            // && !planBlock.CheckFor(cur,RIGHT)
            // && (cur.x != boundPoint[0].x && cur.y != boundPoint[0].y))//
            )
            {
                last_start.x = 1000;
                FRIZY_LOG(LOG_DEBUG,"exin3\n");
                limit.down = cur.x;
                limit.up = cur.x + AREA_LENGTH;
            } 
        if (IsWall() == 0 && cur.y == limit.right && cur.x > limit.down && cur.x < limit.up && divBound == RIGHT
            && _maze.GetMapState(cur.x+1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x+1,cur.y+1,2) != 0
            && _maze.GetMapState(cur.x+2,cur.y-1,2) != 0 && _maze.GetMapState(cur.x+2,cur.y+1,2) != 0
            // && !planBlock.CheckFor(cur,LEFT)
            // && (cur.x != boundPoint[0].x && cur.y != boundPoint[0].y))//
            && last_start == boundPoint[0]
            )
            {
                last_start.x = 1000;
                FRIZY_LOG(LOG_DEBUG,"exin4\n");
                limit.up = cur.x;
                limit.down = cur.x - AREA_LENGTH;
            } 

        //通过上下左右极值确定边界
        if (_map.xmax - _map.xmin == AREA_LENGTH - 3 && limit.up == 1000)
        {
            if (_map.xmax == cur.x){
                limit.up = _map.xmax;
                limit.down = _map.xmax - AREA_LENGTH;
            }
            else{
                limit.up = _map.xmin + AREA_LENGTH;
                limit.down = _map.xmin;                
            }

            
            printf("road:boundup == %d,bounddown == %d\n",limit.up,limit.down);
        }
        if (_map.ymax - _map.ymin == AREA_LENGTH - 3 && limit.left == 1000)
        {
            if (_map.ymax == cur.y){
                limit.left = _map.ymax;
                limit.right = _map.ymax - AREA_LENGTH;
            }
            else{
                limit.left = _map.ymin + AREA_LENGTH;
                limit.right = _map.ymin;                
            }
            
            printf("road:boundleft == %d,boundright == %d\n",limit.left,limit.right);
        }  
        
        //判断机器位于上下左右哪条边界
        if (cur.x == limit.down && cur.y > limit.right && cur.y <= limit.left)
        {
            printf("xia\n");
            divBound = DOWN;
        }
        if (cur.y == limit.right && cur.x >= limit.down && cur.x < limit.up)
        {
            printf("you\n");
            divBound = RIGHT;
        }
        if (cur.x == limit.up && cur.y >= limit.right && cur.y < limit.left)
        {
            printf("shang\n");
            divBound = UP;
        }
        if (cur.y == limit.left && cur.x > limit.down && cur.x <= limit.up)
        {
            printf("zuo\n");
            divBound = LEFT;
        }

		//防止小区域重复划区
		if (abs(cur.x - boundPoint[0].x) + abs(cur.y - boundPoint[0].y) < 2)
		{
			if (blockArrive == 1)	
				blockArrive = 2;
			if (blockArrive == 3)	
				blockArrive = 4;	
			if (blockArrive == 5)	
				blockArrive = 6;	
						
		}
		
		if (abs(cur.x - boundPoint[0].x) + abs(cur.y - boundPoint[0].y) >= 2)
		{
			if (blockArrive == 0)
				blockArrive = 1;
			if (blockArrive == 2)
				blockArrive = 3;
			if (blockArrive == 4)
				blockArrive = 5;
		}




        return 0;
    }



    void blockPlanning::GetBound(vector <boundary>* arr,boundary* stu)
    {
       

        (*arr).clear();
        for (int i = 0;i < boundArr.size();i++)
        {
            //boundary temp = boundArr[i];
            
            (*arr).push_back(boundArr[i]);
            // (*arr)[i].up =  boundArr[i].up;
            // (*arr)[i].down =  boundArr[i].down;
            // (*arr)[i].left =  boundArr[i].left;
            // (*arr)[i].right =  boundArr[i].right;
        }
        stu -> up = limit.up;
        stu -> down = limit.down;
        stu -> left = limit.left;
        stu -> right = limit.right;
    }

    void blockPlanning::SetBound(boundary stu)
    {
        limit = stu;
    }

    //某些原因导致机器沿出边界，需要找到最近的边界点
    void blockPlanning::BackBound(Grid cur)
    {

		FRIZY_LOG(LOG_DEBUG,"BackBound");

        vector <Grid> array;

        Grid temp;

		//开始搜集需要返回的点
		for (int x = limit.down;x <= limit.up;x ++)
		{
            if (_maze.GetMapState(x,limit.left,2) != 2)
            {
                temp.x = x;
                temp.y = limit.left;
				array.push_back(temp);
            }
            if (_maze.GetMapState(x,limit.right,2) != 2)
            {
                temp.x = x;
                temp.y = limit.right;
				array.push_back(temp);					
            }
		}
		
		for (int y = limit.right;y <= limit.left;y ++)
		{
            if (_maze.GetMapState(limit.up,y,2) != 2)
            {
                temp.x = limit.up;
                temp.y = y;
				array.push_back(temp);
            }
            if (_maze.GetMapState(limit.down,y,2) != 2)
            {
                temp.x = limit.down;
                temp.y = y;
				array.push_back(temp);					
            }
		}	
		
		if (array.size() == 0)
		{
            FRIZY_LOG(LOG_DEBUG,"fk2");
            process = PLAN;
            return;
		}
		
        int dis = 1000;
		for (int i = 0;i < array.size();i++)
		{
            if (dis > abs(cur.x - array[i].x) + abs(cur.y - array[i].y))
            {
                dis = abs(cur.x - array[i].x) + abs(cur.y - array[i].y);
                _aim.x = array[i].x;
                _aim.y = array[i].y;
            }
		}

        _aim.kind = backBound;

        printf("tempX == %d,tempY == %d\n",_aim.x,_aim.y);

        roadBlock.SetroadAim(_aim);

        process = ROAD;        
		
		return;
    }


    void blockPlanning::GetBoundOb(Grid cur,Trouble trouble)
    {
        //printf("trouble.%d,%d,%d,%d\n",trouble._maprange.xmax,trouble._maprange.xmin,trouble._maprange.ymax,trouble._maprange.ymin);
        if (cur.x >= trouble._maprange.xmin && cur.x <= trouble._maprange.xmax 
            && cur.y >= trouble._maprange.ymin && cur.y <= trouble._maprange.ymax)
		    return;

        if (cur.x >= limit.up || cur.x <= limit.down || cur.y >= limit.left || cur.y <= limit.right)
            return;
        
        if (_maze.GetMapState(cur.x,cur.y,2) == 2)
        {
            auto it = find(rollOb.begin(),rollOb.end(),cur);
            if (it == rollOb.end())
                rollOb.push_back(cur);
        }
    }


    int blockPlanning::FullBound(Grid cur,Trouble trouble)
    {
        for (int i = 0;i < boundPoint.size();i++)
        {
            if (abs(boundPoint[i].x - cur.x) + abs(boundPoint[i].y - cur.y) == 0 
                && i > 10 && boundPoint.size() > i + 1
                && (abs(boundPoint[i].forward - cur.forward) < 30 || abs(boundPoint[i].forward - cur.forward) > 330)
                && boundPoint.size() > i + 3*boundPoint.size()/4
                && cur.addAngle - boundPoint[0].addAngle < 0)
                // && cur.addAngle < boundPoint[i].addAngle - 200)
            {   
                FRIZY_LOG(LOG_DEBUG,"manle2");
                return 1;
            }						
        }
        if ((abs(cur.x - boundPoint[0].x) + abs(cur.y - boundPoint[0].y) < 2
                && cur.addAngle - boundPoint[0].addAngle < 0
                && boundPoint.size() > 10)
            || cur.addAngle - boundPoint[0].addAngle < -1440)
        {
            FRIZY_LOG(LOG_DEBUG,"manle1.%d",cur.addAngle - boundPoint[0].addAngle);
            return 1;
        }
        return 0;    
    }
    int blockPlanning::ReMove(Grid cur,DIVB forward)
    {
        int16_t tempindex = 0;
        for (int i = 0;i < boundPoint.size()-1;i++)
        {					
            if (cur.x == boundPoint[i].x && cur.y == boundPoint[i].y)
            {
                tempindex = i;
                FRIZY_LOG(LOG_DEBUG,"yejinqu.%d",tempindex);
                break;
            }
        }
        if (tempindex == boundPoint.size() - 1)
        {
            FRIZY_LOG(LOG_DEBUG,"keyikeyi");
            return 0;    
        }
        FRIZY_LOG(LOG_DEBUG,"x1.%d,y1.%d,x2.%d,y2.%d,tempindex.%d"
        ,boundPoint[tempindex].x,boundPoint[tempindex].y,boundPoint[tempindex+1].x,boundPoint[tempindex+1].y,tempindex);
            
        //xia
        if (forward == DOWN)
        {
            if ((boundPoint[tempindex+1].x == cur.x && boundPoint[tempindex+1].y == cur.y - 1)
                || (cur.x == boundPoint[0].x && cur.y - 1 == boundPoint[0].y && cur.forward > 225))
            {
                printf("chongfule1.%d.%d\n",cur.x,cur.y);
                return 1;
            }
            else
                return 0;
        }
        //you
        if (forward == RIGHT)
        {
            if ((boundPoint[tempindex+1].x == cur.x + 1 && boundPoint[tempindex+1].y == cur.y)
                || (cur.x + 1 == boundPoint[0].x && cur.y == boundPoint[0].y && cur.forward > 135 && cur.forward < 270))
            {
                    printf("chongfule2.%d.%d\n",cur.x,cur.y);
                    return 1;
            }
            else
                return 0;
        }	
        //shang
        if (forward == UP)
        {
            if ((boundPoint[tempindex+1].x == cur.x && boundPoint[tempindex+1].y == cur.y + 1)
                || (cur.x == boundPoint[0].x && cur.y + 1 == boundPoint[0].y && cur.forward > 45 && cur.y < 180))
            {
                printf("chongfule3.%d.%d\n",cur.x,cur.y);
                return 1;
            }
            else
                return 0;
        }		
        //zuo
        if (forward == LEFT)
        {
            if ((boundPoint[tempindex+1].x == cur.x - 1 && boundPoint[tempindex+1].y == cur.y)
                || (cur.x - 1 == boundPoint[0].x && cur.y == boundPoint[0].y 
                        && ((cur.forward > 0 && cur.forward < 90) || cur.forward > 315)))
            {
                    printf("chongfule4.%d.%d\n",cur.x,cur.y);
                    return 1;
            }
            else
                return 0;
        }			

    }


    void blockPlanning::DivArea(Sensor sensor,Grid cur,Trouble trouble)
    {
        _trouble = trouble;

        if (SetInformation(sensor,cur))
            return;


        bool tmpS = abs(_map.xmax - _map.xmin) + abs(_map.ymax - _map.ymin) < 10 ? 
                    true:false;


        if ((FullBound(cur,trouble) == 1 && !tmpS)
            || continues > 6 || raoCount > 4
            || (tmpS && cur.addAngle - boundPoint[0].addAngle < -1440))
        {
            FRIZY_LOG(LOG_DEBUG,"manleo.%d.%d.%d.%d,continues.%d",boundPoint[0].x,boundPoint[0].y
            ,boundPoint[0].addAngle,boundPoint.size(),continues);

            last_start = boundPoint[0];
            continues = 0;
            _map.xmax = -1000,_map.xmin = 1000,_map.ymax = -1000,_map.ymin = 1000;

            boundArr.push_back(limit);

            boundPoint.clear();
            
            _aim.kind = searchInside;
            planBlock.PlanSearch(_aim,-1);
            process = PLAN;
            return;
        }

        if (trouble.type == smallarea)
        {
            FRIZY_LOG(LOG_DEBUG,"xiaofanweila");
            _trouble.type = nothing;
            for (int i = 0;i < boundPoint.size();i++)
                boundPoint[i].addAngle = boundPoint[i].addAngle - 360;               
        }
        //绕柱
        if (_trouble.type == windcolumn)
        {
            raoCount++;
            _trouble.type = nothing;
            rollOb.clear();

            FRIZY_LOG(LOG_DEBUG,"raozhula.%d.%d.%d.%d"
                ,_trouble._maprange.xmax,_trouble._maprange.xmin,_trouble._maprange.ymax,_trouble._maprange.ymin);
            //printf("trouble.%d,%d,%d,%d\n",trouble._maprange.xmax,trouble._maprange.xmin,trouble._maprange.ymax,trouble._maprange.ymin);
            for (int i = 0;i < boundPoint.size();i++)
                boundPoint[i].addAngle = boundPoint[i].addAngle + 270;  

            Grid temp;
            //获取rollOb
            for (int i = conIndex;i < boundPoint.size();i++)
            { 
                //
                             
                for (int x = boundPoint[i].x-1;x <= boundPoint[i].x+1;x++)
                {
                    for (int y = boundPoint[i].y-1;y <= boundPoint[i].y+1;y++)
                    {
                        if (x == boundPoint[i].x && y == boundPoint[i].y)
                            continue;
                        if ((x == boundPoint[i].x+1 && y == boundPoint[i].y+1)
                            || (x == boundPoint[i].x+1 && y == boundPoint[i].y-1)
                            || (x == boundPoint[i].x-1 && y == boundPoint[i].y+1)
                            || (x == boundPoint[i].x-1 && y == boundPoint[i].y-1))
                            continue;
                        temp.x = x;
                        temp.y = y;
                        GetBoundOb(temp,trouble);
                    }
                }
            }
            
            for (int i = 0;i < rollOb.size();i++)
                FRIZY_LOG(LOG_DEBUG,"rollOb1.%d.%d.%d",rollOb[i].x,rollOb[i].y,rollOb.size());

            int tempsize = rollOb.size()-1;
            FRIZY_LOG(LOG_DEBUG,"rollOb.size()-1 = %d,%d",rollOb.size()-1,tempsize);
            
            if (tempsize > 0)
            {
                FRIZY_LOG(LOG_DEBUG,"jinqule");
                for (int i = 0;i < (tempsize);i++)
                {
                    FRIZY_LOG(LOG_DEBUG,"rollOb1.%d.%d.%d",rollOb[i].x,rollOb[i].y,rollOb.size());
                    
                    // if (abs(rollOb[i].x - rollOb[i+1].x) + abs(rollOb[i].y - rollOb[i+1].y) > 3)
                    // {
                    //     //删除区间[i,j];
                    //     rollOb.erase(rollOb.begin()+i+1,rollOb.end()); 
                    //     break;
                    // }
                }

            }
            //倒序
            reverse(rollOb.begin(),rollOb.end());
            // for (int i = 0;i < rollOb.size()/2;i++)
            // {
            //     temp = rollOb[i];
            //     rollOb[i] = rollOb[rollOb.size()-1-i];
            //     rollOb[rollOb.size()-1-i] = temp;
            // }

            for (int i = 0;i < rollOb.size();i++)
                FRIZY_LOG(LOG_DEBUG,"rollOb2.%d.%d.%d",rollOb[i].x,rollOb[i].y,rollOb.size());

            do{
                FRIZY_LOG(LOG_DEBUG,"raostop..");
                StopWallFollow();
                chassisBlock.GetSensor(&sensor);
                usleep(10 * 1000);
            }while (sensor.leftw != 0 || sensor.rightw != 0);  
            
            if (rollOb.size() > 5  
                || ((limit.up != 1000 || limit.left != 1000 || boundArr.size() != 0) && rollOb.size() > 2))
            {
                tmpAdd = cur.addAngle;

                _aim.kind = column;

                if (rollOb.size() > 5)
                    _aim.x = rollOb[5].x,_aim.y = rollOb[5].y;
                else
                    _aim.x = rollOb.back().x,_aim.y = rollOb.back().y;     
                
                FRIZY_LOG(LOG_DEBUG,"pathlength > 5.%d.%d.%d,tmpAdd.%d",_aim.x,_aim.y,rollOb.size(),tmpAdd);

                roadBlock.SetroadAim(_aim);
                process = ROAD;
                return;

            }

            else if (rollOb.size() <= 2 && (limit.up != 1000 || limit.left != 1000 || boundArr.size() != 0))
            {
   
                // _aim.x = rollOb.back().x;
                // _aim.y = rollOb.back().y; 

                if ((divBound == DOWN && limit.down != -1000) // && abs(cur.x - limit.down) < 5
                    || (divBound == UP && limit.up != 1000) // && abs(cur.x - limit.up) < 5
                    || (divBound == LEFT && limit.left != 1000) // && abs(cur.y - limit.left) < 5
                    || (divBound == RIGHT && limit.right != -1000)) // && abs(cur.x - limit.right) < 5)
                {
                    FRIZY_LOG(LOG_DEBUG,"fanren");
                    _aim.kind = backBound;
                    planBlock.PlanSearch(_aim,-1);
                    process = PLAN;
                    return;
                }
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"manle3");
                    boundArr.push_back(limit);
                    boundPoint.clear();
                    _aim.kind = searchInside;
                    planBlock.PlanSearch(_aim,-1);
                    process = PLAN;
                    return;
                }
            }
            else
            {
                //直接劈出一片新的区域
                FRIZY_LOG(LOG_DEBUG,"pathlength < 5.%d",rollOb.size());

                do{
                    StopWallFollow();
                    chassisBlock.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);    

                
                limit.up = cur.x + AREA_LENGTH;
                limit.down = cur.x;

                // if (VirBan[0].x != 100 && JudgeArray(x,y,VirBan,45) == 1)
                // {
                //     printf("fanren\n");
                //     waitstart = 1;
                // }

                vector <Grid>().swap(boundPoint);        

                return;	
            }
					

        }
        int changeS = 0;
        static DIVB lastdivBound; 
        if (divBound != lastdivBound)
            changeS = 1;
        lastdivBound = divBound;
        
        FRIZY_LOG(LOG_DEBUG,"div.%d,sxzy:%d.%d.%d.%d,divWall.%d,changeS.%d"
        ,divBound,limit.up,limit.down,limit.left,limit.right,divWall,changeS);
       // int grate = 0;
        switch (divBound)
        {
          case UP:
          {
            if (IsWall() != 0)
            {
                //printf("%d.%d.%d.%d\n",walltime,gratingdis,lsddis,keepwalltime);
                if ((((CalGrating(sensor,cur,1) || justwind) && divWall == UP && ReMove(cur,divBound) == 0
                        && ((cur.forward > 270 || cur.forward < 45) || grate.grating.addAngle < cur.addAngle))
                      || (divWall == DOWN || divWall == RIGHT || divWall == IDLE)
                      || (divWall == LEFT && grate.grateTime > 20 && ReMove(cur,divBound) == 0))
                    && cur.x == limit.up)
                {
                    justwind = 0;
                    memset(&grate,0,sizeof(grate));
                    FRIZY_LOG(LOG_DEBUG,"road:qiezhixian1.%d",grate.grating.x);
                   
                    do{
                        StopWallFollow();
                        chassisBlock.GetSensor(&sensor);
                        usleep(10 * 1000);
                    }while (sensor.leftw != 0 || sensor.rightw != 0);

                    return;			
                }	
                else
                {   
                    FRIZY_LOG(LOG_DEBUG,"road:keepwall1");
                    //divWall = UP;
                    //CalGrating(sensor,cur,1);
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                    return;
                } 
            }                 
            else
            {
                if ((sensor.bump || sensor.obs || sensor.leftVir || sensor.rightVir
                     || _maze.GetMapState(cur.x,cur.y+2,1) == 4) && changeS == 0)
                {
                    memset(&grate,0,sizeof(grate));
                    conIndex = find(boundPoint.begin(), boundPoint.end(),cur)-boundPoint.begin();
                    FRIZY_LOG(LOG_DEBUG,"road:bianjiejinyanqiang1.%d",conIndex);
                    grate.grating.dx = cur.dx,grate.grating.dy = cur.dy,grate.grateTime = 0;
                    grate.grating.addAngle = cur.addAngle;
                    divWall = UP;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                    return;                 					
                }
                else                
                {
                    if (abs(cur.x - limit.up) >= sensor.size && limit.up != 1000)
                    {
                        FRIZY_LOG(LOG_DEBUG,"wulukezou1");
                        tmpAdd = cur.addAngle;
                        _aim.kind = backBound;
                        planBlock.PlanSearch(_aim,-1);
                        process = PLAN;
                        return;
                    }

                    FRIZY_LOG(LOG_DEBUG,"keepup1");	
                    aim.x = cur.x,aim.y = cur.y + 1;

                    if (cur.x == boundPoint[0].x && cur.y == boundPoint[0].y)
                        aim.forward = 270;
                    else
                        aim.forward = 2270;
                    motionBlock.WheelControl(sensor,cur,aim);
                    return;
                }	
            }
            break;
          }
          case DOWN:
          {
            if (IsWall() != 0)
            {
                //printf("%d.%d.%d.%d\n",walltime,gratingdis,lsddis,keepwalltime);
                if ((((CalGrating(sensor,cur,1) || justwind) && divWall == DOWN && ReMove(cur,divBound) == 0
                        && ((cur.forward > 90 && cur.forward < 270) || grate.grating.addAngle < cur.addAngle))
                    || (divWall == UP || divWall == LEFT || divWall == IDLE)
                    || (divWall == RIGHT && grate.grateTime > 20 && ReMove(cur,divBound) == 0))
                && cur.x == limit.down)
                {
                    justwind = 0;
                    memset(&grate,0,sizeof(grate));
                    FRIZY_LOG(LOG_DEBUG,"road:qiezhixian2.%d",grate.grating.x);
                    do{
                        StopWallFollow();
                        chassisBlock.GetSensor(&sensor);
                        usleep(10 * 1000);
                    }while (sensor.leftw != 0 || sensor.rightw != 0);
                    return;			
                }	
                else
                {   
                    FRIZY_LOG(LOG_DEBUG,"road:keepwall2");
                    //divWall = DOWN;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                    return;
                }
            }                 
            else
            {
				
                if ((sensor.bump || sensor.obs || sensor.leftVir || sensor.rightVir
                    || _maze.GetMapState(cur.x,cur.y-2,1) == 4) && changeS == 0)
                {
                    memset(&grate,0,sizeof(grate));
                    conIndex = find(boundPoint.begin(), boundPoint.end(),cur)-boundPoint.begin();
                    FRIZY_LOG(LOG_DEBUG,"road:bianjiejinyanqiang2.%d",conIndex);
                    grate.grating.dx = cur.dx,grate.grating.dy = cur.dy,grate.grateTime = 0;
                    grate.grating.addAngle = cur.addAngle;
                    divWall = DOWN;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                    return;                 					
                }
                else
                {
                    if (abs(cur.x - limit.down) >= sensor.size && limit.down != -1000)
                    {
                        FRIZY_LOG(LOG_DEBUG,"wulukezou2");
                        tmpAdd = cur.addAngle;
                        _aim.kind = backBound;
                        planBlock.PlanSearch(_aim,-1);
                        process = PLAN;
                        return;
                    }                   
                    FRIZY_LOG(LOG_DEBUG,"keepup2");	
                    aim.x = cur.x,aim.y = cur.y - 1;

                    if (cur.x == boundPoint[0].x && cur.y == boundPoint[0].y)
                        aim.forward = 90;
                    else
                        aim.forward = 2090;                    
                    motionBlock.WheelControl(sensor,cur,aim);
                    return;
                }	
            }
                     
            break;


            }
          case LEFT:
          {
            if (IsWall() != 0)
            {
                //printf("%d.%d.%d.%d\n",walltime,gratingdis,lsddis,keepwalltime);
                if ((((CalGrating(sensor,cur,2) || justwind) && divWall == LEFT && ReMove(cur,divBound) == 0
                       && ((cur.forward > 180 && cur.forward < 315) || grate.grating.addAngle < cur.addAngle)) 
                        || (divWall == RIGHT || divWall == UP || divWall == IDLE)
                        || (divWall == DOWN && grate.grateTime > 20 && ReMove(cur,divBound) == 0))
                    && cur.y == limit.left)
                {
                    justwind = 0;
                    memset(&grate,0,sizeof(grate));
                    FRIZY_LOG(LOG_DEBUG,"road:qiezhixian3.%d",grate.grating.x);
                    do{
                        StopWallFollow();
                        chassisBlock.GetSensor(&sensor);
                        usleep(10 * 1000);
                    }while (sensor.leftw != 0 || sensor.rightw != 0);		
                    return;			
                }	
                else
                {   
                    FRIZY_LOG(LOG_DEBUG,"road:keepwall3");
                                        
                    //divWall = LEFT;
                    //CalGrating(sensor,cur,1);
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                    return;
                }
            }                 
            else
            {
				
                if ((sensor.bump || sensor.obs || sensor.leftVir || sensor.rightVir
                    || _maze.GetMapState(cur.x-2,cur.y,1) == 4) && changeS == 0)
                {
                    memset(&grate,0,sizeof(grate));
                    conIndex = find(boundPoint.begin(), boundPoint.end(),cur)-boundPoint.begin();
                    FRIZY_LOG(LOG_DEBUG,"road:bianjiejinyanqiang3.%d",conIndex);
                    grate.grating.dx = cur.dx,grate.grating.dy = cur.dy,grate.grateTime = 0;
                    grate.grating.addAngle = cur.addAngle;
                    divWall = LEFT;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                    return;                 					
                }
                else
                {
                    if (abs(cur.y - limit.left) >= sensor.size && limit.left != 1000)
                    {
                        FRIZY_LOG(LOG_DEBUG,"wulukezou3");
                        tmpAdd = cur.addAngle;
                        _aim.kind = backBound;
                        planBlock.PlanSearch(_aim,-1);
                        process = PLAN;
                        return;
                    }                      
                    FRIZY_LOG(LOG_DEBUG,"keepup3");	
                    aim.x = cur.x - 1,aim.y = cur.y,aim.forward = 2180;
                    if (cur.x == boundPoint[0].x && cur.y == boundPoint[0].y)
                        aim.forward = 180;
                    else
                        aim.forward = 2180;                         
                    motionBlock.WheelControl(sensor,cur,aim);
                    return;
                }	
            }           
            break;
            }
          case RIGHT:
          {
            if (IsWall() != 0)
            {
                //printf("%d.%d.%d.%d\n",walltime,gratingdis,lsddis,keepwalltime);
                if ((((CalGrating(sensor,cur,2) || justwind) && divWall == RIGHT && ReMove(cur,divBound) == 0
                        && ((cur.forward > 0 && cur.forward < 135) || grate.grating.addAngle < cur.addAngle))
                        || (divWall == LEFT || divWall == DOWN || divWall == IDLE)
                        || (divWall == UP && grate.grateTime > 20 && ReMove(cur,divBound) == 0))
                    && cur.y == limit.right)
                {
                    justwind = 0;
                    memset(&grate,0,sizeof(grate));
                    FRIZY_LOG(LOG_DEBUG,"road:qiezhixian4.%d",grate.grating.x);
                    do{
                        StopWallFollow();
                        chassisBlock.GetSensor(&sensor);
                        usleep(10 * 1000);
                    }while (sensor.leftw != 0 || sensor.rightw != 0);	
                    return;			
                }	
                else
                {   
                    FRIZY_LOG(LOG_DEBUG,"road:keepwall4");
                                        
                    //divWall = RIGHT;
                    //CalGrating(sensor,cur,1);
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                    return;
                }
            }                 
            else
            {
                if ((sensor.bump || sensor.obs || sensor.leftVir || sensor.rightVir
                    || _maze.GetMapState(cur.x+2,cur.y,1) == 4) && changeS == 0)
                {
                    memset(&grate,0,sizeof(grate));
                    conIndex = find(boundPoint.begin(), boundPoint.end(),cur)-boundPoint.begin();
                    FRIZY_LOG(LOG_DEBUG,"road:bianjiejinyanqiang4.%d",conIndex);
                    grate.grating.dx = cur.dx,grate.grating.dy = cur.dy,grate.grateTime = 0;
                    grate.grating.addAngle = cur.addAngle;
                    divWall = RIGHT;
                    StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                    return;                 					
                }
                else
                {
                    if (abs(cur.y - limit.right) >= sensor.size && limit.right != -1000)
                    {
                        FRIZY_LOG(LOG_DEBUG,"wulukezou4");
                        tmpAdd = cur.addAngle;
                        _aim.kind = backBound;
                        planBlock.PlanSearch(_aim,-1);
                        process = PLAN;
                        return;
                    }                    
                    FRIZY_LOG(LOG_DEBUG,"keepup4");	
                    
                    aim.x = cur.x + 1,aim.y = cur.y;
                    if (cur.x == boundPoint[0].x && cur.y == boundPoint[0].y)
                        aim.forward = 0;
                    else
                        aim.forward = 2000;
                    motionBlock.WheelControl(sensor,cur,aim);
                    return;
                }	
            }       
            break;   
            }
          default:
            break;
        }

    }

}




