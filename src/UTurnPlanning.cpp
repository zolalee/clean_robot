/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-10 14:41:44
 * @Project      : UM_path_planning
 */

#include "navigation_algorithm/UTurnPlanning.h"
#include <math.h>
#include <iostream>     // std::cout
#include <algorithm>    // std::find
#include <vector>       // std::vector
int cur_x = 0;
int cur_y = 0;
extern useerobot::Maze _maze;
namespace useerobot
{
    extern int areaClean;
    extern PRO process;
    extern mapRange _map;
    RoadAim record_charge_aim;
    extern vector <Grid> boundPoint;
    extern vector <boundary> boundArr;
    extern int lasttake;
    bool cleanTaskOverIndex =false;
    static ROAD_KIND searchKind = idle;
    static vector <GIVEUP> giveUp;
    //static int archwallSign = 0;
    static ARCHWALL arch_wall;
    static Grid gaSengAim;
    vector <GIVEUP> obarr;

    UTurnPlanning::UTurnPlanning(/* args */)
    {
    }

    UTurnPlanning::~UTurnPlanning()
    {

    }
    static int sign1 = 5;
    void UTurnPlanning::init()
    {
        UTURN = SEARCH_WALL; 
        searchKind = idle;   
        ARCH_STATE = LEFT_0;
        keepwallTime = 0;
        aim.x = 1000;
        aim.y = 1000;
        aim.forward = 1000;
        _map = {-1000,1000,-1000,1000};    
        obarr.clear();    
    }
    
    
    //判断有没有可能是假点
    int8_t UTurnPlanning::Virob(int x,int y)
    {
	
        int8_t temps = 0;
        
        
        //shang
        if (_maze.GetMapState(x,y,2) == 2 && _maze.GetMapState(x,y,1) != 2
            && _maze.GetMapState(x-1,y,2) == 1 
            && _maze.GetMapState(x+1,y,2) == 0 && _maze.GetMapState(x+1,y,1) != 2)
            temps = 1;
        
        //xia
        if (_maze.GetMapState(x,y,2) == 2 && _maze.GetMapState(x,y,1) != 2
            && _maze.GetMapState(x+1,y,2) == 1 
            && _maze.GetMapState(x-1,y,2) == 0 && _maze.GetMapState(x-1,y,1) != 2)
            temps = 2;
        //zuo
        if (_maze.GetMapState(x,y,2) == 2 && _maze.GetMapState(x,y,1) != 2
            && _maze.GetMapState(x,y-1,2) == 1 
            && _maze.GetMapState(x,y+1,2) == 0 && _maze.GetMapState(x,y+1,1) != 2)
            temps = 3;						
        //	you
        if (_maze.GetMapState(x,y,2) == 2 && _maze.GetMapState(x,y,1) != 2
            && _maze.GetMapState(x,y+1,2) == 1 
            && _maze.GetMapState(x,y-1,2) == 0 && _maze.GetMapState(x,y-1,1) != 2)
            temps = 4;	

        if (temps != 0)
        {
            for (int i = 0;i < obarr.size();i++)
            {
                if (obarr[i].x == x && obarr[i].y == y)
                {
                    FRIZY_LOG(LOG_DEBUG,"gangan.%d.%d",x,y);
                    return 0;
                }
            }   
            //FRIZY_LOG(LOG_DEBUG,"temps.%d",temps);     
            return temps;
        }
        else
            return 0;
    }
    
    void UTurnPlanning::Kmeans(int x,int y,int kinds)
    {
        int8_t count = 0;
        FRIZY_LOG(LOG_DEBUG,"Kmeans.%d,%d.%d",x,y,kinds);

        switch (kinds)
        {
            case 1 :
            {
                for (int i = -50;i < 50;i++)
                {
                    if (Virob(x,y-i) == kinds)
                    {	
                        if (count >= 99)
                        {
                            printf("taida1\n");
                            break;
                        }
                        count ++;
                        FRIZY_LOG(LOG_DEBUG,"push.%d.%d",x,y-i);
                        obarr.push_back({x,y-i,kinds,0});
                    }
                    else if (count > 0)
                        break;
                }	
                if (count > 0)
                   obarr.push_back({1000}); 
                
                break;
            }
            case 2 :
            {
                for (int i = -50;i < 50;i++)
                {
                    if (Virob(x,y+i) == kinds)
                    {	
                        if (count >= 99)
                        {
                            printf("taida2\n");
                            break;
                        }
                        count ++;
                        FRIZY_LOG(LOG_DEBUG,"push.%d.%d",x,y+i);
                        obarr.push_back({x,y+i,kinds,0});
                    }
                    else if (count > 0)
                        break;								
                        
                }
                if (count > 0)
                   obarr.push_back({1000}); 
                
                break;                

            }
            case 3 :
            {

                for (int i = -50;i < 50;i++)
                {
                    if (Virob(x-i,y) == kinds)
                    {	
                        if (count >= 99)
                        {
                            printf("taida3\n");
                            break;
                        }
                        count ++;
                        FRIZY_LOG(LOG_DEBUG,"push.%d.%d",x-i,y);
                        obarr.push_back({x-i,y,kinds,0});
                    }
                    else if (count > 0)
                        break;								
                        
                }
                if (count > 0)
                   obarr.push_back({1000}); 
                
                break;    
            }
            case 4 :
            {
                for (int i = -50;i < 50;i++)
                {
                    if (Virob(x+i,y) == kinds)
                    {	
                        if (count >= 99)
                        {
                            printf("taida3\n");
                            break;
                        }
                        count ++;
                        FRIZY_LOG(LOG_DEBUG,"push.%d.%d",x+i,y);
                        obarr.push_back({x+i,y,kinds,0});
                    }
                    else if (count > 0)
                        break;								
                        
                }
                if (count > 0)
                   obarr.push_back({1000}); 
                
                break;  
            }  
       

       
        }

    }
    
    int UTurnPlanning::ClearOB(Grid cur)
    {
            
        //obarr.clear();
        FRIZY_LOG(LOG_DEBUG,"zuihouzuihou.%d.%d.%d.%d",archBound.up,archBound.down,archBound.left,archBound.right);

        for (int y = archBound.right+1; y < archBound.left; y++)
        {
            for (int x = archBound.down+1; x < archBound.up; x++)
            {
                int kinds = Virob(x,y);
            
                if (kinds != 0)
                {
                    Kmeans(x,y,kinds);
                }
            }
        }

        for (int i = 0;i < obarr.size();i++)
        {

            FRIZY_LOG(LOG_DEBUG,"gangan2.%d.%d.%d.%d",obarr[i].x,obarr[i].y,obarr[i].count,obarr[i].close);
            
        }   
        FRIZY_LOG(LOG_DEBUG,"clear1.%d",obarr.size());
        //obarr.clear();
        //obarr这一数组提取出目标点,1000下一个索引
        //选取ASTAR距离最近的点
        //FRIZY_LOG(LOG_DEBUG,"clear2.%d",obarr.size());
        
        int dis = 1000;
        int num = obarr.size() - 2;
        RobotType sss;

        for (int i = 0;i < num;i++)
        {
            Grid temp = {obarr[i+2].x,obarr[i+2].y};

            if ((obarr[i].x == 1000 || i == 0)
                && temp.x != 1000 && JudgeGiveup(temp,searchKind) == 0 
                && obarr[i+2].close == 0
                && obarr[i+1].x != 1000 && obarr[i+2].x != 1000)
            {
                vector<pair<float, float>> astarL;
                if (searchKind == searchBound || searchKind == searchUnclean)
                    astarL = astarPlan.astarLength(cur.x,cur.y,obarr[i+2].x,obarr[i+2].y,sss,0,1);
                else
                    astarL = astarPlan.astarLength(cur.x,cur.y,obarr[i+2].x,obarr[i+2].y,sss,0,0);   

                int tempdis = astarL.size();
                if (tempdis == 0)
                    tempdis =  500 + abs(cur.x - obarr[i+2].x) + abs(cur.y - obarr[i+2].y);                 
                FRIZY_LOG(LOG_DEBUG,"obaa.%d,%d,%d",obarr[i+2].x,obarr[i+2].y,tempdis);
                
                if (dis > tempdis)
                {
                    _aim.x = obarr[i+2].x;
                    _aim.y = obarr[i+2].y;
                    dis = tempdis;
                    lasttake = obarr[i+2].count;
                }
            }
        }

        //不存在值得尝试的点
        if (dis == 1000)
        {
            FRIZY_LOG(LOG_DEBUG,"no vir");
            //sssss5 = 0;
            return 0;
        }
        else
        {
            
            FRIZY_LOG(LOG_DEBUG,"333.a == %d,b == %d,dis == %d",_aim.x,_aim.y,dis);

            return 1;			
        }
    }

    void UTurnPlanning::PlanSearch(RoadAim aim,int flag)
    {
        if (flag == 2)
        {
            searchKind = archwallSign;
            gaSengAim.x  = aim.x;
            gaSengAim.y  = aim.y;
            return;
        }

        searchKind = aim.kind;

        if (flag == -1)
            return;

        //flag >= 0
        for (int i = 0; i < giveUp.size();i++)
        {
            if (giveUp[i].x == aim.x && giveUp[i].y == aim.y)
            {
                if (flag == 1)
                {
                    FRIZY_LOG(LOG_DEBUG,"_jia5");
                    giveUp[i].count = 5; 
                }
                   
                else{
                    FRIZY_LOG(LOG_DEBUG,"_jia1");
                    giveUp[i].count ++;
                }  
                
                return;
            }    
        }

        GIVEUP temp;
        temp.x = aim.x;
        temp.y = aim.y;
        temp.count = 0;

        if (flag == 1)
            temp.count = 5; 
        else  
            temp.count = 1;

        FRIZY_LOG(LOG_DEBUG,"giveup.%d.%d.%d",temp.x,temp.y,temp.count);
        
        giveUp.push_back(temp);  
    }
    
    int UTurnPlanning::CheckFor(Grid cur,DIVB forward)
    {
        int temps = 0;
        if (forward == UP)
        {
            for (int i = 1;i <= AREA_LENGTH;i++)
            {
                if (_maze.GetMapState(cur.x+i,cur.y,2) != 0)
                {
                    temps = 1;
                    break;
                }   
                    
            }
        }
        if (forward == DOWN)
        {
            for (int i = 1;i <= AREA_LENGTH;i++)
            {
                if (_maze.GetMapState(cur.x-i,cur.y,2) != 0)
                {
                    temps = 1;
                    break;
                }   
            }
        }        
        if (forward == LEFT)
        {
            for (int i = 1;i <= AREA_LENGTH;i++)
            {
                if (_maze.GetMapState(cur.x,cur.y+i,2) != 0)
                {
                    temps = 1;
                    break;
                }   
            }
        }  
        if (forward == RIGHT)
        {
            for (int i = 1;i <= AREA_LENGTH;i++)
            {
                if (_maze.GetMapState(cur.x,cur.y-i,2) != 0)
                {
                    temps = 1;
                    break;

                }   
            }
        } 
        if (temps == 0)
            return 0;
        else
            return 1;
    }    

    int UTurnPlanning::JudgeGiveup(Grid cur,ROAD_KIND kind)
    {

        if (kind == searchUnclean || kind == searchBound){
            //小范围区域丢弃
            CloseArr.clear();

            int tmpS = 0;
            if (kind == searchUnclean){
                if (cur.x + 1 == archBound.up || cur.x - 1 == archBound.down
                    || cur.y + 1 == archBound.left || cur.y - 1 == archBound.right)
                    tmpS = 10;
                else
                    tmpS = Dsearch(cur.x,cur.y);

            }
                
            else{

                if (_maze.GetMapState(cur.x,cur.y,1) == 2){

                    FRIZY_LOG(LOG_DEBUG,"shiqiang.%d.%d",cur.x,cur.y);
                    return 1;
                }

                for (int x=cur.x-1; x<=cur.x+1; ++x)
                   for (int y=cur.y-1; y<=cur.y+1; ++y)
                        if (_maze.GetMapState(x,y,2) == 0 && abs(x-cur.x) + abs(y-cur.y) == 1){
                            tmpS = Dsearch(x,y);
                            FRIZY_LOG(LOG_DEBUG,"tmps.%d.%d.%d",tmpS,x,y);
                            break;
                        }
            }

            
            int threshold = kind == searchBound ? 8 : 4;

            if (tmpS < threshold){
                FRIZY_LOG(LOG_DEBUG,"diuqi.%d.%d.%d",cur.x,cur.y,tmpS);
                return 1;   
            }
        }

        // if (kind == searchBound)
        // {
        //     for (int x = cur.x - 1;x <= cur.x + 1;x++)
        //       for (int y = cur.y - 1;y <= cur.y + 1;y++)  
        //         if (_maze.GetMapState(x,y,1) == 2 || _maze.GetMapState(x,y,2) == 2)
        //         {
        //             if (_maze.GetMapState(x,y,1) == 2)
        //                 FRIZY_LOG(LOG_DEBUG,"shiqiang.%d.%d",x,y);
        //             return 1;
        //         }    
        // }

        if ((cur.x + 1 == archBound.up && _maze.GetMapState(cur.x+1,cur.y,2) == 1 && _maze.GetMapState(cur.x-1,cur.y,2) == 0)
        || (cur.x - 1 == archBound.down && _maze.GetMapState(cur.x-1,cur.y,2) == 1 && _maze.GetMapState(cur.x+1,cur.y,2) == 0)
        || (cur.y + 1 == archBound.left && _maze.GetMapState(cur.x,cur.y+1,2) == 1 && _maze.GetMapState(cur.x,cur.y-1,2) == 0)           
        || (cur.y - 1 == archBound.right && _maze.GetMapState(cur.x,cur.y-1,2) == 1 && _maze.GetMapState(cur.x,cur.y+1,2) == 0))
        {
            if (kind == searchInside)
            {
                if ((cur.x + 1 == archBound.up && CheckFor(cur,DOWN) == 0)
                    || (cur.x - 1 == archBound.down && CheckFor(cur,UP) == 0)
                    || (cur.y + 1 == archBound.left && CheckFor(cur,RIGHT) == 0)
                    || (cur.y - 1 == archBound.right && CheckFor(cur,LEFT) == 0))
                {
                    FRIZY_LOG(LOG_DEBUG,"guolvlv.%d.%d",cur.x,cur.y);
                    return 1;
                }
            }
            if (kind == searchUnclean)
                return 1;
        }

 
        for (auto temp : giveUp)
        { 
            if (kind == searchBound)
            {
                for (int i = -3;i < 3;i++)
                {
                    if (temp.y + i > archBound.left || temp.y + i < archBound.right)
                        continue;
                         
                    if (temp.x == cur.x && temp.y + i == cur.y && temp.count > 4
                        && (temp.x == archBound.up || temp.x == archBound.down) 
                        && _maze.GetMapState(cur.x,cur.y,2) == 1)
                    {
                        return 1;
                    }																
                }
                for (int i = -3;i < 3;i++)
                {
                    if (temp.x + i > archBound.up || temp.x + i < archBound.down)
                        continue;
                         
                    if (temp.y == cur.y && temp.x + i == cur.x && temp.count > 4
                        && (temp.y == archBound.left || temp.y == archBound.right) 
                        && _maze.GetMapState(cur.x,cur.y,2) == 1)
                    {
                        return 1;
                    }																
                }

            }
            if (kind == searchUnclean)
            {
                Grid aim;
                if (temp.count > 4)
                {
                    aim.x = temp.x,aim.y = temp.y;

                    if (SameLine(cur,aim,0) == 1){
                        FRIZY_LOG(LOG_DEBUG,"fangqile.%d.%d,%d.%d",cur.x,cur.y,aim.x,aim.y);
                        return 1;
                    }
                        
                }
            }
        }
        return 0;
    }

    int UTurnPlanning::SameLine(Grid cur,Grid aim,int state)
    {
        //printf("infor.%d.%d.%d.%d.%d\n",cur.x,cur.y,aim.x,aim.y,state);
        if (cur.y == aim.y)
        {
            if (aim.x > cur.x)
            {
                for (int i = 0;cur.x + i <= aim.x;i++)
                {
                    if (_maze.GetMapState(cur.x+i,cur.y,2) != state)
                    {

                        //printf("return 01:%d.%d.%d\n",cur.x+i,cur.y,_maze.GetMapState(cur.x+i,cur.y,2));
                        return 0;
                    }
                }
            }
            else
            {
                for (int i = 0;aim.x + i <= cur.x;i++)
                {
                    if (_maze.GetMapState(aim.x+i,aim.y,2) != state)
                    {
                        //printf("return 02\n");
                        return 0;
                    }
                    
                }						
            }
            //printf("return1\n");
            return 1;	

        }
        if (cur.x == aim.x)
        {
            if (aim.y > cur.y)
            {
                for (int i = 0;cur.y + i <= aim.y;i++)
                {
                    if (_maze.GetMapState(cur.x,cur.y+i,2) != state)
                    {
                        return 0;
                    }
                }
            }
            else
            {
                for (int i = 0;aim.y + i <= cur.y;i++)
                {
                    if (_maze.GetMapState(aim.x,aim.y+i,2) != state)
                    {
                        return 0;
                    }
                }						
            }	
            //printf("return1\n");
            return 1;
        }
        return 0;

    }

    bool UTurnPlanning::CleanRecharge(){

        chassisPlan.getPlanningInfo(&charger_planning_info);
        if(charger_planning_info.charger_front_position.x == 0 && charger_planning_info.charger_front_position.y == 0)
        {
            
            if(record_charge_aim.x!=0&&record_charge_aim.y!=0)
            {
                FRIZY_LOG(LOG_INFO, "THE RECORD RECHARGE IS %d ,%d",record_charge_aim.x,record_charge_aim.y);
                _aim.x = record_charge_aim.x;
                _aim.y = record_charge_aim.y;
            }
            else
            {
                FRIZY_LOG(LOG_INFO, "NO SLAM RECHARGE INFO AND NO RECORD RECHARGE INFO");
                _aim.x = 0;
                _aim.y = 0;
            }
            
        }
        else
        {
            _aim.x =round_doubel((charger_planning_info.charger_front_position.x * 100)/15);
            _aim.y =round_doubel((charger_planning_info.charger_front_position.y * 100)/15);                        
        }          
            
        _aim.kind = recharge;
        FRIZY_LOG(LOG_DEBUG,"recharge.%d.%d\n",_aim.x,_aim.y);
        roadPlan.SetroadAim(_aim);
        process = ROAD;
        UTURN = ARCH,ARCH_STATE = NO_ARCH;
        searchKind = idle;
        cleanTaskOverIndex = true;
        return true;   
    }

    int UTurnPlanning::Dsearch(int x,int y){

        Grid tmp = {x,y};
        if(CloseArr.size() > 10 || _maze.GetMapState(x,y,2) != 0
             || find(CloseArr.begin(), CloseArr.end(),tmp) != CloseArr.end())
            return 0;

        CloseArr.push_back(tmp);
        int num = 1;
        num += Dsearch(x+1,y);
        num += Dsearch(x,y+1);
        num += Dsearch(x-1,y);
        num += Dsearch(x,y-1);
        return num;        
    }

    
    //搜索漏扫区 获取目标点
    void UTurnPlanning::SearchUnlean(Sensor sensor,Grid cur)
    {
        if (IsWall() != 0)
        {
            do{
                FRIZY_LOG(LOG_DEBUG,"road stop1");
                StopWallFollow();
                chassisPlan.GetSensor(&sensor);
                usleep(10 * 1000);
            }while (sensor.leftw != 0 || sensor.rightw != 0);
        }
        else{
            do{
                FRIZY_LOG(LOG_DEBUG,"road stop2");
                controlw.WheelControl(sensor,cur,cur);
                chassisPlan.GetSensor(&sensor);
                usleep(10 * 1000);
            }while (sensor.leftw != 0 || sensor.rightw != 0);
        }
        //searchKind = searchUnclean;

        if (archBound.left == 1000)
        {
            archBound.left = 200;
            archBound.right = -200;
        }
        if (archBound.up == 1000)
        {
            archBound.up = 200;
            archBound.down = -200;
        }

        if (searchKind == backBound)
        {
            FRIZY_LOG(LOG_DEBUG,"BackBound");


            vector <Grid> array;

            Grid temp;
            //开始搜集需要返回的点
            for (int x = archBound.down;x <= archBound.up;x ++)
            {
                if (_maze.GetMapState(x,archBound.left,2) == 1
                     && _maze.GetMapState(x+1,archBound.left,2) != 2
                     && _maze.GetMapState(x-1,archBound.left,2) != 2)
                {
                    temp.x = x;
                    temp.y = archBound.left;
                    array.push_back(temp);
                }
                if (_maze.GetMapState(x,archBound.right,2) == 1
                    && _maze.GetMapState(x+1,archBound.right,2) != 2
                    && _maze.GetMapState(x-1,archBound.right,2) != 2)
                {
                    temp.x = x;
                    temp.y = archBound.right;
                    array.push_back(temp);					
                }
            }
            
            for (int y = archBound.right;y <= archBound.left;y ++)
            {
                if (_maze.GetMapState(archBound.up,y,2) == 1
                    && _maze.GetMapState(archBound.up,y+1,2) != 2
                    && _maze.GetMapState(archBound.up,y-1,2) != 2)
                {
                    temp.x = archBound.up;
                    temp.y = y;
                    array.push_back(temp);
                }
                if (_maze.GetMapState(archBound.down,y,2) == 1
                    && _maze.GetMapState(archBound.down,y+1,2) != 2
                    && _maze.GetMapState(archBound.down,y-1,2) != 2)
                {
                    temp.x = archBound.down;
                    temp.y = y;
                    array.push_back(temp);					
                }
            }	
            
            if (array.size() == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"fk3");
                searchKind = searchBound;
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

            FRIZY_LOG(LOG_DEBUG,"backBound.%d.%d.%d",_aim.x,_aim.y,_maze.GetMapState(_aim.x,_aim.y,2));

            roadPlan.SetroadAim(_aim);

            process = ROAD;        
            
            return;
        }

        if (searchKind == searchUnclean || searchKind == searchInside)
        {
            FRIZY_LOG(LOG_DEBUG,"start to to");
            
            if (searchKind == searchInside)
               FRIZY_LOG(LOG_DEBUG,"searchInside"); 
            //test
                // int x ,y;
                // for(int i = -50 ;i<50;i++)
                // {
                //     for(int j = -50 ;j<50;j++)
                    
                //     {
                //         int tem_n = _maze.GetMapState(i,j,1);
                //         printf("@300=%04d,%04d,00%d\n",i,j,tem_n);
                    
                //     }
                // }

            vector <Grid> uncleanPoint;

            for (int y = archBound.right+1;y < archBound.left;y++)
            {
                for (int x = archBound.down+1;x < archBound.up;x++)
                {
                    if ((_maze.GetMapState(x,y,2) == 0 && _maze.VirBound(x,y) == 0)
                        && ((_maze.GetMapState(x,y+1,2) == 1 && _maze.GetMapState(x+1,y+1,2) == 1 && _maze.GetMapState(x-1,y+1,2) == 1)
                            || (_maze.GetMapState(x,y-1,2) == 1 && _maze.GetMapState(x+1,y-1,2) == 1 && _maze.GetMapState(x-1,y-1,2) == 1)
                            || ((_maze.GetMapState(x+1,y,2) == 1 || _maze.GetMapState(x-1,y,2) == 1) 
                                && _maze.GetMapState(x,y+1,2) != 2 && _maze.GetMapState(x,y-1,2) != 2)))
                    {
                        Grid temps;
                        temps.x = x,temps.y = y,temps.forward = 0; 

                        if (JudgeGiveup(temps,searchKind) == 1)
                            continue;

                        FRIZY_LOG(LOG_DEBUG,"temp.%d.%d",temps.x,temps.y);
             
                        uncleanPoint.push_back(temps);
                    }
                }
            }

            FRIZY_LOG(LOG_DEBUG,"the length == %d",uncleanPoint.size());

            if (uncleanPoint.size() == 0
                || (uncleanPoint.size() < 3 && searchKind == searchInside))
            {

                FRIZY_LOG(LOG_DEBUG,"length == 0");

                if (areaClean){
                    areaClean = 0;
                    FRIZY_LOG(LOG_DEBUG,"huaquhuichong");
                    CleanRecharge();
                    return;
                }

                if (ClearOB(cur) == 0)
                    searchKind = searchBound;
                else
                {
                    searchKind = searchBound;
                    // _aim.kind = searchLast;
           
                    // roadPlan.SetroadAim(_aim);

                    // process = ROAD;

                    // UTURN = ARCH,ARCH_STATE = NO_ARCH;
                    // searchKind = idle;
                             
                }
               
                return;

            }

            int dis = 1000;
            
            for(int i = 0;i < uncleanPoint.size();i++)
            { 

                // auto it = find(PassPoint.begin(),PassPoint.end(),uncleanPoint[i]);

                // if (it != PassPoint.end())
                //     continue;
            
                if (uncleanPoint[i].forward == 1000)
                    continue;
              
                
                RobotType sss;
              
                
                Grid tempP;
                tempP.x = uncleanPoint[i].x + UporDown(cur,uncleanPoint[i],1).redis;   
                tempP.y = uncleanPoint[i].y;
                
                vector<pair<float, float>> astarL;
 
                if (searchKind == searchBound || searchKind == searchUnclean)
                    astarL = astarPlan.astarLength(cur.x,cur.y,tempP.x,tempP.y,sss,0,1);
                else
                    astarL = astarPlan.astarLength(cur.x,cur.y,tempP.x,tempP.y,sss,0,0);   

               
                int tempdis = astarL.size();

                if (tempdis == 0)
                    tempdis =  500 + abs(cur.x - tempP.x) + abs(cur.y - tempP.y);

                FRIZY_LOG(LOG_DEBUG,"111.%d,%d,%d",uncleanPoint[i].x,uncleanPoint[i].y,tempdis);
                FRIZY_LOG(LOG_DEBUG,"_111.%d,%d,%d",tempP.x,tempP.y,tempdis);
                
                if (dis >= tempdis)
                {
                    aim.x = tempP.x;
                    aim.y = tempP.y;
                    dis = tempdis;
                }
                
                
                //SameLine(array[i].x,array[i].y,array[j].x,array[j].y,1)
                for(int j = 0;j < uncleanPoint.size();j++)
                {
                    // if ((uncleanPoint[i].x == uncleanPoint[j].x)
                    //     || (uncleanPoint[i].y == uncleanPoint[j].y))  
                    //printf(".%d.%d\n",uncleanPoint[j].x,uncleanPoint[j].y);

                    if (SameLine(uncleanPoint[i],uncleanPoint[j],0) == 1)
                    {
                        uncleanPoint[j].forward = 1000;
                        //PassPoint.push_bacsearchboundk(uncleanPoint[j]);
                    }
                }
            }

            //为了保证在正确的一侧
            //aim.x = aim.x + UporDown(cur,asearchboundim,1).redis;

            FRIZY_LOG(LOG_DEBUG,"_222.%d.%d.%d\n",aim.x,aim.y,dis);

            _aim.kind = searchKind;
            _aim.x = aim.x,_aim.y = aim.y;

           // RoadPlanning temp;
            roadPlan.SetroadAim(_aim);

            process = ROAD;

			cur_x = cur.x;
			cur_y = cur.y;

            printf("cur_x,cur_y:%d.%d.%d\n",cur_x ,cur_y);
            UTURN = ARCH,ARCH_STATE = NO_ARCH;
            searchKind = idle;
            return;
        }

        if (searchKind == searchBound)
        {
            FRIZY_LOG(LOG_DEBUG,"searchbound");
            for (auto temp : archBoundArr)
            {
                FRIZY_LOG(LOG_DEBUG,"bound!.%d.%d.%d.%d,%d",temp.up,temp.down,temp.left,temp.right,archBoundArr.size());
            }

            _map = {-1000,1000,-1000,1000};
            vector <Grid> uncleanPoint;
            Grid temps;
            for (auto temp : archBoundArr)
            {
                if (temp.left == 1000)
                    temp.left = 200,temp.right = -200;
                if (temp.up == 1000)
                    temp.up = 200,temp.down = -200;    
                                    
                for (int x = temp.down+1;x < temp.up;x++)
                {
                    if (_maze.GetMapState(x,temp.left,2) == 1 && _maze.VirBound(x,temp.left) == 0
                        && _maze.GetMapState(x,temp.left+1,2) == 0
                        //&& _maze.GetMapState(x+1,temp.left,2) == 1 && _maze.GetMapState(x-1,temp.left,2) == 1
                        && _maze.GetMapState(x,temp.left-1,2) == 1
                        && (_maze.GetMapState(x+1,temp.left+1,2) == 0 || _maze.GetMapState(x-1,temp.left+1,2) == 0))
                    {
                        temps.x = x;
                        temps.y = temp.left;
                        FRIZY_LOG(LOG_DEBUG,"tmp1.%d.%d",temps.x,temps.y);
                        if (JudgeGiveup(temps,searchKind) == 1)
                            continue;
                        uncleanPoint.push_back(temps);
                    }
                    if (_maze.GetMapState(x,temp.right,2) == 1 && _maze.VirBound(x,temp.right) == 0
                         && _maze.GetMapState(x,temp.right-1,2) == 0
                        //&& _maze.GetMapState(x+1,temp.right,2) == 1 && _maze.GetMapState(x-1,temp.right,2) == 1
                        && _maze.GetMapState(x,temp.right+1,2) == 1
                        && (_maze.GetMapState(x+1,temp.right-1,2) == 0 || _maze.GetMapState(x-1,temp.right-1,2) == 0))
                    {
                        temps.x = x;
                        temps.y = temp.right;
                        FRIZY_LOG(LOG_DEBUG,"tmp1.%d.%d",temps.x,temps.y);
                        if (JudgeGiveup(temps,searchKind) == 1)
                            continue;
                        uncleanPoint.push_back(temps);
                    }
                }
                for (int y = temp.right+1;y < temp.left;y++)
                {
                    if (_maze.GetMapState(temp.down,y,2) == 1 && _maze.VirBound(temp.down,y) == 0
                         && _maze.GetMapState(temp.down-1,y,2) == 0
                        //&& _maze.GetMapState(temp.down,y+1,2) == 1 && _maze.GetMapState(temp.down,y-1,2) == 1
                        && _maze.GetMapState(temp.down+1,y,2) == 1
                        && (_maze.GetMapState(temp.down-1,y+1,2) == 0 || _maze.GetMapState(temp.down-1,y-1,2) == 0))
                    {
                        temps.x = temp.down;
                        temps.y = y;
                        FRIZY_LOG(LOG_DEBUG,"tmp1.%d.%d",temps.x,temps.y);
                        if (JudgeGiveup(temps,searchKind) == 1)
                            continue;                       
                        uncleanPoint.push_back(temps);
                    }
                    if (_maze.GetMapState(temp.up,y,2) == 1 && _maze.VirBound(temp.up,y) == 0
                         && _maze.GetMapState(temp.up+1,y,2) == 0
                        //&& _maze.GetMapState(temp.up,y+1,2) == 1 && _maze.GetMapState(temp.up,y-1,2) == 1
                        && _maze.GetMapState(temp.up-1,y,2) == 1
                        && (_maze.GetMapState(temp.up+1,y+1,2) == 0 || _maze.GetMapState(temp.up+1,y-1,2) == 0))
                    {
                        temps.x = temp.up;
                        temps.y = y;
                        FRIZY_LOG(LOG_DEBUG,"tmp1.%d.%d",temps.x,temps.y);
                        if (JudgeGiveup(temps,searchKind) == 1)
                            continue;                        
                        uncleanPoint.push_back(temps);
                    }                    
                }
            }

            if (uncleanPoint.size() == 0)
            {
                
                for (int x = -95;x <= 95;x++)
                {
                    for (int y = -95;y <= 95;y++)
                    {
                        if (_maze.GetMapState(x,y,1) == 2)
                        {
                            printf("@300=%04d,%04d,00%d\n",x,y,2);
                        }
                    }	
                }  
                sleep(1);

                CleanRecharge();
                    
                return;                 
            }
            else
            {
    
                int16_t dis = 1000;

                for (int i = 0;i < uncleanPoint.size();i++)
                {
                    if (dis > abs(cur.x - uncleanPoint[i].x) +  abs(cur.y - uncleanPoint[i].y))
                    {
                        dis = abs(cur.x - uncleanPoint[i].x) +  abs(cur.y - uncleanPoint[i].y);
                        _aim.x = uncleanPoint[i].x;
                        _aim.y = uncleanPoint[i].y;
                    }
                }
                _aim.kind = searchBound;
                printf("tempX == %d,tempY == %d\n",_aim.x,_aim.y);
                     
            }


            //RoadPlanning roadclass;
            roadPlan.SetroadAim(_aim);
            process = ROAD;
            UTURN = ARCH,ARCH_STATE = NO_ARCH;
            searchKind = idle;
            return;

        }
    }


    void UTurnPlanning::ArchWall(Sensor sensor,Grid cur)
    {
        searchKind = idle;
        switch (arch_wall)
        {
        case RIGHTWALL_0:

            if (cur.y <= aim.y + sensor.size && cur.y > aim.y
                
                && cur.x + 1 < archBound.up
                && (_maze.GetMapState(cur.x+1,cur.y-1,2) == 0 || _maze.GetMapState(cur.x+2,cur.y-1,2) == 0))
            {
                printf("walling1\n");
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                return;
            }
            else if (cur.y == aim.y)
            {
                do{
                    printf("out1\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                
                
                ARCH_STATE = RIGHT_0;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out5\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  
                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                return;					
            }  

        case RIGHTWALL_180:
            if (cur.y <= aim.y + sensor.size && cur.y > aim.y
                && cur.x - 1 > archBound.down
                && (_maze.GetMapState(cur.x-1,cur.y-1,2) == 0 || _maze.GetMapState(cur.x-2,cur.y-1,2) == 0))
            {
                printf("walling2\n");
                StartWallFollow(RandomWallFollow, LEFTAW, QUICK);
                return;
            }
            else if (cur.y == aim.y)
            {
                do{
                    printf("out2\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);    

                ARCH_STATE = RIGHT_180;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out6\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  
                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                return;					
            }   

        case LEFTWALL_0:
            if (cur.y >= aim.y - sensor.size && cur.y < aim.y
                && cur.x + 1 < archBound.up
                && (_maze.GetMapState(cur.x+1,cur.y+1,2) == 0 || _maze.GetMapState(cur.x+2,cur.y+1,2) == 0))
            {
                printf("walling3\n");
                StartWallFollow(RandomWallFollow, LEFTAW, QUICK);
                return;
            }
            else if (cur.y == aim.y)
            {
                do{
                    printf("out3\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                
                
                ARCH_STATE = LEFT_0;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out7\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;
                return;					
            }   
            case LEFTWALL_180:
            if (cur.y >= aim.y - sensor.size && cur.y < aim.y
                && cur.x - 1 > archBound.down
                && (_maze.GetMapState(cur.x-1,cur.y+1,2) == 0 || _maze.GetMapState(cur.x-2,cur.y+1,2) == 0))
            {
                printf("walling4\n");
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
                return;
            }
            else if (cur.y == aim.y)
            {
                do{
                    printf("out4\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);                
                
                ARCH_STATE = LEFT_180;
                UTURN = ARCH;
                return;					
            }
            else
            {
                do{
                    printf("out8\n");
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                 }while (sensor.leftw != 0 || sensor.rightw != 0);  
                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;					
            }                          
        default:
            break;
        }        

    }

    
    Rearch UTurnPlanning::UporDown(Grid cur,Grid aim,int othersign)
    {
		int8_t tempup = 0,tempdown = 0;
        Rearch temp;
        
		for (int i = 1;i < 50;i++)
		{
            if (_maze.GetMapState(aim.x+i,aim.y,2) == 0 
                && (_maze.GetMapState(aim.x+i,aim.y+1,2) == 1 || _maze.GetMapState(aim.x+i,aim.y-1,2) == 1))

                    tempup ++;
            else
                break;
        }
        for (int i = 1;i < 50;i++) 
        {   
		   if (_maze.GetMapState(aim.x-i,aim.y,2) == 0 
                && (_maze.GetMapState(aim.x-i,aim.y+1,2) == 1 || _maze.GetMapState(aim.x-i,aim.y-1,2) == 1))
                    tempdown ++;
            else
                break;
		}

        if (othersign == 1)
        {
            int upX = aim.x + tempup;
            int downX = aim.x - tempdown;
            
            if (abs(cur.x - upX) > abs(cur.x - downX))
            {   
                FRIZY_LOG(LOG_DEBUG,"xiafang");
                tempdown = -1 * tempdown;
                temp.redis = tempdown + 1;
            }
            else if (abs(cur.x - upX) == abs(cur.x - downX))
            {
                FRIZY_LOG(LOG_DEBUG,"zhongfang");
                temp.redis = 0;
            }            
            else
            {
                FRIZY_LOG(LOG_DEBUG,"shangfang");
                temp.redis = tempup - 1;
            }
            return temp;
        }

		FRIZY_LOG(LOG_DEBUG,"tempup.%d,tempdown.%d",tempup,tempdown);
		
        if (tempup == 0 && tempdown == 0)
        {
            if (_maze.GetMapState(aim.x-1,aim.y,2) == 0 && _maze.GetMapState(aim.x+1,aim.y,2) != 0)
            {
                if (_maze.GetMapState(aim.x,aim.y+1,2) == 0 && _maze.GetMapState(aim.x,aim.y-1,2) != 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"rrr11");
                    temp.refor = LEFT_180;
                }
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"rrr1");
                    temp.refor = RIGHT_180;
                }
            }
            else
            {
                if (_maze.GetMapState(aim.x,aim.y+1,2) == 0 && _maze.GetMapState(aim.x,aim.y-1,2) != 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"rrr22");
                    temp.refor = LEFT_0;
                }
                else
                {                
                    FRIZY_LOG(LOG_DEBUG,"rrr2");
                    temp.refor = RIGHT_0;
                }
            }   
            temp.redis = 0;
            return temp;	        
        }
		else if (tempup < tempdown)
		{
            if (_maze.GetMapState(aim.x,aim.y+1,2) == 1 && _maze.GetMapState(aim.x,aim.y-1,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra1");
                temp.refor = RIGHT_180;
            }
            else if (_maze.GetMapState(aim.x,aim.y-1,2) == 1 && _maze.GetMapState(aim.x,aim.y+1,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra2");
                temp.refor =  LEFT_180;
            }
            else if (cur.y <= archBound.right+1)
            {
                FRIZY_LOG(LOG_INFO,"map warn2");
                temp.refor = LEFT_180;
            }
            else
            {
                FRIZY_LOG(LOG_INFO,"map warn1");
                temp.refor = RIGHT_180;
            } 

            temp.redis = tempup - 1;
            FRIZY_LOG(LOG_DEBUG,"1o.%d",temp.redis);
            

            return temp;	
		}
		else
		{
            if (_maze.GetMapState(aim.x,aim.y+1,2) == 1 && _maze.GetMapState(aim.x,aim.y-1,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra3");
                temp.refor = RIGHT_0;
            }
            else if (_maze.GetMapState(aim.x,aim.y-1,2) == 1 && _maze.GetMapState(aim.x,aim.y+1,2) == 0)
            {
                FRIZY_LOG(LOG_DEBUG,"ra4");
                temp.refor = LEFT_0;    
            }
            else if (cur.y <= archBound.right+1)
            {
                FRIZY_LOG(LOG_INFO,"map warn4");
                temp.refor = LEFT_0;
            }
            else
            {
                FRIZY_LOG(LOG_INFO,"map warn3");
                temp.refor = RIGHT_0;
            }
            
            
            tempdown = -1 * tempdown;
            temp.redis = tempdown + 1;
            printf("2o.%d\n",temp.redis);

   
            return temp;            
		}
    }


    bool UTurnPlanning::StopArch(Sensor sensor,Grid cur)
    {
        FRIZY_LOG(LOG_DEBUG,"start arch.%d.%d",_maze.GetMapState(cur.x+1,cur.y,2),_maze.GetMapState(cur.x-1,cur.y,2));
        
        //获取地图的最大最小值
        if (cur.x > _map.xmax) _map.xmax = cur.x;
        if (cur.x < _map.xmin) _map.xmin = cur.x;
        if (cur.y > _map.ymax) _map.ymax = cur.y;
        if (cur.y < _map.ymin) _map.ymin = cur.y;

        FRIZY_LOG(LOG_DEBUG,"MAX.%d.%d.%d.%d",_map.xmax,_map.xmin,_map.ymax,_map.ymin);

        if (archBound.up == 1000)
        {
            if (cur.x - _map.xmin >= AREA_LENGTH 
                && (ARCH_STATE == LEFT_0 || ARCH_STATE == RIGHT_0 || UTURN == BUMP_WALL))
            {
                archBound.up = cur.x;
                archBound.down = _map.xmin;
                FRIZY_LOG(LOG_DEBUG,"gong1.%d.%d",archBound.up,archBound.down);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);                
            }
            if (_map.xmax - cur.x >= AREA_LENGTH 
                && (ARCH_STATE == LEFT_180 || ARCH_STATE == RIGHT_180 || UTURN == BUMP_WALL))
            {
                archBound.up = _map.xmax;
                archBound.down = cur.x;							
                FRIZY_LOG(LOG_DEBUG,"gong2.%d.%d",archBound.up,archBound.down);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);			
            }
        }

        if (archBound.left == 1000)
        {
            if (cur.y - _map.ymin >= AREA_LENGTH - 1 
                && (ARCH_STATE == LEFT_0 || ARCH_STATE == LEFT_180 || UTURN == BUMP_WALL))
            {
                archBound.left = cur.y + 1;
                archBound.right = _map.ymin;

                FRIZY_LOG(LOG_DEBUG,"gong3.%d.%d\n",archBound.left,archBound.right);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);
            }
            if (_map.ymax - cur.y >= AREA_LENGTH - 1 
                && (ARCH_STATE == RIGHT_0 || ARCH_STATE == RIGHT_180 || UTURN == BUMP_WALL))
            {
                archBound.left = _map.ymax;
                archBound.right = cur.y - 1;						
                FRIZY_LOG(LOG_DEBUG,"gong4.%d.%d\n",archBound.left,archBound.right);
                boundArr.push_back({archBound.up,archBound.down,archBound.left,archBound.right});
                blockPlan.SetBound(archBound);				
            }		
        }


        int temps = 0;
        for(int x = cur.x - 1;x <= cur.x + 1; x++)
        {
            for(int y = cur.y - 1; y <= cur.y + 1; y++)
            {
                temps = _maze.GetMapState(x,y,2) == 0 ? 0:temps+1;
            }
        } 

        if (temps == 9
            || (cur.y >= archBound.left 
				&& (sensor.bump != 0 || sensor.obs != 0 
                    || (_maze.GetMapState(cur.x+1,cur.y,2) != 0 && _maze.GetMapState(cur.x-1,cur.y,2) != 0)) 
				&& (ARCH_STATE == LEFT_0 || ARCH_STATE == LEFT_180))
		    || (cur.y <= archBound.right 
				&& (sensor.bump != 0 || sensor.obs != 0 
                    || (_maze.GetMapState(cur.x+1,cur.y,2) != 0 && _maze.GetMapState(cur.x-1,cur.y,2) != 0)) 
				&& (ARCH_STATE == RIGHT_0 || ARCH_STATE == RIGHT_180))
            || (cur.x >= archBound.up && (cur.y >= archBound.left || cur.y <= archBound.right) 
				&& (ARCH_STATE == LEFT_0 || ARCH_STATE == RIGHT_0))
		    || (cur.x <= archBound.down && (cur.y >= archBound.left || cur.y <= archBound.right) 
				&& (ARCH_STATE == LEFT_180 || ARCH_STATE == RIGHT_180)))
        {
            searchKind = searchUnclean;
            FRIZY_LOG(LOG_DEBUG,"beibaowei");
            return 1;
        }
            

    }

    
    void UTurnPlanning::BumpWall(Sensor sensor,Grid cur)
    {

        if ((cur.addAngle - bumpwall.addAngle > 180 && cur.x == bumpwall.x && cur.y == bumpwall.y) 
            || cur.addAngle - bumpwall.addAngle > 330
            || ((cur.x <= archBound.down + 1 || cur.x >= archBound.up - 1
                    || cur.y <= archBound.right + 1 || cur.y >= archBound.left - 1)
                && abs(cur.x - bumpwall.x) + abs(cur.y - bumpwall.y) > 3) 
            || abs(cur.x - bumpwall.x) + abs(cur.y - bumpwall.y) > 10
            || find(boundPoint.begin(), boundPoint.end(),cur) != boundPoint.end())

            //find(v.begin(), v.end(), key)-v.begin() 获得索引
        {   
            
            FRIZY_LOG(LOG_DEBUG,"chongxinsou.%d,%d,%d",bumpwall.x,bumpwall.y,bumpwall.addAngle);
            do{
            StopWallFollow();
            chassisPlan.GetSensor(&sensor);
            usleep(10 * 1000);
            }while (sensor.leftw != 0 || sensor.rightw != 0);    

            if (cur.x < archBound.up && cur.x > archBound.down && cur.y < archBound.left && cur.y > archBound.right
                && (_maze.GetMapState(cur.x+1,cur.y,2) == 0 || _maze.GetMapState(cur.x-1,cur.y,2) == 0
                || _maze.GetMapState(cur.x,cur.y+1,2) == 0 || _maze.GetMapState(cur.x,cur.y-1,2) == 0)){
                    
                    FRIZY_LOG(LOG_DEBUG,"rearchle");
                    do{
                        StopWallFollow();
                        chassisPlan.GetSensor(&sensor);
                        usleep(10 * 1000);
                    }while (sensor.leftw != 0 || sensor.rightw != 0);  

                    UTURN = ARCH,ARCH_STATE = NO_ARCH;
                    searchKind = idle;
                    return;	                   
            }   
            searchKind = searchUnclean;
            UTURN = SEARCH_UNCLEAN;
            return;			
        }  
        else{

            FRIZY_LOG(LOG_DEBUG,"bumpwalling");
            StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
            return;            
        }   
    }
    //弓字型
    void UTurnPlanning::Arch(Sensor sensor,Grid cur)
    {


        if (StopArch(sensor,cur))
        {
            UTURN = SEARCH_UNCLEAN;
            return;
        }

        if (ARCH_STATE == NO_ARCH)
        {
            ARCH_STATE = UporDown(cur,cur,0).refor;
            FRIZY_LOG(LOG_DEBUG,"rearch.%d",ARCH_STATE);
        }
        
        switch (ARCH_STATE)
        {
            case RIGHT_0:
            printf("RIGHT_0.%d\n",sensor.bump);

            if ((_maze.GetMapState(cur.x+1,cur.y,2) == 1 && _maze.GetMapState(cur.x+2,cur.y,2) != 0)
                || cur.x == archBound.up 
                || (cur.x + 1 == archBound.up && _maze.GetMapState(cur.x+1,cur.y,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x+1,cur.y) == 1)
            {
			    if ((cur.y != archBound.right + 1 || _maze.GetMapState(cur.x,cur.y-1,2) != 0)
                    && ((_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y+1,2) == 0 
                            && _maze.GetMapState(cur.x-2,cur.y+1,2) == 0)
					    || (_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x,cur.y-1,2) != 0 
                            && _maze.GetMapState(cur.x-1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x-2,cur.y-1,2) != 0))
                    && _maze.VirBound(cur.x,cur.y+1) == 0)
                {
                    printf("arch zhuan5\n");
                    aim.x = cur.x;
                    aim.y = cur.y + sensor.size;
                    aim.forward = 270;
                    ARCH_STATE = LEFT_PRE_180;
                    controlw.WheelControl(sensor,cur,aim);                    
                }
                else if ((sensor.bump != 0 || sensor.obs != 0)
                        && _maze.GetMapState(cur.x+2,cur.y,2) == 0 && _maze.GetMapState(cur.x+2,cur.y-1,2) == 0
                        && _maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y+1,2) == 0 
                        && cur.x < archBound.up - 3)
                {
                    FRIZY_LOG(LOG_DEBUG,"_bumpwall1.%d.%d.%d",cur.x,cur.y,cur.addAngle);
                    bumpwall = cur;
                    UTURN = BUMP_WALL;
                    return;
                }
                else
                {
                    printf("arch zhuan1\n");
                    aim.x = cur.x;
                    aim.y = cur.y - sensor.size;
                    aim.forward = 90;
                    ARCH_STATE = RIGHT_PRE_180;
                    controlw.WheelControl(sensor,cur,aim);                    
                }
            }
            else
            {
                printf("right0\n");
                aim.x = cur.x + 1;
                aim.y = cur.y;
                aim.forward = 0;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;

            case RIGHT_PRE_180:
            if (cur.y == aim.y)
            {
                printf("get1\n");
                aim.x = cur.x - 1;
                aim.y = cur.y;
                aim.forward = 180;
                ARCH_STATE = RIGHT_180;
                controlw.WheelControl(sensor,cur,aim);
            }
            else if ((_maze.GetMapState(cur.x,cur.y-1,2) != 0 
                    && _maze.GetMapState(cur.x-1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x-2,cur.y-1,2) != 0)
				|| _maze.VirBound(cur.x,cur.y-1) == 1)
			{

				printf("zhijiesou1\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;							
			}

            else if (sensor.bump != 0 || sensor.obs != 0)
            {
                printf("archwall1\n");

                UTURN = ARCH_WALL;
                arch_wall = RIGHTWALL_180;

                return;                   
            }
            else
            {
                printf("right90\n");
                controlw.WheelControl(sensor,cur,aim);
            }
            break;


            case RIGHT_180:
            if ((_maze.GetMapState(cur.x-1,cur.y,2) == 1 && _maze.GetMapState(cur.x-2,cur.y,2) != 0)
                || cur.x == archBound.down 
                || (cur.x - 1 == archBound.down && _maze.GetMapState(cur.x-1,cur.y,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x-1,cur.y) == 1)
            {

			    if ((cur.y != archBound.right + 1 || _maze.GetMapState(cur.x,cur.y-1,2) != 0)
                     && ((_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y+1,2) == 0 
                            && _maze.GetMapState(cur.x+2,cur.y+1,2) == 0)
					    || (_maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x,cur.y-1,2) != 0 
                            && _maze.GetMapState(cur.x+1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x+2,cur.y-1,2) != 0))
                    && _maze.VirBound(cur.x,cur.y+1) == 0)
                {
                    printf("arch zhuan6\n");
                    aim.x = cur.x;
                    aim.y = cur.y + sensor.size;
                    aim.forward = 270;
                    ARCH_STATE = LEFT_PRE_0;
                    controlw.WheelControl(sensor,cur,aim);

                }
                else if ((sensor.bump != 0 || sensor.obs != 0)
                        && _maze.GetMapState(cur.x-2,cur.y,2) == 0 && _maze.GetMapState(cur.x-2,cur.y-1,2) == 0
                        && _maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 0
                        && cur.x > archBound.down + 3)
                {
                    FRIZY_LOG(LOG_DEBUG,"_bumpwall2.%d.%d.%d",cur.x,cur.y,cur.addAngle);

                    bumpwall = cur;
                    UTURN = BUMP_WALL;
                    return;
                }                
                else
                {
                    printf("arch zhuan2\n");
                    aim.x = cur.x;
                    aim.y = cur.y - sensor.size;
                    aim.forward = 90;
                    ARCH_STATE = RIGHT_PRE_0;
                    controlw.WheelControl(sensor,cur,aim);                    
                }

            }
            else
            {
                printf("right180\n");
                aim.x = cur.x - 1;
                aim.y = cur.y;
                aim.forward = 180;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;

            case RIGHT_PRE_0:
            if (cur.y == aim.y)
            {
                printf("get2\n");
                aim.x = cur.x + 1;
                aim.y = cur.y;
                aim.forward = 0;
                ARCH_STATE = RIGHT_0;
                controlw.WheelControl(sensor,cur,aim);
            }
            else if ((_maze.GetMapState(cur.x,cur.y-1,2) != 0 
                    && _maze.GetMapState(cur.x+1,cur.y-1,2) != 0 && _maze.GetMapState(cur.x+2,cur.y-1,2) != 0)
				|| _maze.VirBound(cur.x,cur.y-1) == 1)
			{
				printf("zhijiesou2\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;							
			}            

            else if (sensor.bump != 0 || sensor.obs != 0)
            {
                printf("archwall2\n");
                UTURN = ARCH_WALL;
                arch_wall = RIGHTWALL_0;
                
                // aim.x = cur.x + 1;
                // aim.y = cur.y;
                // aim.forward = 0;
                // ARCH_STATE = LEFT_0;
                // controlw.WheelControl(sensor,cur,aim);


                return;                   
            }                
            else
            {
                controlw.WheelControl(sensor,cur,aim);
            }
            break;  


            case LEFT_0:
            if ((_maze.GetMapState(cur.x+1,cur.y,2) == 1 && _maze.GetMapState(cur.x+2,cur.y,2) != 0)
                || cur.x == archBound.up
                || (cur.x + 1 == archBound.up && _maze.GetMapState(cur.x+1,cur.y,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x+1,cur.y) == 1)
            {
			    if ((cur.y != archBound.left - 1 || _maze.GetMapState(cur.x,cur.y+1,2) != 0)
                    && ((_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y-1,2) == 0 
                            && _maze.GetMapState(cur.x-2,cur.y-1,2) == 0)
					    || (_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x,cur.y+1,2) != 0 
                            && _maze.GetMapState(cur.x-1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x-2,cur.y+1,2) != 0))
                    && _maze.VirBound(cur.x,cur.y-1) == 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan7");
                    aim.x = cur.x;
                    aim.y = cur.y - sensor.size;
                    aim.forward = 90;
                    ARCH_STATE = RIGHT_PRE_180;
                    controlw.WheelControl(sensor,cur,aim);

                }
                else if ((sensor.bump != 0 || sensor.obs != 0)
                        && _maze.GetMapState(cur.x+2,cur.y,2) == 0 && _maze.GetMapState(cur.x+2,cur.y+1,2) == 0
                        && _maze.GetMapState(cur.x,cur.y+1,2) == 0 && _maze.GetMapState(cur.x-1,cur.y+1,2) == 0
                        && cur.x < archBound.up - 3)
                {
                    FRIZY_LOG(LOG_DEBUG,"_bumpwall3.%d.%d.%d",cur.x,cur.y,cur.addAngle);

                    bumpwall = cur;
                    UTURN = BUMP_WALL;
                    return;
                }                   
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan3");
                    aim.x = cur.x;
                    aim.y = cur.y + sensor.size;
                    aim.forward = 270;
                    ARCH_STATE = LEFT_PRE_180;
                    controlw.WheelControl(sensor,cur,aim);
                }
            }
            else
            {
                aim.x = cur.x + 1;
                aim.y = cur.y;
                aim.forward = 0;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;

            case LEFT_PRE_180:
            if (cur.y == aim.y)
            {
                aim.x = cur.x - 1;
                aim.y = cur.y;
                aim.forward = 180;
                ARCH_STATE = LEFT_180;
                controlw.WheelControl(sensor,cur,aim);
            }
            else if ((_maze.GetMapState(cur.x,cur.y+1,2) != 0 
                    && _maze.GetMapState(cur.x-1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x-2,cur.y+1,2) != 0)
				|| _maze.VirBound(cur.x,cur.y+1) == 1)
			{

				printf("zhijiesou3\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;							
			}            

            else if (sensor.bump != 0 || sensor.obs != 0)
            {
                printf("archwall3\n");
                UTURN = ARCH_WALL;
                arch_wall = LEFTWALL_180;
                // aim.x = cur.x - 1;
                // aim.y = cur.y;
                // aim.forward = 180;
                // ARCH_STATE = RIGHT_180;
                // controlw.WheelControl(sensor,cur,aim);                    
                return;                   
            }                
            else
            {
                controlw.WheelControl(sensor,cur,aim);
            }
            break;


            case LEFT_180:
            if ((_maze.GetMapState(cur.x-1,cur.y,2) == 1 && _maze.GetMapState(cur.x-2,cur.y,2) != 0)
                || cur.x == archBound.down
                || (cur.x - 1 == archBound.down && _maze.GetMapState(cur.x-1,cur.y,2) != 0)
                || sensor.bump != 0 || sensor.obs != 0
                || _maze.VirBound(cur.x-1,cur.y) == 1)
            {
			    if ((cur.y != archBound.left - 1 || _maze.GetMapState(cur.x,cur.y+1,2) != 0)
                    && ((_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 0 
                            && _maze.GetMapState(cur.x+2,cur.y-1,2) == 0)
					    || (_maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x,cur.y+1,2) != 0 
                            && _maze.GetMapState(cur.x+1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x+2,cur.y+1,2) != 0))
                    && _maze.VirBound(cur.x,cur.y-1) == 0)
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan8");
                    aim.x = cur.x;
                    aim.y = cur.y - sensor.size;
                    aim.forward = 90;
                    ARCH_STATE = RIGHT_PRE_0;
                    controlw.WheelControl(sensor,cur,aim);

                }  
                else if ((sensor.bump != 0 || sensor.obs != 0)
                        && _maze.GetMapState(cur.x-2,cur.y,2) == 0 && _maze.GetMapState(cur.x-2,cur.y+1,2) == 0
                        && _maze.GetMapState(cur.x,cur.y-1,2) == 0 && _maze.GetMapState(cur.x+1,cur.y-1,2) == 0
                        && cur.x > archBound.down + 3)
                {
                    FRIZY_LOG(LOG_DEBUG,"_bumpwall4.%d.%d.%d",cur.x,cur.y,cur.addAngle);

                    bumpwall = cur;
                    UTURN = BUMP_WALL;
                    return;
                }                     
                else
                {
                    FRIZY_LOG(LOG_DEBUG,"arch zhuan4");
                    aim.x = cur.x;
                    aim.y = cur.y + sensor.size;
                    aim.forward = 270;
                    ARCH_STATE = LEFT_PRE_0;
                    controlw.WheelControl(sensor,cur,aim);                    
                }
            }
            else
            {
                aim.x = cur.x - 1;
                aim.y = cur.y;
                aim.forward = 180;
                controlw.WheelControl(sensor,cur,aim);
            }
            break;


            case LEFT_PRE_0:
            if (cur.y == aim.y)
            {
                aim.x = cur.x + 1;
                aim.y = cur.y;
                aim.forward = 0;
                ARCH_STATE = LEFT_0;
                controlw.WheelControl(sensor,cur,aim);
            }
            else if ((_maze.GetMapState(cur.x,cur.y+1,2) != 0 
                    && _maze.GetMapState(cur.x+1,cur.y+1,2) != 0 && _maze.GetMapState(cur.x+2,cur.y+1,2) != 0)
				|| _maze.VirBound(cur.x,cur.y+1) == 1)
			{

				printf("zhijiesou4\n");

                searchKind = searchUnclean;
                UTURN = SEARCH_UNCLEAN;

                return;							
			}             

            else if (sensor.bump != 0 || sensor.obs != 0)
            {
                printf("archwall4\n");
                UTURN = ARCH_WALL;
                arch_wall = LEFTWALL_0;
                // aim.x = cur.x + 1;
                // aim.y = cur.y;
                // aim.forward = 0;
                // ARCH_STATE = RIGHT_0;
                // controlw.WheelControl(sensor,cur,aim);       
                
                return;                   
            }                  
            else
            {
                controlw.WheelControl(sensor,cur,aim);
            }
            break;                 

            default:
                break;
        }
        
    }
   
   //zhaoqiang
    void UTurnPlanning::SearchWall(Sensor sensor,Grid cur)
    {
        
        // printf("get wall\n");
        // StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
        // process = BOUND;
        // return;

        if (_trouble.type == windcolumn)
        {
            printf("RI\n");
             if (IsWall() != 0)
             {
                 do{
                     StopWallFollow();
                     chassisPlan.GetSensor(&sensor);
                     usleep(10*1000);
                    }while (sensor.leftw != 0 || sensor.rightw != 0);
             }
            _trouble.type = nothing;
            archBound.down = cur.x;
            archBound.up = cur.x + AREA_LENGTH;
            archBound.left = 1000;
            archBound.right = -1000;
            blockPlan.SetBound(archBound);
            process = BOUND;
            return;
        }
        
        if ((abs(cur.x - aim.x) + abs(cur.y - aim.y) < sensor.size * 3 && (sensor.bump || sensor.obs))
            || abs(cur.x - aim.x) + abs(cur.y - aim.y) == 0
            || keepwallTime > 400
            || sensor.leftFrontVir || sensor.rightFrontVir)
        {
            FRIZY_LOG(LOG_DEBUG,"get wall");
            StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
            process = BOUND;
            return;
        }
        
        if (aim.x == 1000 || (keepwallTime % 100 == 0 && keepwallTime > 0))
        {
            aim.forward = 1000;
            int dis = 1000;
            //右
            for (int i = -100; i <= cur.y; i++)
            {
                if (_maze.GetMapState(cur.x,i,1) == 2
                    && (_maze.GetMapState(cur.x+1,i,1) == 2 && _maze.GetMapState(cur.x+2,i,1) == 2
                            && _maze.GetMapState(cur.x-1,i,1) == 2 && _maze.GetMapState(cur.x-2,i,1) == 2))
                {
                    if (abs(cur.y - i) < dis)
                    {
                        aim.x = cur.x;
                        aim.y = i;
                        dis = abs(cur.y - i);
                    }
                    break;
                }
            }

            //左
            for (int i = 100; i >= cur.y; i--)
            {
                if (_maze.GetMapState(cur.x,i,1) == 2
                    && (_maze.GetMapState(cur.x+1,i,1) == 2 && _maze.GetMapState(cur.x+2,i,1) == 2
                            && _maze.GetMapState(cur.x-1,i,1) == 2 && _maze.GetMapState(cur.x-2,i,1) == 2))
                {
                    if (abs(cur.y - i) < dis)
                    {
                        aim.x = cur.x;
                        aim.y = i;
                        dis = abs(cur.y - i);
                    }
                    break;
                }
            }

            //上
            for (int i = 100; i >= cur.x; i--)
            {
                if (_maze.GetMapState(i,cur.y,1) == 2
                    && (_maze.GetMapState(i,cur.y+1,1) == 2 && _maze.GetMapState(i,cur.y+2,1) == 2
                            && _maze.GetMapState(i,cur.y-1,1) == 2 && _maze.GetMapState(i,cur.y-2,1) == 2))
                {
                    if (abs(cur.x - i) < dis)
                    {
                        aim.x = i;
                        aim.y = cur.y;
                        dis = abs(cur.x - i);
                    }
                    break;
                }
            }

            //下
            for (int i = -100; i <= cur.x; i++)
            {
                if (_maze.GetMapState(i,cur.y,1) == 2
                    && (_maze.GetMapState(i,cur.y+1,1) == 2 && _maze.GetMapState(i,cur.y+2,1) == 2
                            && _maze.GetMapState(i,cur.y-1,1) == 2 && _maze.GetMapState(i,cur.y-2,1) == 2))
                {
                    if (abs(cur.x - i) < dis)
                    {
                        aim.x = i;
                        aim.y = cur.y;
                        dis = abs(cur.x - i);
                    }
                    break;
                }
            }
                        
            FRIZY_LOG(LOG_DEBUG,"searchwall a.%d,b.%d,dis.%d",aim.x,aim.y,dis);
        }

        if (aim.x == 1000)
        {
            int dis = 1000;
            for (int x = -95;x <= 95;x++)
            {
                for (int y = -95;y <= 95;y++)
                {
                    if (_maze.GetMapState(x,y,1) == 2 && abs(cur.x-x) + abs(cur.y-y) < dis)
                    {
                        dis = abs(cur.x-x) + abs(cur.y-y);
                        aim.x = x;
                        aim.y = y;
                    }
                }	
            }   
            FRIZY_LOG(LOG_DEBUG,"jiaqiang.%d,%d",aim.x,aim.y);        
        }
		
        if ((sensor.bump || sensor.obs)
            && IsWall() == 0 && aim.x != 1000)
        {
            FRIZY_LOG(LOG_DEBUG,"start wall wall");
            StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
        }
        else if (IsWall() != 0)
        {
            FRIZY_LOG(LOG_DEBUG,"wall wall");
            keepwallTime ++;
        }
        else if (aim.x != 1000)
        {
            if (aim.forward == 1000)
                aim = roadPlan.ConAngle(cur,aim);
            
            if (IsWall() != 0)
            {
                do{
                    StopWallFollow();
                    chassisPlan.GetSensor(&sensor);
                    usleep(10 * 1000);
                }while (sensor.leftw != 0 || sensor.rightw != 0);
            }
            
 
            // static int sign1 = 5;

            if (sign1 > 0)
            {
                sign1 --;
                for (int x = -20;x <= 20;x++)
                    for (int y = -20;y <= 20;y++)
                        if (_maze.GetMapState(x,y,1) == 2)
                            printf("@300=%04d,%04d,00%d\n",x,y,2); 
            }     

            FRIZY_LOG(LOG_DEBUG,"go wall.%d.%d.%f",aim.x,aim.y,aim.forward);
            controlw.WheelControl(sensor,cur,aim); 
        }
        else
        {
            FRIZY_LOG(LOG_DEBUG,"wait...");

            static int sign = 0;
            
            if (sign % 6 == 0)
            {
                
                for (int x = -20;x <= 20;x++)
                    for (int y = -20;y <= 20;y++)
                        if (_maze.GetMapState(x,y,1) == 2)
                            printf("@300=%04d,%04d,00%d\n",x,y,2);    
            }         
            sign++;
            controlw.ClearPid();
            chassisPlan.chassisSpeed(100,-100,1);
            
        }   
        
        return;
        
    }

    //区域内部行走逻辑
    void UTurnPlanning::InsidePlanning(Sensor sensor,Grid cur,Trouble trouble)
    {
        
        blockPlan.GetBound(&archBoundArr,&archBound); 
        _trouble = trouble;

        FRIZY_LOG(LOG_DEBUG,"uturn.%d.%d,for:%d,%d,%d,%d,searchKind.%d,%d"
        ,UTURN,sensor.bump,archBound.up,archBound.down,archBound.left,archBound.right,searchKind,archBoundArr.size());     

        if (searchKind != idle && searchKind != archwallSign)
            UTURN = SEARCH_UNCLEAN;

        if (searchKind == archwallSign)
        {
            FRIZY_LOG(LOG_DEBUG,"gaseng");
            UTURN = ARCH_WALL;
            searchKind = idle;
            Rearch gaSeng;
 
            gaSeng = UporDown(cur,gaSengAim,0);

            if (RIGHT_0 == gaSeng.refor)
            {
                FRIZY_LOG(LOG_DEBUG,"gaseng1");
                arch_wall = RIGHTWALL_0;
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
            }
            if (RIGHT_180 == gaSeng.refor)
            {
                FRIZY_LOG(LOG_DEBUG,"gaseng2");
                arch_wall = RIGHTWALL_180;
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
            }
            if (LEFT_0 == gaSeng.refor)
            {
                FRIZY_LOG(LOG_DEBUG,"gaseng3");
                arch_wall = LEFTWALL_0;
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
            }
            if (LEFT_180 == gaSeng.refor)
            {
                FRIZY_LOG(LOG_DEBUG,"gaseng4");
                arch_wall = LEFTWALL_180;
                StartWallFollow(RandomWallFollow, RIGHTAW, QUICK);
            }   
            
        }

        switch (UTURN)
        {
            case SEARCH_WALL:
                SearchWall(sensor,cur);
                break;     

            case BUMP_WALL:
                BumpWall(sensor,cur);
                break;       
             
            case ARCH_WALL:
                ArchWall(sensor,cur);
                break;

            case SEARCH_UNCLEAN:
                SearchUnlean(sensor,cur);
                break;  

            case ARCH:
                Arch(sensor,cur);
                break;

            default:
                break;
        }

    }


    void UTurnPlanning::uTurnMovePath(Point &startPoint, blockCorner &selectBlockCorner,RobotType &robotShape)
    {
        FRIZY_LOG(LOG_INFO, "start to find path at uturn model ");
        // currentpose = GetCurGridPose();
        uTurnPath.clear();// 清空之前的路径
        if (robotShape ==RobotType::circle )
        {

            FRIZY_LOG(LOG_INFO, "the robot type is cirle" );
            currentpose = GetCurGridPose();
            
            //机器人四种case
            switch (_cornerIdex)
            {
            case 0 :
                FRIZY_LOG(LOG_INFO, "the selected is topLeftCorner" );
                rotato_yawl = 3*_Pi/2 - currentpose.forward;
                if(!turnRoundCorner(rotato_yawl))
                {
                    FRIZY_LOG(LOG_ERROR, "turnRoundCorner is false" );
                    break;
                }
                moveTrajectory(startPoint,selectBlockCorner,_cornerIdex, robotShape);
                break;
            case 1 :
                FRIZY_LOG(LOG_INFO, "the selected is topRightCorner" );
                rotato_yawl = 3*_Pi/2 - currentpose.forward;
                if(!turnRoundCorner(rotato_yawl))
                {
                    FRIZY_LOG(LOG_ERROR, "turnRoundCorner is false" );
                    break;
                }
                moveTrajectory(startPoint,selectBlockCorner,_cornerIdex, robotShape);               
                break;                
            case 2 :
                FRIZY_LOG(LOG_INFO, "the selected is bottomLeftCorner" );
                rotato_yawl = _Pi/2 - currentpose.forward;
                if(!turnRoundCorner(rotato_yawl))
                {
                    FRIZY_LOG(LOG_ERROR, "turnRoundCorner is false" );
                    break;
                }
                moveTrajectory(startPoint,selectBlockCorner,_cornerIdex, robotShape);              
                break;
            case 3 :
                FRIZY_LOG(LOG_INFO, "the selected is bottomRightCorner" );
                rotato_yawl = _Pi/2 - currentpose.forward;
                if(!turnRoundCorner(rotato_yawl))
                {
                    FRIZY_LOG(LOG_ERROR, "turnRoundCorner is false" );
                    break;
                }
                moveTrajectory(startPoint,selectBlockCorner,_cornerIdex, robotShape);              
                break;                            
            default:
                FRIZY_LOG(LOG_ERROR, "The _cornerIdex is NULL" );
                break;
            }

        }

        else if(robotShape ==RobotType::rectangle)
        {
            /* need to do */
        }


        
        
    }
    pointCoordinate UTurnPlanning::getNearestCoordinate(Point &startPoint, blockCorner &selectBlockCorner)
    {
        FRIZY_LOG(LOG_INFO, "start to get the min euclidean corner ");
        //欧几里得距离最近
        topLeftEuclidean = pow((selectBlockCorner.topLeftCorner.x - startPoint.x),2)+pow((selectBlockCorner.topLeftCorner.y - startPoint.y),2);
        topRightEuclidean = pow((selectBlockCorner.topRightCorner.x - startPoint.x),2)+pow((selectBlockCorner.topRightCorner.y - startPoint.y),2);
        bottomLeftEuclidean = pow((selectBlockCorner.bottomLeftCorner.x - startPoint.x),2)+pow((selectBlockCorner.bottomLeftCorner.y - startPoint.y),2);
        bottomRightEuclidean = pow((selectBlockCorner.bottomRightCorner.x - startPoint.x),2)+pow((selectBlockCorner.bottomRightCorner.y - startPoint.y),2);
        minEuclidean = min(min(topLeftEuclidean,topRightEuclidean),min(bottomLeftEuclidean,bottomRightEuclidean));
    
        //计算最近的corner
        if  (minEuclidean== topLeftEuclidean)
            {
                _cornerIdex = _topLeftCorner;
                return  selectBlockCorner.topLeftCorner;
            }
        if  (minEuclidean== topRightEuclidean)
            {
                _cornerIdex = _topRightCorner;
                return  selectBlockCorner.topRightCorner;
            }            
        if  (minEuclidean== bottomLeftEuclidean)
            {
                _cornerIdex = _bottomLeftCorner;
                return  selectBlockCorner.bottomLeftCorner;
            }
        if  (minEuclidean== bottomRightEuclidean)
            {
                _cornerIdex = _bottomRightCorner;
                return  selectBlockCorner.bottomRightCorner;
            }            
        
    }

    list<Point*> UTurnPlanning::uTurnGetPath()
    {
        FRIZY_LOG(LOG_INFO, "start to get  the path in  uturn model" );
        return uTurnPath;
    }

    bool UTurnPlanning::goCorner(Point &startPoint, blockCorner &selectBlockCorner,RobotType &robotShape)
    {
        FRIZY_LOG(LOG_INFO, "start to go corner in uturn model" );
        if (robotShape ==RobotType::circle )
        {
            outLineIndex = robot_radius/resolution;
            // 要检验下地图的坐标xy轴正负
            reselectBlockCorner.topLeftCorner.x = selectBlockCorner.topLeftCorner.x+int(outLineIndex);
            reselectBlockCorner.topLeftCorner.y = selectBlockCorner.topLeftCorner.y+int(outLineIndex);
            reselectBlockCorner.topRightCorner.x = selectBlockCorner.topRightCorner.x-int(outLineIndex);
            reselectBlockCorner.topRightCorner.y = selectBlockCorner.topRightCorner.y+int(outLineIndex);
            reselectBlockCorner.bottomLeftCorner.x = selectBlockCorner.bottomLeftCorner.x+int(outLineIndex);
            reselectBlockCorner.bottomLeftCorner.y = selectBlockCorner.bottomLeftCorner.y-int(outLineIndex);
            reselectBlockCorner.bottomRightCorner.x = selectBlockCorner.bottomRightCorner.x-int(outLineIndex);
            reselectBlockCorner.bottomRightCorner.y = selectBlockCorner.bottomRightCorner.y-int(outLineIndex);
            //获得最近的corner
            nearestCorner = getNearestCoordinate(startPoint,selectBlockCorner);
            auto astarMaze = Maze(_maze);
            astarMaze.startPoint = startPoint;
            // endpoint 赋值 nearestCorner
            astarMaze.endPoint.x = nearestCorner.x;
            astarMaze.endPoint.y = nearestCorner.y; 

            auto Astar = aStar();
            auto Motioncontrol = MotionControl();

            // 点到点规划
            astarResult = Astar.findPath(astarMaze.startPoint, astarMaze.endPoint, robotShape);
            astarPath = Astar.getPath(astarResult);
            Motioncontrol.pathTransformtoOrder(astarPath); //路径到motioncontrol模块


        }

        else if(robotShape ==RobotType::rectangle)
        {
            /* need to do */
        }

        return true ;

    }
    bool UTurnPlanning::turnRoundCorner(float &rotatoyawl)
    {
        if (rotatoyawl >0)
        {
            _wheel_mode = left;

        }
        if (rotatoyawl <0)
        {
            _wheel_mode = right;
        }            
        _move_data.rotateangle = rotatoyawl;
        _move_data.speed = 0;
        _motioncontrol.wheelControl(_wheel_mode,_move_data); //执行转向
        return true;  
    }
    void UTurnPlanning::moveTrajectory(Point &startPoint,blockCorner &selectBlockCorner,cornerIdex &_cornerIdex,RobotType &robotShape)
    {
        FRIZY_LOG(LOG_INFO, "start to plan trajectory" );
        // Point *temp_point= &startPoint;
        // uTurnPath.push_front(temp_point);
        _currentPoint.x = startPoint.x;
        _currentPoint.y = startPoint.y;
        if (robotShape ==RobotType::circle )
        {
            switch (_cornerIdex)
            {
            case 0:
                {
                FRIZY_LOG(LOG_INFO, "start to plan trajectory at _topLeftCorner" );
                robot_orientation = 1;
                // 会存在5-10cm漏扫可能 需要添加test
                for(int i = startPoint.x; i < reselectBlockCorner.topRightCorner.x; i+=cleaning_interval)
                {
                    _endPoint.x = _currentPoint.x;
                    if (robot_orientation ==1)
                    {                        
                        _endPoint.y = reselectBlockCorner.bottomLeftCorner.y;
                    }
                    if (robot_orientation == -1)
                    {
                        _endPoint.y = startPoint.y;
                    }
                    uTurnLinePathControl(_currentPoint,_endPoint);
                    if(turnIndex == true)
                    {
                        if((i+cleaning_interval) < reselectBlockCorner.topRightCorner.x)
                        {
                            FRIZY_LOG(LOG_INFO, "start to turnMove at _topLeftCorner" );
                            if(turnMove(_currentPoint,_cornerIdex)!= true)
                            FRIZY_LOG(LOG_ERROR, "fail to turnMove at _topLeftCorner" );
                            else
                            FRIZY_LOG(LOG_INFO, "success to turnMove at _topLeftCorner" );
                        }
                        else
                        {   auto _cornerIdex_tmp  = cornerIdex::_topRightCorner;
                            if(turnMove(_currentPoint,_cornerIdex_tmp) == true)
                            {
                                FRIZY_LOG(LOG_INFO, "move to the right boundary" );
                            }
                            else
                            {
                                FRIZY_LOG(LOG_ERROR, "fail to move to the right boundary" );
                            }
                        }
                    }

                }
                // 覆盖因为障碍turnround导致未清扫区域
                first_planning_index == true;
                for(int i = _currentPoint.x; i >startPoint.x; i-=cleaning_interval)
                {   
                    auto &p = uTurnPoint.front();
                    if(i == p->x )
                    {
                        // need to do
                        _currentPoint.x = i;
                        uTurnPath.push_back(&_currentPoint);
                        if (robot_orientation ==1)
                        {                        
                            _endPoint.y = reselectBlockCorner.bottomLeftCorner.y;
                        }
                        if (robot_orientation == -1)
                        {
                            _endPoint.y = p->y;
                        }
                        uTurnLinePathControl(_currentPoint,_endPoint);

                        uTurnPoint.pop_front();
                    }
                    
                }
                first_planning_index == false;   
                break;
                }
            case 1:
                {
                FRIZY_LOG(LOG_INFO, "start to plan trajectory at _topRightCorner" );
                robot_orientation = 1;
                // 会存在5-10cm漏扫可能 需要添加test
                for(int i = startPoint.x; i > reselectBlockCorner.topLeftCorner.x; i-=cleaning_interval)
                {
                    _endPoint.x = _currentPoint.x;
                    if (robot_orientation ==1)
                    {                        
                        _endPoint.y = reselectBlockCorner.bottomRightCorner.y;
                    }
                    if (robot_orientation == -1)
                    {
                        _endPoint.y = startPoint.y;
                    }
                    uTurnLinePathControl(_currentPoint,_endPoint);
                    if(turnIndex == true)
                    {
                        if((i-cleaning_interval) > reselectBlockCorner.topLeftCorner.x)
                        {
                            FRIZY_LOG(LOG_INFO, "start to turnMove at _topRightCorner" );
                            if(turnMove(_currentPoint,_cornerIdex)!= true)
                            FRIZY_LOG(LOG_ERROR, "fail to turnMove at _topRightCorner" );
                            else
                            FRIZY_LOG(LOG_INFO, "success to turnMove at _topRightCorner" );
                        }
                        else
                        {   auto _cornerIdex_tmp  = cornerIdex::_topLeftCorner;
                            if(turnMove(_currentPoint,_cornerIdex_tmp) == true)
                            {
                                FRIZY_LOG(LOG_INFO, "move to the left boundary" );
                            }
                            else
                            {
                                FRIZY_LOG(LOG_ERROR, "fail to move to the left boundary" );
                            }
                        }
                    }

                }
                // 覆盖因为障碍turnround导致未清扫区域
                first_planning_index == true;
                for(int i = _currentPoint.x; i <startPoint.x; i+=cleaning_interval)
                {   auto &p = uTurnPoint.front();
                    if(i == p->x )
                    {
                        // need to do
                        _currentPoint.x = i;
                        uTurnPath.push_back(&_currentPoint);
                        if (robot_orientation ==1)
                        {                        
                            _endPoint.y = reselectBlockCorner.bottomLeftCorner.y;
                        }
                        if (robot_orientation == -1)
                        {
                            _endPoint.y = p->y;
                        }
                        uTurnLinePathControl(_currentPoint,_endPoint);

                        uTurnPoint.pop_front();
                    }
                }
                first_planning_index == false;                      
                break;
                }
            case 2:
                {
                FRIZY_LOG(LOG_INFO, "start to plan trajectory at _bottomLeftCorner" );
                robot_orientation = -1;
                // 会存在5-10cm漏扫可能 需要添加test
                for(int i = startPoint.x; i < reselectBlockCorner.topRightCorner.x; i+=cleaning_interval)
                {
                    _endPoint.x = _currentPoint.x;
                    if (robot_orientation ==1)
                    {                        
                        _endPoint.y = startPoint.y;
                    }
                    if (robot_orientation == -1)
                    {
                        _endPoint.y = reselectBlockCorner.topLeftCorner.y;
                    }
                    uTurnLinePathControl(_currentPoint,_endPoint);
                    if(turnIndex == true)
                    {
                        if((i+cleaning_interval) < reselectBlockCorner.topRightCorner.x)
                        {
                            FRIZY_LOG(LOG_INFO, "start to turnMove at _bottomLeftCorner" );
                            if(turnMove(_currentPoint,_cornerIdex)!= true)
                            FRIZY_LOG(LOG_ERROR, "fail to turnMove at _bottomLeftCorner" );
                            else
                            FRIZY_LOG(LOG_INFO, "success to turnMove at _bottomLeftCorner" );
                        }
                        else
                        {   auto _cornerIdex_tmp  = cornerIdex::_topRightCorner;
                            if(turnMove(_currentPoint,_cornerIdex_tmp) == true)
                            {
                                FRIZY_LOG(LOG_INFO, "move to the right boundary" );
                            }
                            else
                            {
                                FRIZY_LOG(LOG_ERROR, "fail to move to the right boundary" );
                            }
                        }
                    }

                }
                // 覆盖因为障碍turnround导致未清扫区域
                first_planning_index == true;
                for(int i = _currentPoint.x; i >startPoint.x; i-=cleaning_interval)
                {   
                    auto &p = uTurnPoint.front();
                    if(i == p->x )
                    {
                        // need to do
                        _currentPoint.x = i;
                        uTurnPath.push_back(&_currentPoint);
                        if (robot_orientation ==1)
                        {                        
                            _endPoint.y = p->y;
                        }
                        if (robot_orientation == -1)
                        {
                            _endPoint.y = reselectBlockCorner.topLeftCorner.y ;
                        }
                        uTurnLinePathControl(_currentPoint,_endPoint);

                        uTurnPoint.pop_front();
                    }
                    
                }
                first_planning_index == false;                     
                break;
                }
            case 3:
                {
                FRIZY_LOG(LOG_INFO, "start to plan trajectory at _bottomRightCorner" );  
                robot_orientation = -1;
                // 会存在5-10cm漏扫可能 需要添加test
                for(int i = startPoint.x; i > reselectBlockCorner.topLeftCorner.x; i-=cleaning_interval)
                {
                    _endPoint.x = _currentPoint.x;
                    if (robot_orientation ==1)
                    {                        
                        _endPoint.y = startPoint.y;
                    }
                    if (robot_orientation == -1)
                    {
                        _endPoint.y = reselectBlockCorner.topLeftCorner.y;
                    }
                    uTurnLinePathControl(_currentPoint,_endPoint);
                    if(turnIndex == true)
                    {
                        if((i-cleaning_interval) > reselectBlockCorner.topLeftCorner.x)
                        {
                            FRIZY_LOG(LOG_INFO, "start to turnMove at _bottomRightCorner" );
                            if(turnMove(_currentPoint,_cornerIdex)!= true)
                            FRIZY_LOG(LOG_ERROR, "fail to turnMove at _bottomRightCorner" );
                            else
                            FRIZY_LOG(LOG_INFO, "success to turnMove at _bottomRightCorner" );
                        }
                        else
                        {   auto _cornerIdex_tmp  = cornerIdex::_topRightCorner;
                            if(turnMove(_currentPoint,_cornerIdex_tmp) == true)
                            {
                                FRIZY_LOG(LOG_INFO, "move to the left boundary" );
                            }
                            else
                            {
                                FRIZY_LOG(LOG_ERROR, "fail to move to the left boundary" );
                            }
                        }
                    }

                }
                // 覆盖因为障碍turnround导致未清扫区域
                first_planning_index == true;
                for(int i = _currentPoint.x; i >startPoint.x; i-=cleaning_interval)
                {   
                    auto &p = uTurnPoint.front();
                    if(i == p->x )
                    {
                        // need to do
                        _currentPoint.x = i;
                        uTurnPath.push_back(&_currentPoint);
                        if (robot_orientation ==1)
                        {                        
                            _endPoint.y = p->y;
                        }
                        if (robot_orientation == -1)
                        {
                            _endPoint.y = reselectBlockCorner.topLeftCorner.y ;
                        }
                        uTurnLinePathControl(_currentPoint,_endPoint);

                        uTurnPoint.pop_front();
                    }
                    
                }
                first_planning_index == false;                    
                break;
                }                                            
            default:
                break;
            }

        }

        else if(robotShape ==RobotType::rectangle)
        {
            //need to do
        }
        

    }
    bool UTurnPlanning::uTurnPointIsCanReach(Point* &currentPoint,Point* &targetPoint)
    {
        if(currentPoint->x != targetPoint->x)
        {
            FRIZY_LOG(LOG_ERROR, "currentPoint.x != targetPoint.x at the uTurnPointIsCanReach fuction" );
            return false;
        }
        else
        {
            if(targetPoint->y > currentPoint->y)
            {
                if(_maze.Map[currentPoint->x][currentPoint->y+cleaning_interval+1]->n=0) // 此处可以根据机器实际机型再考虑清楚下
                return true;
            }
            if(targetPoint->y < currentPoint->y)
            {
                if(_maze.Map[currentPoint->x][currentPoint->y-cleaning_interval-1]->n=0) // 此处可以根据机器实际机型再考虑清楚下
                return true;
            }
        }
    }
    // 以Y轴为轴线进行直线巡航
  void UTurnPlanning::uTurnLinePathControl(Point &startPoint,Point &endPoint)
    {
        if (endPoint.y - startPoint.y >0)
            robot_orientation =1 ; // 机器人运动方向Y轴向下
        if (endPoint.y - startPoint.y <0)
            robot_orientation =-1 ;// 机器人运动防线Y轴向上
        for(int j = startPoint.y; j < endPoint.y; j++)
        {

            auto currentPoint = _maze.Map[startPoint.x][j];
            auto targetPoint = _maze.Map[startPoint.x][j+cleaning_interval+1];
            if(uTurnPointIsCanReach(currentPoint,targetPoint))
            {
                _wheel_mode = forward;
                _move_data.rotateangle = 0;
                _move_data.speed = straight_speed;
                _move_data.movepose.i = startPoint.x;
                _move_data.movepose.j = j+robot_orientation;
                // _motioncontrol.wheelControl(_wheel_mode,_move_data);// 这里可以不执行 在上一层接口调用pathTransformtoOrder 来执行
                for(int a = 0; a< int(robot_radius/resolution);a++ )
                {
                    
                    _maze.recordMap[startPoint.x+a][j+robot_orientation]->n = 2;
                    _maze.recordMap[startPoint.x-a][j+robot_orientation]->n = 2;

                } // 标注已经清扫过的区域
                
                _currentPoint.x = startPoint.x ;
                _currentPoint.y = j+robot_orientation;
                uTurnPath.push_back(&_currentPoint);
                turnIndex =true;
            }
            else
            {
                FRIZY_LOG(LOG_INFO, "there is obstacle in targetPoint at int(robot_radius/resolution)+1");
                turnIndex =true;
                if(first_planning_index ==false)
                uTurnPoint.push_front(targetPoint);
                return ; // 无法通行退出循环
            }

        }


    }
    bool UTurnPlanning::turnMove(Point &startPoint,cornerIdex &_cornerIdex)
    {
        if(_cornerIdex == 0 || _cornerIdex == 2) // 从左向右开始清扫
        {
        if(robot_orientation ==1)
        {
            // _wheel_mode = left;
            // _move_data.rotateangle = _Pi/2;
            // _move_data.speed = 0.2;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data); //这里可以不执行 在上一层接口调用pathTransformtoOrder 来执行
            // FRIZY_LOG(LOG_INFO, "turn left done");//是否需要加判断？
            // _wheel_mode = forward ;
            // _move_data.rotateangle = 0.0;
            // _move_data.movepose.i = _currentPoint.x + cleaning_interval;
            // _move_data.movepose.j = _currentPoint.y;
            // _move_data.movepose.forward = 0.0;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data); //这里可以不执行 在上一层接口调用pathTransformtoOrder 来执行

            // _wheel_mode = left;
            // _move_data.rotateangle = _Pi/2;
            // _move_data.speed = 0.2;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data);//是否需要加判断？ //这里可以不执行 在上一层接口调用pathTransformtoOrder 来执行

            _currentPoint.x = _currentPoint.x + cleaning_interval;
            uTurnPath.push_back(&_currentPoint);
            turnIndex =false;
            robot_orientation == -1;
            return true;
        } 
        if(robot_orientation ==-1)
        {
            // _wheel_mode = right;
            // _move_data.rotateangle = _Pi/2;
            // _move_data.speed = 0.2;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data);
            // FRIZY_LOG(LOG_INFO, "turn right done");//是否需要加判断？

            // _wheel_mode = forward ;
            // _move_data.rotateangle = 0.0;
            // _move_data.movepose.i = _currentPoint.x + cleaning_interval;
            // _move_data.movepose.j = _currentPoint.y;
            // _move_data.movepose.forward = 0.0;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data);
            

            // _wheel_mode = right;
            // _move_data.rotateangle = _Pi/2;
            // _move_data.speed = 0.2;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data);

            _currentPoint.x = _currentPoint.x + cleaning_interval;
            uTurnPath.push_back(&_currentPoint);
            robot_orientation == 1;
            turnIndex =false;
            return true;
        }
        }
        if(_cornerIdex == 1 || _cornerIdex == 3) //从右向左清扫
        {
        if(robot_orientation ==1)
        {
            // _wheel_mode = right;
            // _move_data.rotateangle = _Pi/2;
            // _move_data.speed = 0.2;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data);
            // FRIZY_LOG(LOG_INFO, "turn left done");//是否需要加判断？
            // _wheel_mode = forward ;
            // _move_data.rotateangle = 0.0;
            // _move_data.movepose.i = _currentPoint.x - cleaning_interval;
            // _move_data.movepose.j = _currentPoint.y;
            // _move_data.movepose.forward = _Pi;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data); 

            // _wheel_mode = right;
            // _move_data.rotateangle = _Pi/2;
            // _move_data.speed = 0.2;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data);

            _currentPoint.x =_currentPoint.x - cleaning_interval;
            uTurnPath.push_back(&_currentPoint);
            turnIndex =false;
            robot_orientation == -1;
            return true;
        }
        if(robot_orientation ==-1)  
        {
            // _wheel_mode = left;
            // _move_data.rotateangle = _Pi/2;
            // _move_data.speed = 0.2;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data);
            // FRIZY_LOG(LOG_INFO, "turn left done");//是否需要加判断？
            // _wheel_mode = forward ;
            // _move_data.rotateangle = 0.0;
            // _move_data.movepose.i = _currentPoint.x - cleaning_interval;
            // _move_data.movepose.j = _currentPoint.y;
            // _move_data.movepose.forward = _Pi;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data); 

            // _wheel_mode = left;
            // _move_data.rotateangle = _Pi/2;
            // _move_data.speed = 0.2;
            // _motioncontrol.wheelControl(_wheel_mode,_move_data);

            _currentPoint.x = _currentPoint.x - cleaning_interval;
            uTurnPath.push_back(&_currentPoint);
            turnIndex =false;
            robot_orientation == 1;
            return true;
        }        
        }

    }    

}