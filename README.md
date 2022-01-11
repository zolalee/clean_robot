<!--
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-08 11:41:23
 * @LastEditTime : 2021-07-26 15:11:58
 * @Project      : UM_path_planning
-->
# 扫地机器人规划控制模块

## 使用依赖及编译
used for um LDS clean robot path planning

System requirements:

RM133 Tina system


Build 
cd {your workspace}

mkdir build

cd build
# 编译simulation mode
cmake .. -DSIMULATION=ON

make
# 编译 physical mode
cmake .. -DSIMULATION=OFF

make

## 使用方法
### 架构图
<img alt="Elevation Map Example" src="/doc/frame.jpg" width="700">
Need to add ....

