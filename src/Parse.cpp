/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-20 15:08:21
 * @LastEditTime : 2021-07-27 18:03:32
 * @Project      : UM_path_planning
 */

#include "common_function/Parse.h"

namespace useerobot
{
    Parse::Parse(/* args */)
    {
        
    }
    
    Parse::~Parse()
    {
    }
    YAML::Node Parse::getParseConfig()
    {
        try
        {
            config = YAML::LoadFile("/mnt/app/config/planning_config.yaml");
            return config;
        }
        
        catch(std::exception &e){
        FRIZY_LOG(LOG_ERROR, "Failed to load yaml file ");}
    }
}

