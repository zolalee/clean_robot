/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-20 15:06:57
 * @LastEditTime : 2021-07-13 17:03:26
 * @Project      : UM_path_planning
 */
#pragma once
#ifndef PARSE_H
#define PARSE_H

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <common_function/logger.h>

using namespace std;

namespace useerobot
{
    class Parse
    {
    private:
        /* data */
    public:
        Parse(/* args */);
        ~Parse();
        YAML::Node getParseConfig();
        YAML::Node config;
    };
        

}

#endif


