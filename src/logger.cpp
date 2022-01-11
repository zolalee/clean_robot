/*
 * Copyright (C) 2021 Useerobot Ltd.All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2021-05-07 10:39:32
 * @Project      : UM_path_planning
 */


#include "common_function/logger.h"
#include <cstdlib>
#include <ctime>
 
std::ofstream Logger::m_error_log_file;
std::ofstream Logger::m_info_log_file;
std::ofstream Logger::m_warn_log_file;
 
void initLogger(const std::string&info_log_filename,
                const std::string&warn_log_filename,
               const std::string&error_log_filename){
   Logger::m_info_log_file.open(info_log_filename.c_str());
   Logger::m_warn_log_file.open(warn_log_filename.c_str());
   Logger::m_error_log_file.open(error_log_filename.c_str());
}
 
std::ostream& Logger::getStream(log_rank_tlog_rank){
   return (INFO == log_rank) ?
                (m_info_log_file.is_open() ?m_info_log_file : std::cout) :
                (WARNING == log_rank ?
                    (m_warn_log_file.is_open()? m_warn_log_file : std::cerr) :
                    (m_error_log_file.is_open()? m_error_log_file : std::cerr));
}
 
std::ostream& Logger::start(log_rank_tlog_rank,
                            const int32 line,
                            const std::string&function) {
   time_t tm;
   time(&tm);
   char time_string[128];
   ctime_r(&tm, time_string);
   return getStream(log_rank) << time_string
                               << "function (" << function << ")"
                               << "line " << line
                               <<std::flush;
}
 
Logger::~Logger(){
   getStream(m_log_rank) << std::endl << std::flush;
   
   if (FATAL == m_log_rank) {
       m_info_log_file.close();
       m_info_log_file.close();
       m_info_log_file.close();
       abort();
    }
}
