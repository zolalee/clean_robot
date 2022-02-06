/*
 * Copyright (C) 2021 Useerobot. All rights reserved.
 * @Author       : Zola
 * @Description  : 
 * @Date         : 2021-05-06 17:14:52
 * @LastEditTime : 2022-01-26 15:00:20
 * @Project      : UM_path_planning
 */

#pragma once
#ifndef FRIZY_LOG_PRINT_H
#define FRIZY_LOG_PRINT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#ifdef _WIN32
#define TrimFilePath(x) strrchr(x, '\\') ? strrchr(x, '\\') + 1 : x
#else  // Linux/Unix
#define TrimFilePath(x) strrchr(x, '/') ? strrchr(x, '/') + 1 : x
#endif

//#define LogTime

#define LogColorNone_			"\033[0m"
#define LogColorRed_				"\033[0;31m"
#define LogColorLightRed_		"\033[1;31m"
#define LogColorYellow_			"\033[0;33m"
#define LogColorLightYellow_		"\033[1;33m"
#define LogColorGreen_			"\033[0;32m"
#define LogColorLightGreen_		"\033[1;32m"

#define ColorfulPrint_(fmt, ...) \
	do { \
        printf(fmt , ##__VA_ARGS__); \
	} while(0)

#ifndef LogTime
#define PrintLog_(color, fmt, ...) \
	do { \
        ColorfulPrint_(\
			 "[UM_PLANNING_Node_Log| %s(%d)]: " fmt "\n", TrimFilePath(__FILE__), __LINE__, ##__VA_ARGS__); \
	} while(0)
#else
#define PrintLog_(color, fmt, ...) \
	do { \
		time_t t; time(&t); \
		struct tm *pTm = localtime(&t); \
        ColorfulPrint_(color, \
			"[%02d:%02d:%02d| FRIZYLog: %s(%d)]: " fmt "\n", \
			pTm->tm_hour, pTm->tm_min, pTm->tm_sec, __FILE__, __LINE__, ##__VA_ARGS__); \
	} while(0)
#endif

#ifndef FRIZY_LOG_LEVEL
    #define FRIZY_LOG_LEVEL 0
#endif

#define LOG_DEBUG       0
#define LOG_INFO        1
#define LOG_WARNING     2
#define LOG_ERROR       3


#define PrintDbg_(fmt, ...) PrintLog_(LogColorNone_, fmt, ##__VA_ARGS__)
#define PrintInfo_(fmt, ...) PrintLog_(LogColorLightGreen_, fmt, ##__VA_ARGS__)
#define PrintWarn_(fmt, ...) PrintLog_(LogColorLightYellow_, fmt, ##__VA_ARGS__)
#define PrintErr_(fmt, ...) PrintLog_(LogColorLightRed_, fmt, ##__VA_ARGS__)



#define FRIZY_LOG(level, fmt, ...) do {                      \
    if (level == LOG_DEBUG) { if(LOG_DEBUG >= FRIZY_LOG_LEVEL) {PrintDbg_(fmt, ##__VA_ARGS__);}            \
    } else if (level == LOG_INFO) { if(LOG_INFO >= FRIZY_LOG_LEVEL) {PrintInfo_(fmt, ##__VA_ARGS__);}            \
    } else if (level == LOG_WARNING) { if(LOG_WARNING >= FRIZY_LOG_LEVEL) {PrintWarn_(fmt, ##__VA_ARGS__);}            \
    } else if (level == LOG_ERROR) { if(LOG_ERROR >= FRIZY_LOG_LEVEL) {PrintErr_(fmt, ##__VA_ARGS__);}            \
    } else {                                                     \
    }                                                            \
} while (0)


#ifdef NDEBUG
#define Assert(EXPRESSION) ((void)0)
#else
#define Assert(EXPRESSION) do { \
    if (EXPRESSION) { \
        ((void)0); \
    } else { \
        (_assert(#EXPRESSION, TrimFilePath(__FILE__), __LINE__)); \
    } \
} while (0)
#endif


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif