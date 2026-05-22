/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef J5_VIN_UTILS_HB_UTILS_H_
#define J5_VIN_UTILS_HB_UTILS_H_
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdarg.h>

typedef enum vin_log_level {
	VIN_NONE = 0,
	VIN_ERR,
	VIN_WARN,
	VIN_INFO,
	VIN_DEBUG,
	MAX_VIN_LOG_LEVEL
} vin_log_level_e;

// #define RS_ERROR   std::cout << "\033[1m\033[31m"  // bold red
// #define RS_WARNING std::cout << "\033[1m\033[33m"  // bold yellow
// #define RS_INFO    std::cout << "\033[1m\033[32m"  // bold green
// #define RS_INFOL   std::cout << "\033[32m"         // green
// #define RS_DEBUG   std::cout << "\033[1m\033[36m"  // bold cyan
// #define RS_REND    "\033[0m" << std::endl

// #define RS_TITLE   std::cout << "\033[1m\033[35m"  // bold magenta
// #define RS_MSG     std::cout << "\x1b[0m"  // bold white

#define ANSI_COLOR_RESET   "\x1b[0m"

// #define debug_err(fmt, ...)		printf("\033[1m\033[31m%s(%u) %s[ERR] " fmt ANSI_COLOR_RESET,__FILE__,__LINE__,__FUNCTION__,##__VA_ARGS__);	 /*PRQA S 0342,1038*/
// #define debug_warn(fmt, ...)		printf("\033[1m\033[33m%s(%u) %s[WARN] " fmt ANSI_COLOR_RESET,__FILE__,__LINE__,__FUNCTION__,##__VA_ARGS__)  /*PRQA S 0342,1038*/
// #define debug_dbg(fmt, ...)		printf("\033[1m\033[36m%s(%u) %s[DEBUG] " fmt ANSI_COLOR_RESET,__FILE__,__LINE__,__FUNCTION__,##__VA_ARGS__)  /*PRQA S 0342,1038*/
// #define debug_info(fmt, ...)		printf("\033[1m\033[32m%s(%u) %s[INFO] " fmt ANSI_COLOR_RESET,__FILE__,__LINE__,__FUNCTION__,##__VA_ARGS__)  /*PRQA S 0342,1038*/
#define debug_err(fmt, ...)	 do{ \
    struct timeval tv_cam; \
    struct tm* ptm; \
    char time_string[40]; \
    long milliseconds; \
    gettimeofday(&tv_cam, NULL); \
    ptm = localtime(&tv_cam.tv_sec); \
    strftime(time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", ptm); \
    milliseconds = tv_cam.tv_usec / 1000; \
	printf("\033[1m\033[31m[%s.%03ld] %s(%u) %s[ERR] " fmt ANSI_COLOR_RESET,time_string,milliseconds,__FILE__,__LINE__,__FUNCTION__,##__VA_ARGS__); \
}while (0)
#define debug_warn(fmt, ...)	 do{ \
    struct timeval tv_cam; \
    struct tm* ptm; \
    char time_string[40]; \
    long milliseconds; \
    gettimeofday(&tv_cam, NULL); \
    ptm = localtime(&tv_cam.tv_sec); \
    strftime(time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", ptm); \
    milliseconds = tv_cam.tv_usec / 1000; \
	printf("\033[1m\033[33m[%s.%03ld] %s(%u) %s[WARN] " fmt ANSI_COLOR_RESET,time_string,milliseconds,__FILE__,__LINE__,__FUNCTION__,##__VA_ARGS__); \
}while (0)
#define debug_dbg(fmt, ...)	 do{ \
    struct timeval tv_cam; \
    struct tm* ptm; \
    char time_string[40]; \
    long milliseconds; \
    gettimeofday(&tv_cam, NULL); \
    ptm = localtime(&tv_cam.tv_sec); \
    strftime(time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", ptm); \
    milliseconds = tv_cam.tv_usec / 1000; \
	printf("\033[1m\033[36m[%s.%03ld] %s(%u) %s[DEBUG] " fmt ANSI_COLOR_RESET,time_string,milliseconds,__FILE__,__LINE__,__FUNCTION__,##__VA_ARGS__); \
}while (0)
#define debug_info(fmt, ...)	 do{ \
    struct timeval tv_cam; \
    struct tm* ptm; \
    char time_string[40]; \
    long milliseconds; \
    gettimeofday(&tv_cam, NULL); \
    ptm = localtime(&tv_cam.tv_sec); \
    strftime(time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", ptm); \
    milliseconds = tv_cam.tv_usec / 1000; \
	printf("\033[1m\033[32m[%s.%03ld] %s(%u) %s[INFO] " fmt ANSI_COLOR_RESET,time_string,milliseconds,__FILE__,__LINE__,__FUNCTION__,##__VA_ARGS__); \
}while (0)
#define debug_crit(fmt, ...)	 do{ \
    struct timeval tv_cam; \
    struct tm* ptm; \
    char time_string[40]; \
    long milliseconds; \
    gettimeofday(&tv_cam, NULL); \
    ptm = localtime(&tv_cam.tv_sec); \
    strftime(time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", ptm); \
    milliseconds = tv_cam.tv_usec / 1000; \
	printf("\033[1m\033[35m[%s.%03ld] %s(%u) %s[CRIT] " fmt ANSI_COLOR_RESET,time_string,milliseconds,__FILE__,__LINE__,__FUNCTION__,##__VA_ARGS__); \
}while (0)



#endif //J5_VIN_UTILS_HB_UTILS_H_
