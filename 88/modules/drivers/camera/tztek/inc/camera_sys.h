/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_sys.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAMERA_SYS_H__
#define __CAMERA_SYS_H__

#include <stdint.h>
#include <time.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def CAMERA_TIME_CAL_UNIT_S2MS
 * calculate unit s to ms
 */
#define CAMERA_TIME_CAL_UNIT_S2MS	(1000U)
/**
 * @def CAMERA_TIME_CAL_UNIT_MS2US
 * calculate unit ms to us
 */
#define CAMERA_TIME_CAL_UNIT_MS2US	(1000U)
/**
 * @def CAMERA_TIME_CAL_UNIT_S2US
 * calculate unit s to us
 */
#define CAMERA_TIME_CAL_UNIT_S2US	(1000000U)
/**
 * @def CAMERA_TIME_CAL_UNIT_S2NS
 * calculate unit s to us
 */
#define CAMERA_TIME_CAL_UNIT_S2NS	(1000000000U)
/**
 * @def CAMERA_USE_MS
 * calculate use_us unit as us to ms
 */
#define CAMERA_USE_MS(us)		((us) / CAMERA_TIME_CAL_UNIT_MS2US)
/**
 * @def CAMERA_USE_RUS
 * calculate use_us unit as us to remained us
 */
#define CAMERA_USE_RUS(us)		((us) % CAMERA_TIME_CAL_UNIT_MS2US)

/**
 * @def camera_mutex_t
 * the mutex type define in camera sys
 */
#define camera_mutex_t		pthread_mutex_t
/**
 * @def camera_mutexattr_t
 * the mutex type define in camera sys
 */
#define camera_mutexattr_t	pthread_mutexattr_t
/**
 * @def CAMERA_MUTEX_INITIALIZER
 * the mutex initializer macro in camera sys
 */
#define CAMERA_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER

#define camera_mutex_init(m, a) pthread_mutex_init(m, a)
#define camera_mutex_destroy(m) pthread_mutex_destroy(m)
#define camera_mutex_lock(m)	pthread_mutex_lock(m)
#define camera_mutex_unlock(m)	pthread_mutex_unlock(m)

/**
 * @def camera_pthread_t
 * the pthread id type define in camera sys
 */
#include <pthread.h>

typedef unsigned long int  pthread_t;
#define camera_pthread_t        pthread_t

#define camera_pthread_create(id, attr, func, arg) \
                                pthread_create(id, attr, func, arg)
#define camera_pthread_join(id, res) \
				pthread_join(id, res)
#define camera_pthread_cancel(id) \
				pthread_cancel(id)
#define camera_pthread_exit(ret) \
				pthread_exit(ret)
#ifdef HOBOT_MCU_CAMSYS
#include "osal_time.h"

#define usleep(s)			osal_usleep(s)
#define camera_prctl(op, ...)
#define camera_sys_get_board_id()	({int32_t __ret = 0; ret;})
#define camera_sys_gettime_ts(ts)	({int32_t __ret = 0; ret;})
#define camera_sys_gettime_us()		({int64_t __ret = 0; ret;})
#define camera_sys_gettid()		({int64_t __ret = 0; ret;})
#define camera_sys_getpid()		({int64_t __ret = 0; ret;})
#else /* HOBOT_MCU_CAMSYS */
#include <unistd.h>
#include <sys/time.h>
#include <sys/prctl.h>

#define camera_prctl(op, ...)		prctl(op, ##__VA_ARGS__)
#define camera_sys_gettid()		pthread_self()
#define camera_sys_getpid()		getpid()

typedef enum
{
	GEAC90 = 0,
	GEAC91 = 1,
	GEAC91V = 2,
	GEAC91VP = 3,
	GEACJX = 4,
	GEACSC = 5,
	GEACN2_3 = 6,
	T24DG26TYA_ORIN = 7,
	T24DG26TYA_NX = 8,
	T24DG26TYB_ORIN = 9,
	T24DG26TYB_NX = 10,
	C24HH01 = 11    // AGI-C24HH01
} DEVICE_TYPE;

extern int32_t camera_sys_get_board_id(void);
extern int32_t camera_sys_gettime_ts(struct timespec *ts);
extern uint64_t camera_sys_gettime_us(void);
extern uint64_t camera_sys_gettime_ms_local(char *buffer);
extern int vin_get_master(void);
#endif /* HOBOT_MCU_CAMSYS */

#define camera_sys_usleep(s)	usleep(s)
#define camera_sys_sleep(s)	camera_sys_usleep((s) * CAMERA_TIME_CAL_UNIT_S2US)
#define camera_sys_msleep(ms)	camera_sys_usleep((ms) * CAMERA_TIME_CAL_UNIT_MS2US)

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_SYS_H__ */
