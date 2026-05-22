
#pragma once

#ifdef _WIN32

#include "modules/drivers/lidar/robosense/rs_driver/src/rs_driver/driver/input/win/input_sock_select.hpp"

#else  //__linux__

#ifdef ENABLE_EPOLL_RECEIVE
#include "modules/drivers/lidar/robosense/rs_driver/src/rs_driver/driver/input/unix/input_sock_epoll.hpp"
#else
#include "modules/drivers/lidar/robosense/rs_driver/src/rs_driver/driver/input/unix/input_sock_select.hpp"
#endif

#endif

