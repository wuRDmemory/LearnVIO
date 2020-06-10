#pragma once

#include <iostream>
#include <stdio.h>
#if defined(USE_ROS)
#include <ros/ros.h>
#endif

#if defined(USE_ROS)
#define LOGD(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define LOGI(...) ROS_LOG(::ros::console::levels::Info,  ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define LOGW(...) ROS_LOG(::ros::console::levels::Warn,  ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define LOGE(...) ROS_LOG(::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#else
#define LOGD(...) printf(__VA_ARGS__);printf("\n");
#define LOGI(...) printf(__VA_ARGS__);printf("\n");
#define LOGW(...) printf(__VA_ARGS__);printf("\n");
#define LOGE(...) printf(__VA_ARGS__);printf("\n");
#endif

