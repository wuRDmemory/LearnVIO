/*
 * @Author: your name
 * @Date: 2020-05-24 17:32:19
 * @LastEditTime: 2020-05-27 22:31:38
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /LearnVIO/include/LearnVIO/config.h
 */ 
#pragma once

#include <ros/ros.h>

#include <iostream>
#include <vector>

using namespace std;

extern string CONFIG_PATH;
extern string IMAGE_TOPIC;
extern string IMU_TOPIC;
extern vector<string> CAM_NAMES;
extern string FISHEYE_MASK;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

void readParameters(ros::NodeHandle& n);
