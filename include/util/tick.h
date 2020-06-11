/*
 * @Author: your name
 * @Date: 2020-05-25 22:46:54
 * @LastEditTime: 2020-05-27 22:41:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /LearnVIO/include/tick.h
 */ 
#pragma once

#include <iostream>
#include <chrono>

using namespace std;
using namespace chrono;

class Tick{
private:
    chrono::system_clock::time_point ts_;


public:
    Tick() {
        ts_ = chrono::system_clock::now();
    }

    double delta_time() {
        double dt = \
        chrono::duration_cast<chrono::duration<double>>(chrono::system_clock::now() - ts_).count();
        return dt; 
    }  
};

