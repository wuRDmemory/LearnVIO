/*
 * @Author: your name
 * @Date: 2020-05-25 22:46:54
 * @LastEditTime: 2020-05-25 22:53:46
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
    bool is_print_;
    chrono::system_clock::time_point ts_;


public:
    Tick() {
        ts_ = chrono::system_clock::now();
        is_print_ = false;
    }

    ~Tick() {
        if (is_print_) return;

        double dt = \
        chrono::duration_cast<chrono::duration<double>>(chrono::system_clock::now() - ts_).count();
        cout << "[Tick] Time eclipse: " << dt << endl;
    }

    void eclipse() {
        double dt = \
        chrono::duration_cast<chrono::duration<double>>(chrono::system_clock::now() - ts_).count();
        cout << "[Tick] Time eclipse: " << dt << endl;

        is_print_ = true;
    }  
};

