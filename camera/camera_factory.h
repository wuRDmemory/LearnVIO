/*
 * @Author: your name
 * @Date: 2020-05-27 21:57:09
 * @LastEditTime: 2020-05-27 22:24:00
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /LearnVIO/camera/camera_factory.h
 */ 
#pragma once

#include "camera_base.h"
#include "pinhole_camera.h"

class CameraFactory {
public:
    static CameraModelPtr buildModuleFromConfigFile(string config_file) {
        if (config_file.empty()) {
            assert(false && "config file is empty!!!!");
        }

        cv::FileStorage fs;
        fs.open(config_file, cv::FileStorage::READ);
        
        if (!fs.isOpened()) {
            assert(false && "can not open config file!!!");
        }


        {   /// read camera node
            cv::FileNode nd = fs["camera"];
            
            int width  = nd["width"];
            int height = nd["height"];

            string camera_name = nd["name"];
            if (camera_name == "pinhole") {
                float fx = nd["fx"];
                float fy = nd["fy"];
                float cx = nd["cx"];
                float cy = nd["cy"];

                float k1 = nd["k1"];
                float k2 = nd["k2"];
                float p1 = nd["p1"];
                float p2 = nd["p2"];
                float k3 = nd["k3"];

                return new PinholeCamera(width, height, camera_name, fx, fy, cx, cy, k1, k2, p1, p2, k3);
            }
            else {
                return NULL;
            }
        }
    }
};

