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

                float d0 = nd["d0"];
                float d1 = nd["d1"];
                float d2 = nd["d2"];
                float d3 = nd["d3"];
                float d4 = nd["d4"];

                return new PinholeCamera(width, height, camera_name, fx, fy, cx, cy, d0, d1, d2, d3, d4);
            }
            else {
                return NULL;
            }
        }
    }
};

