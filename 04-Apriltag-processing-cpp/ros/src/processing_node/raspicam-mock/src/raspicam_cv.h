#pragma once
#include <ros/ros.h>

#include <mutex>
#include <condition_variable>
#include <thread>

#include <opencv2/core/core.hpp>
//#include <rosbag/bag.h>
//#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>

#include "../../utils.h"

namespace raspicam {
    class RaspiCam_Cv {
    public:
        RaspiCam_Cv();
        void retrieve(cv::Mat &image);
        bool set(int propId, double value);

        ~RaspiCam_Cv();
        bool open();
        bool grab();
        int getHeight();
        int getWidth();

    private:
        void img_comp_callback(const sensor_msgs::CompressedImageConstPtr &img);
        void cam_info_callback(const sensor_msgs::CameraInfo &cam_info_);
        void save_params(const std::string &calibr_file);
        bool fetch();
//        void spinner();

//        rosbag::Bag bag;
//        rosbag::View *view;
//        rosbag::View::iterator view_it;

        concurrent_blocking_queue<cv::Mat> img_queue;
        cv::Mat image;
//        std::mutex m;
//        std::condition_variable c;
        bool image_is_new;

        bool was_cam_info;
        sensor_msgs::CameraInfo cam_info;
        ros::Subscriber raw_img_sub, cam_info_sub;
        std::thread spinner_thread;
    };
}
