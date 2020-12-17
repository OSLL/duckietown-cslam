#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <unistd.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/yaml.h>

#include "raspicam_cv.h"

using namespace std;

namespace raspicam {

    void RaspiCam_Cv::save_params(const string &calibr_file) {
        std::vector<double> K(9);
        for (int i = 0; i < 9; i++) {
            K[i] = cam_info.K[i];
        }
        std::vector<double> D = cam_info.D;

        YAML::Emitter out;
        out <<
            YAML::BeginMap <<
            YAML::Key << "camera_matrix" << YAML::Value <<
            YAML::BeginMap <<
            YAML::Key << "cols" << YAML::Value << 3 <<
            YAML::Key << "data" << YAML::Value << YAML::Flow << K <<
            YAML::Key << "rows" << YAML::Value << 3 <<
            YAML::EndMap <<
            YAML::Key << "distortion_coefficients" << YAML::Value <<
            YAML::BeginMap <<
            YAML::Key << "cols" << YAML::Value << D.size() <<
            YAML::Key << "data" << YAML::Value << YAML::Flow << D <<
            YAML::Key << "rows" << YAML::Value << 1 <<
            YAML::EndMap <<
            YAML::EndMap;

        ofstream fout(calibr_file);
        fout << out.c_str();
        fout.close();
        cout << "================ file dump ========================\n"
             << out.c_str() << "\n"
             << "-------------------------------------------------\n"
             << "width = " << cam_info.width << "  height = " << cam_info.height << "\n"
             << "=================================================" << endl;
    }

    void RaspiCam_Cv::cam_info_callback(const sensor_msgs::CameraInfo &cam_info_) {
        cout << "cam_info_callback" << endl;
        was_cam_info = true;
        cam_info = cam_info_;
    }

    void RaspiCam_Cv::img_comp_callback(const sensor_msgs::CompressedImageConstPtr &img) {
        cout << "img_comp_callback     " << img_queue.size() << endl;
        int rows = 1;
        int cols = (int) (cam_info.height * cam_info.width);
        cv::Mat buf(rows, cols, CV_8U, const_cast<uint8_t *>(img->data.data()));
        cv::Mat color_image = cv::imdecode(buf, cv::IMREAD_COLOR);
        cv::Mat mono_image;
        cv::cvtColor(color_image, image, cv::COLOR_BGR2GRAY);

        image_is_new = true;
        img_queue.push(move(mono_image));
    }

    void spinner() {
        ros::spin();
    }

    RaspiCam_Cv::RaspiCam_Cv() :
            image_is_new(false), was_cam_info(false) {
        ros::NodeHandle node_handle("~");

        string device_name    = getenv("ACQ_DEVICE_NAME",      "watchtower10");
        string img_topic_suf  = getenv("ACQ_TOPIC_RAW",        "camera_node/image");
        string info_topic_suf = getenv("ACQ_TOPIC_CAMERAINFO", "camera_node/camera_info");
        string calibr_file    = getenv("ACQ_CALIBR_YAML",      "calibr.yaml");

        string img_raw_topic  = "/" + device_name + "/" + img_topic_suf;
        string cam_info_topic = "/" + device_name + "/" + info_topic_suf;

        raw_img_sub  = node_handle.subscribe(img_raw_topic,  1000, &RaspiCam_Cv::img_comp_callback, this);
        cam_info_sub = node_handle.subscribe(cam_info_topic, 1,    &RaspiCam_Cv::cam_info_callback, this);

        ros::Rate rate(10);
        while (!was_cam_info && ros::ok()) {
            cout << "wait cam_info" << endl;
            ros::spinOnce();
            rate.sleep();
        }
        save_params(calibr_file);
        spinner_thread = thread(spinner);
    }

    RaspiCam_Cv::~RaspiCam_Cv() {
        img_queue.close();
        spinner_thread.join();
    }

    void RaspiCam_Cv::retrieve(cv::Mat &img) {
        if (!image_is_new) {
            throw runtime_error("retrieve old image");
        }
        image.copyTo(img);
    }

    bool RaspiCam_Cv::set(int, double) {
        return true;
    }

    bool RaspiCam_Cv::open() {
        return fetch();
    }

    bool RaspiCam_Cv::grab() {
        return fetch();
    }

    int RaspiCam_Cv::getHeight() {
        return image.rows;
    }

    int RaspiCam_Cv::getWidth() {
        return image.cols;
    }

    bool RaspiCam_Cv::fetch() {
        cout << "fetch" << endl;
        ros::spinOnce();
        bool ret = img_queue.pop(image);
        cout << "popped" << endl;
        return ret;
    }
}

//namespace raspicam {
//
//    void RaspiCam_Cv::img_raw_callback(const sensor_msgs::ImageConstPtr &img) {
//        unique_lock<mutex> lock(m);
//
//        if (image_is_new) {
//            throw runtime_error("receive a new image before processing the old one");
//        }
//
//        bool mono = img->encoding == sensor_msgs::image_encodings::MONO8 ||
//                    img->encoding == sensor_msgs::image_encodings::TYPE_8UC1;
//        int type = mono ? CV_8UC1 : CV_8UC3;
//
//        const cv::Mat tmp(img->height, img->width, type, const_cast<uint8_t *>(img->data.data()), img->step);
//
//        if (!mono) {
//            cv::cvtColor(tmp, image, cv::COLOR_BGR2GRAY);
//        } else {
//            tmp.copyTo(image);
//        }
//
//        image_is_new = true;
//    }
//
//    RaspiCam_Cv::RaspiCam_Cv() {
//        ros::NodeHandle node_handle("~");
//        raw_img_sub = node_handle.subscribe("img_top", 1000, &RaspiCam_Cv::img_raw_callback, this);
//        image_is_new = false;
//    }
//
//    RaspiCam_Cv::~RaspiCam_Cv() {
//    }
//
//    void RaspiCam_Cv::retrieve(cv::Mat &img) {
//        image.copyTo(img);
//    }
//
//    bool RaspiCam_Cv::set(int, double) {
//        return true;
//    }
//
//    bool RaspiCam_Cv::open() {
//        return fetch();
//    }
//
//    bool RaspiCam_Cv::grab() {
//        return fetch();
//    }
//
//    int RaspiCam_Cv::getHeight() {
//        return image.rows;
//    }
//
//    int RaspiCam_Cv::getWidth() {
//        return image.cols;
//    }
//
//    bool RaspiCam_Cv::fetch() {
//        unique_lock<mutex> lock(m);
//        while (!image_is_new) {
//            c.wait(lock);
//        }
//        return true;
//    }
//}


//namespace raspicam {
//    RaspiCam_Cv::RaspiCam_Cv() {
//        bag.open("/data/markers.bag");
//        view = new rosbag::View(bag);
//        view_it = view->begin();
////        cout << "aaaaaaaaaaaaaaaaaaaaaaaaa" << endl;
//    }
//
//    RaspiCam_Cv::~RaspiCam_Cv() {
//        bag.close();
//        delete view;
//    }
//
//    void RaspiCam_Cv::retrieve(cv::Mat &img) {
//         image.copyTo(img);
//    }
//
//    bool RaspiCam_Cv::set(int, double) {
//        return true;
//    }
//
//    bool RaspiCam_Cv::open() {
//        return fetch();
//    }
//
//    bool RaspiCam_Cv::grab() {
//        return fetch();
//    }
//
//    int RaspiCam_Cv::getHeight() {
//        return image.rows;
//    }
//
//    int RaspiCam_Cv::getWidth() {
//        return image.cols;
//    }
//
//    bool RaspiCam_Cv::fetch() {
//        if (view_it == view->end()) {
//            return false;
//        }
//        std::string topic_name = view_it->getTopic();
//        if (topic_name == "watchtower02/camera_node/image") {
//            try {
//                sensor_msgs::ImageConstPtr imgMsgPtr = view_it->instantiate<sensor_msgs::Image>();
//                image = cv_bridge::toCvCopy(imgMsgPtr)->image;
//            } catch (cv_bridge::Exception& e) {
//                ROS_ERROR("Image convert error");
//            }
//        }
//        ++view_it;
//        return true;
//    }
//}