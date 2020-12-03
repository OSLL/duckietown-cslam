#include "raspicam_cv.h"
#include <iostream>
#include <unistd.h>
//#include "private/private_impl.h"
//#include <opencv2/imgproc.hpp>
//#include "cvversioning.h"
//#include "scaler.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;


namespace raspicam {
    RaspiCam_Cv::RaspiCam_Cv() {
        bag.open("/data/markers.bag");
        view = new rosbag::View(bag);
        view_it = view->begin();
//        cout << "aaaaaaaaaaaaaaaaaaaaaaaaa" << endl;
    }

    RaspiCam_Cv::~RaspiCam_Cv() {
        bag.close();
        delete view;
    }

    void RaspiCam_Cv::retrieve(cv::Mat &img) {
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
        if (view_it == view->end()) {
            return false;
        }
        std::string topic_name = view_it->getTopic();
        if (topic_name == "watchtower02/camera_node/image") {
            try {
                sensor_msgs::ImageConstPtr imgMsgPtr = view_it->instantiate<sensor_msgs::Image>();
                image = cv_bridge::toCvCopy(imgMsgPtr)->image;
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Image convert error");
            }
        }
        ++view_it;
        return true;
    }
}
