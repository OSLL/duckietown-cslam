#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "duckietown_msgs/AprilTagExtended.h"
#include "aruco.h"
#include "dcf/dcfmarkertracker.h"

using namespace std;

bool was_cam_info = false;
sensor_msgs::CameraInfo cam_info;
ros::Publisher shift_cmd_pub;


class IArucoDetector {
public:
    virtual vector<Marker> set_params_detect_and_estimate(const cv::Mat &image, const CameraParameters &cam_params) = 0;

    virtual ~IArucoDetector() = default;
};

struct TimerAvrg {
    std::vector<double> times;
    size_t curr = 0, n;
    std::chrono::high_resolution_clock::time_point begin, end;

    TimerAvrg(int _n = 100) {
        n = _n;
        times.reserve(n);
    }

    inline void start() { begin = std::chrono::high_resolution_clock::now(); }

    inline void stop() {
        end = std::chrono::high_resolution_clock::now();
        double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) * 1e-6;
        if (times.size() < n) times.push_back(duration);
        else {
            times[curr] = duration;
            curr++;
            if (curr >= times.size()) curr = 0;
        }
    }

    double getAvrg() {
        double sum = 0;
        for (auto t:times) sum += t;
        return sum / double(times.size());
    }
};

TimerAvrg timer;

class DFCArucoDetector : public IArucoDetector {
public:
    DFCArucoDetector(float marker_size, const string &config_file_string) :
            marker_size(marker_size) {
        marker_tracker.loadParamsFromFile(config_file_string);
    }

    vector<Marker> set_params_detect_and_estimate(const cv::Mat &image, const CameraParameters &cam_params) override {
        marker_tracker.setParams(cam_params, marker_size);
        map<int, cv::Ptr<TrackerImpl>> set_trackers = marker_tracker.track(image, 0.1);
        marker_tracker.estimatePose();
        vector<Marker> markers_vec(set_trackers.size());
        int i = 0;
        for (const auto &t : set_trackers) {
            markers_vec[i] = t.second->getMarker();
            i++;
        }
        return markers_vec;
    }

    ~DFCArucoDetector() override = default;

private:
    DFCMarkerTracker marker_tracker;
    float marker_size;
};

class STDArucoDetector : public IArucoDetector {
public:
    STDArucoDetector(float marker_size, const string &config_file_string) :
            marker_size(marker_size) {
        marker_detector.loadParamsFromFile(config_file_string);

        MarkerDetector::Params params = marker_detector.getParameters();
        marker_detector.setDetectionMode(params.detectMode, params.minSize);
        params.setCornerRefinementMethod(params.cornerRefinementM);
    }

    vector<Marker> set_params_detect_and_estimate(const cv::Mat &image, const CameraParameters &cam_params) override {
        return marker_detector.detect(image, cam_params, marker_size);
    }

    ~STDArucoDetector() override = default;

private:
    MarkerDetector marker_detector;
    float marker_size;
};


IArucoDetector *aruco_detector = nullptr;



void cam_info_callback(const sensor_msgs::CameraInfo &cam_info_) {
    was_cam_info = true;
    cam_info = cam_info_;
}

void img_raw_callback(const sensor_msgs::Image &img) {
    if (!was_cam_info) {
        return;
    }
    timer.start();

    cv::Size image_size;
    image_size.height = cam_info.height;
    image_size.width = cam_info.width;

    const cv::Mat image(img.height, img.width, CV_8UC3, const_cast<uint8_t *>(&img.data[0]), img.step);


    cv::Mat camera_matrix(3, 3, CV_32F);
    camera_matrix.at<float>(0, 0) = cam_info.K[0];
    camera_matrix.at<float>(0, 2) = cam_info.K[2];
    camera_matrix.at<float>(1, 1) = cam_info.K[4];
    camera_matrix.at<float>(1, 2) = cam_info.K[5];

    cv::Mat distortion(4, 1, CV_32F);
    distortion.at<float>(0, 0) = cam_info.D[0];
    distortion.at<float>(1, 0) = cam_info.D[1];
    distortion.at<float>(2, 0) = cam_info.D[2];
    distortion.at<float>(3, 0) = cam_info.D[3];

    CameraParameters cam_params;
    try {
        cam_params = CameraParameters(camera_matrix, distortion, image_size);
    } catch (cv::Exception &e) {
        printf("\nFailed: [%s]\n", e.msg.c_str());
    } catch (std::exception &e) {
        printf("\nFailed: [%s]\n", e.what());
    }

    vector<Marker> markers_vector = aruco_detector->set_params_detect_and_estimate(image, cam_params);

    timer.stop();
    cout << "                                                 " << timer.getAvrg() * 1000 << endl;
}

string getenv(const char *variable_name, const char *default_value) {
    const char* value = getenv(variable_name);
    return value ? value : default_value;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "apriltag_processor_node");
    ros::NodeHandle node_handle("~");

    string device_name    = getenv("ACQ_DEVICE_NAME",      "watchtower10");
    string img_topic_suf  = getenv("ACQ_TOPIC_RAW",        "camera_node/image");
    string info_topic_suf = getenv("ACQ_TOPIC_CAMERAINFO", "camera_node/camera_info");
    string pose_topic_suf = getenv("ACQ_POSES_TOPIC",      "poses");
    string config_file    = getenv("ACQ_CONFIG_YML",       "config.yml");
    string detector_type  = getenv("ACQ_DETECTOR_TYPE",    "DFC");
    float  marker_size    = stof(getenv("ACQ_TAG_SIZE",    "0.065"));

    string img_raw_topic  = "/" + device_name + "/" + img_topic_suf;
    string cam_info_topic = "/" + device_name + "/" + info_topic_suf;
    string tag_pose_topic = "/poses_acquisition/" + pose_topic_suf;

    ros::Subscriber comp_img_pub = node_handle.subscribe(img_raw_topic, 20, img_raw_callback);
    ros::Subscriber cam_info_sub = node_handle.subscribe(cam_info_topic, 1, cam_info_callback);

    shift_cmd_pub = node_handle.advertise<duckietown_msgs::AprilTagExtended>(tag_pose_topic, 20);

    if (detector_type == "DFC") {
        aruco_detector = new DFCArucoDetector(marker_size, config_file);
    } else if (detector_type == "STD") {
        aruco_detector = new STDArucoDetector(marker_size, config_file);
    } else {
        printf("Unknown detector type: %s, use 'DFC' or 'STD'\n", detector_type.c_str());
        return EXIT_FAILURE;
    }

    ros::spin();
    delete aruco_detector;
}