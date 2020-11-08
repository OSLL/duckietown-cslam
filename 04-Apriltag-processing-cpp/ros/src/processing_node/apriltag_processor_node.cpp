#include <ros/ros.h>

#include <chrono>
#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <boost/lexical_cast.hpp>
#include <opencv2/core.hpp>
#include <raspicam_cv.h>
#include <cvversioning.h>
#include <yaml-cpp/yaml.h>

#include "duckietown_msgs/AprilTagExtended.h"
#include "aruco.h"
#include "dcf/dcfmarkertracker.h"

//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/Image.h>
//#include <boost/thread/thread.hpp>
//#include <boost/lockfree/queue.hpp>
//#include <sensor_msgs/image_encodings.h>
//#include <ctime>


using namespace std;

class timer_avg {
public:
    explicit timer_avg(int n = 100) :
            n(n) {
        for (int i = 0; i < n; i++) {
            times.push(chrono::high_resolution_clock::now());
        }
    }

    inline void tick() {
        times.push(chrono::high_resolution_clock::now());
        times.pop();
    }

    double avg() {
        return chrono::duration_cast<chrono::microseconds>(times.back() - times.front()).count() * 1e-6 / n;
    }

private:
    queue<chrono::high_resolution_clock::time_point> times;
    size_t n;
};

template<typename T>
class concurrent_blocking_queue {
public:
    concurrent_blocking_queue() {
        closed = false;
    }

    bool pop(T &item) {
//        cout << "pop lock" << endl;
        unique_lock<mutex> lock(m);
//        cout << "pop unlock" << endl;
        while (q.empty() && !closed) {
//            cout << "pop wait" << endl;
            c.wait(lock);
        }
//        cout << "pop get" << endl;
        if (closed) {
            return false;
        }
        item = q.front();
        q.pop();
        return true;
    }

    void push(const T &item) {
//        cout << "push lock" << endl;
        unique_lock<mutex> lock(m);
//        cout << "push unlock" << endl;
        q.push(item);
        lock.unlock();
        c.notify_one();
    }

    void push(T &&item) {
//        cout << "push lock" << endl;
        unique_lock<mutex> lock(m);
//        cout << "push unlock" << endl;
        q.push(move(item));
        lock.unlock();
        c.notify_one();
    }

    void close() {
        closed = true;
        c.notify_one();
    }

    int size() {
        unique_lock<mutex> lock(m);
        return q.size();
    }

private:
    queue<T> q;
    mutex m;
    condition_variable c;
    bool closed;
};

struct tag_data {
    int tag_id;
    string tag_family;
    vector<cv::Point> corners;
    cv::Mat tvec, rvec;
};

struct marker_tracker_params {
    int height, width;
    string calibr_file, config_file;
    float marker_size;
};

CameraParameters load_params(unsigned int height, unsigned int width, const string &calibr_file) {
    cv::Size image_size;
    image_size.height = height;
    image_size.width = width;

    YAML::Node root = YAML::LoadFile(calibr_file);
    auto camera_matrix_yaml = root["camera_matrix"];
    int cols = camera_matrix_yaml["cols"].as<int>();
    int rows = camera_matrix_yaml["rows"].as<int>();
    cv::Mat camera_matrix(rows, cols, CV_32F);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            camera_matrix.at<float>(i, j) = camera_matrix_yaml["data"][i * cols + j].as<float>();
        }
    }

    auto distortion_yaml = root["distortion_coefficients"];
    cols = distortion_yaml["cols"].as<int>();
    cv::Mat distortion(cols, 1, CV_32F);
    for (int j = 0; j < cols; j++) {
        distortion.at<float>(j, 0) = distortion_yaml["data"][j].as<float>();
    }

    return CameraParameters(camera_matrix, distortion, image_size);
}

void img_processor(const marker_tracker_params& params,
                   concurrent_blocking_queue<cv::Mat> &img_queue,
                   concurrent_blocking_queue<tag_data> &tag_queue) {
    DFCMarkerTracker marker_tracker;
    CameraParameters cam_params = load_params(params.height, params.width, params.calibr_file);
    marker_tracker.setParams(cam_params, params.marker_size);
    marker_tracker.loadParamsFromFile(params.config_file);
    cv::Mat image;
    while (img_queue.pop(image)) {
//        sleep(1);
        map<int, cv::Ptr<TrackerImpl>> set_trackers = marker_tracker.track(image, 0.1);
        marker_tracker.estimatePose();
        vector<Marker> markers_vec(set_trackers.size());
        int i = 0;
        for (const auto &t : set_trackers) {
            Marker &marker = t.second->getMarker();
            i++;

            tag_data tag;
            tag.tag_id = marker.id;
            tag.tag_family = marker.dict_info;
            tag.tvec = marker.Tvec;
            tag.rvec = marker.Rvec;
            for (int j = 0; j < marker.size(); j++) {
                tag.corners[j] = marker[j];
            }
            tag_queue.push(tag);
        }
    }
}

geometry_msgs::Quaternion rvec2quat(cv::Mat rvec) {
    geometry_msgs::Quaternion quat;
    float x = rvec.at<float>(0);
    float y = rvec.at<float>(1);
    float z = rvec.at<float>(2);
    float r = sqrt(x * x + y * y + z * z);
    if (r < 0.00001) {
        quat.x = 1;
        return quat;
    }
    float c = cos(r / 2);
    float s = sin(r / 2);
    quat.x = c;
    quat.y = s * z / r;
    quat.z = -s * y / r;
    quat.w = -s * x / r;
    return quat;
}

string getenv(const char *variable_name, const char *default_value) {
    const char* value = getenv(variable_name);
    return value ? value : default_value;
}

template<typename T>
T getenv(const char *variable_name, T default_value) {
    const char* value = getenv(variable_name);
    return value ? boost::lexical_cast<T>(value) : default_value;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "apriltag_processor_node");
    ros::NodeHandle node_handle("~");

    string device_name    = getenv("ACQ_DEVICE_NAME",      "watchtower10");
//    string img_topic_suf  = getenv("ACQ_TOPIC_RAW",        "camera_node/image");
//    string info_topic_suf = getenv("ACQ_TOPIC_CAMERAINFO", "camera_node/camera_info");
    string pose_topic_suf = getenv("ACQ_POSES_TOPIC",      "poses");
    string config_file    = getenv("ACQ_CONFIG_YML",       "config.yml");
    string calibr_file    = getenv("ACQ_CALIBR_YAML",      "calibr.yaml");
    string detector_type  = getenv("ACQ_DETECTOR_TYPE",    "DFC");
    int    img_proc_num   = getenv("ACQ_IMAGE_PROCESSORS",  3);
    float  marker_size    = getenv("ACQ_TAG_SIZE",          0.065);

//    string img_raw_topic  = "/" + device_name + "/" + img_topic_suf;
//    string cam_info_topic = "/" + device_name + "/" + info_topic_suf;
    string tag_pose_topic = "/poses_acquisition/" + pose_topic_suf;

    ros::Publisher apriltags_pub = node_handle.advertise<duckietown_msgs::AprilTagExtended>(tag_pose_topic, 20);

    raspicam::RaspiCam_Cv video;
    video.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    if (!video.open()) {
        cerr << "Error opening camera" << endl;
        return -1;
    }
    sleep(3);

    concurrent_blocking_queue<cv::Mat> img_queue;
    concurrent_blocking_queue<tag_data> tag_queue;

    vector<thread> img_processors;
    for (int i = 0; i < img_proc_num; i++) {
        marker_tracker_params params;
        params.height = video.getHeight();
        params.width = video.getWidth();
        params.calibr_file = calibr_file;
        params.config_file = config_file;
        params.marker_size = marker_size;
        img_processors.emplace_back(thread(img_processor, params, ref(img_queue), ref(tag_queue)));
    }

    cv::Mat image;
    tag_data tag;
    duckietown_msgs::AprilTagExtended apriltag_msg;
    geometry_msgs::Vector3 vec3;
    geometry_msgs::Quaternion quat;
    int iter = 0;
    int tag_msg_seq = 0;
    timer_avg timer;
    while (video.grab() && ros::ok()) {
        video.retrieve(image);
        img_queue.push(move(image));
        ros::Time t = ros::Time::now();
        while (tag_queue.size() != 0) {
            tag_queue.pop(tag);
            apriltag_msg.tag_id = tag.tag_id;
            apriltag_msg.tag_family = tag.tag_family;
            for (int i = 0; i < 4; i++) {
                apriltag_msg.corners[2 * i + 0] = tag.corners[i].x;
                apriltag_msg.corners[2 * i + 1] = tag.corners[i].y;
            }
            vec3.x = tag.tvec.at<float>(0);
            vec3.y = tag.tvec.at<float>(1);
            vec3.z = tag.tvec.at<float>(2);
            apriltag_msg.transform.translation = vec3;
            apriltag_msg.transform.rotation = rvec2quat(tag.rvec);
            apriltag_msg.header.stamp = t;
            apriltag_msg.header.seq = tag_msg_seq++;
            apriltag_msg.header.frame_id = device_name;
            apriltags_pub.publish(apriltag_msg);
        }

//        img_msg.height = image.rows;
//        img_msg.width = image.cols;
//        img_msg.step = image.cols * 3;
//        img_msg.data.assign(image.data, image.data + image.rows * image.cols * 3);
//        img_msg.encoding = sensor_msgs::image_encodings::BGR8;
//        img_msg.header.stamp = ros::Time::now();
//        img_msg.header.seq = id++;
//        comp_img_pub.publish(img_msg);
        timer.tick();

        cout << "                                                         " << ++iter << "   " << img_queue.size() << "   " << timer.avg() * 1000 << endl;
    }
    img_queue.close();
    for (int i = 0; i < img_proc_num; i++) {
        img_processors[i].join();
    }
}