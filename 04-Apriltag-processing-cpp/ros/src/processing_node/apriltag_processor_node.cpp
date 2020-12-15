#include <ros/ros.h>

#include <chrono>
#include <iostream>
#include <fstream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <boost/lexical_cast.hpp>
#include <opencv2/core.hpp>
#include <raspicam_cv.h>
#include <yaml-cpp/yaml.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include "duckietown_msgs/AprilTagDetection.h"
#include "duckietown_msgs/AprilTagDetectionArray.h"
#include "aruco.h"
#include "dcf/dcfmarkertracker.h"


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

class arg_timer_avg {
public:
    explicit arg_timer_avg(int n = 100) :
            n(n) {
        for (int i = 0; i < n; i++) {
            times.push(chrono::high_resolution_clock::now());
            args.push(0.0);
        }
        sum = 0;
    }

    inline void tick(double t) {
        times.push(chrono::high_resolution_clock::now());
        times.pop();
        sum += t - args.back();
        args.push(t);
        args.pop();
    }

    double vel() {
        return (sum * n) / (chrono::duration_cast<chrono::microseconds>(times.back() - times.front()).count()) * 1e6;
    }

private:
    queue<chrono::high_resolution_clock::time_point> times;
    queue<double> args;
    double sum;
    size_t n;
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

template<typename T>
class concurrent_blocking_queue {
public:
    concurrent_blocking_queue() {
        closed = false;
    }

    bool pop(T &item) {
        unique_lock<mutex> lock(m);
        while (q.empty() && !closed) {
            c.wait(lock);
        }
        if (closed) {
            return false;
        }
        item = q.front();
        q.pop();
        return true;
    }

    void push(const T &item) {
        unique_lock<mutex> lock(m);
        q.push(item);
        lock.unlock();
        c.notify_one();
    }

    void push(T &&item) {
        unique_lock<mutex> lock(m);
        q.push(move(item));
        lock.unlock();
        c.notify_one();
    }

    void close() {
        closed = true;
        c.notify_all();
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
    vector<cv::Point2f> corners;
    cv::Mat tvec, rvec;
};

struct tags_data {
    vector<tag_data> tags;
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

sensor_msgs::CameraInfo cam_info;

void save_params(const string &calibr_file) {
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

void img_processor(const marker_tracker_params& params,
                   concurrent_blocking_queue<cv::Mat> &img_queue,
                   concurrent_blocking_queue<tags_data> &tags_queue) {
    DFCMarkerTracker marker_tracker;
    CameraParameters cam_params = load_params(params.height, params.width, params.calibr_file);
    marker_tracker.setParams(cam_params, params.marker_size);
    marker_tracker.loadParamsFromFile(params.config_file);
    cv::Mat image;
    while (img_queue.pop(image)) {
        map<int, cv::Ptr<TrackerImpl>> set_trackers = marker_tracker.track(image, 0.1);
        marker_tracker.estimatePose();
        vector<Marker> markers_vec(set_trackers.size());
        tags_data tags;
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
                tag.corners.push_back(marker[j]);
            }
            tags.tags.push_back(tag);
        }
        tags_queue.push(tags);
    }
}

geometry_msgs::Quaternion rvec2quat(cv::Mat rvec) {
    geometry_msgs::Quaternion quat;
    double x = rvec.at<float>(0);
    double y = rvec.at<float>(1);
    double z = rvec.at<float>(2);
    double r = sqrt(x * x + y * y + z * z);
    if (r < 0.00001) {
        quat.x = 1;
        return quat;
    }
    double c = cos(r / 2);
    double s = sin(r / 2);
    quat.x = c;
    quat.y = s * z / r;
    quat.z = -s * y / r;
    quat.w = -s * x / r;
    return quat;
}

concurrent_blocking_queue<tags_data> tags_queue;

void publisher(ros::Publisher &apriltags_pub, string &device_name) {
    cv::Mat image;
    tags_data tags;
    geometry_msgs::Vector3 vec3;
    geometry_msgs::Quaternion quat;
    int tag_msg_seq = 0;
    arg_timer_avg timer;
    while (tags_queue.pop(tags)) {
        duckietown_msgs::AprilTagDetectionArray apriltags_msg;
        timer.tick(tags_queue.size());
        cout << "                                                                 "
             << "                                                   " << timer.vel() << endl;

        ros::Time t = ros::Time::now();
        for (auto tag : tags.tags) {
            if (tag.tvec.cols * tag.tvec.rows != 3 || tag.rvec.cols * tag.rvec.rows != 3) {
                cout << "                                                                           ";
                cout << "estimation_error: tvec=" << tag.tvec << "  rvec=" << tag.rvec << endl;
                continue;
            }
            duckietown_msgs::AprilTagDetection apriltag_msg;
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

            apriltags_msg.detections.push_back(apriltag_msg);
        }
        if (apriltags_msg.detections.empty()) {
            continue;
        }

        apriltags_msg.header.stamp = t;
        apriltags_msg.header.seq = tag_msg_seq++;
        apriltags_msg.header.frame_id = device_name;
        apriltags_pub.publish(apriltags_msg);
    }
}

bool was_cam_info = false;

void cam_info_callback(const sensor_msgs::CameraInfo &cam_info_) {
    was_cam_info = true;
    cam_info = cam_info_;
}

concurrent_blocking_queue<cv::Mat> img_queue;

void img_raw_callback(const sensor_msgs::ImageConstPtr &img) {
    if (!was_cam_info) {
        return;
    }
    static int iter = 0;
    static timer_avg timer;
    static TimerAvrg timer1;
    static TimerAvrg timer2;

    timer1.start();
    bool mono = img->encoding == sensor_msgs::image_encodings::MONO8 ||
                img->encoding == sensor_msgs::image_encodings::TYPE_8UC1;
    int type = mono ? CV_8UC1 : CV_8UC3;

    const cv::Mat tmp(img->height, img->width, type, const_cast<uint8_t *>(img->data.data()), img->step);
    timer1.stop();
    timer2.start();

    if (!mono) {
        cv::Mat mono_image;
        cv::cvtColor(tmp, mono_image, cv::COLOR_BGR2GRAY);
        img_queue.push(move(mono_image));
    } else {
        cv::Mat image;
        tmp.copyTo(image);
        img_queue.push(move(image));
    }
    timer2.stop();

    timer.tick();
    cout << "r   " << (mono ? "mono" : "bgr ") << "                                                "
         << ++iter << "   " << img_queue.size() << "   " << timer.avg() * 1000
         << "   " << timer1.getAvrg() * 1000
         << "   " << timer2.getAvrg() * 1000
         << endl;
}

void img_comp_callback(const sensor_msgs::CompressedImageConstPtr &img) {
    if (!was_cam_info) {
        return;
    }
    static int iter = 0;
    static timer_avg timer;
    static TimerAvrg timer1;
    static TimerAvrg timer2;
    static TimerAvrg timer3;

    timer1.start();
    int rows = 1;
    int cols = (int) (cam_info.height * cam_info.width);
    cv::Mat buf(rows, cols, CV_8U, const_cast<uint8_t *>(img->data.data()));
    timer1.stop();
    timer2.start();
    cv::Mat color_image = cv::imdecode(buf, cv::IMREAD_COLOR);
    timer2.stop();
    cv::Mat image;
    timer3.start();
    cv::cvtColor(color_image, image, cv::COLOR_BGR2GRAY);
    timer3.stop();

    img_queue.push(move(image));

    timer.tick();
    cout << "c                                                        "
         << ++iter << "   " << img_queue.size() << "   " << timer.avg() * 1000
         << "   " << timer1.getAvrg() * 1000 << "   " << timer2.getAvrg() * 1000
         << "   " << timer3.getAvrg() * 1000 << endl;
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
    string img_topic_suf  = getenv("ACQ_TOPIC_RAW",        "camera_node/image");
    string info_topic_suf = getenv("ACQ_TOPIC_CAMERAINFO", "camera_node/camera_info");
    string pose_topic_suf = getenv("ACQ_POSES_TOPIC",      "poses");
    string config_file    = getenv("ACQ_CONFIG_YML",       "config.yml");
    string calibr_file    = getenv("ACQ_CALIBR_YAML",      "calibr.yaml");
    string detector_type  = getenv("ACQ_DETECTOR_TYPE",    "DFC");
    int    img_proc_num   = getenv("ACQ_IMAGE_PROCESSORS",  3);
    float  marker_size    = getenv("ACQ_TAG_SIZE",          0.065);

    string img_raw_topic  = "/" + device_name + "/" + img_topic_suf;
    string img_comp_topic = img_raw_topic + "/compressed2";
    string cam_info_topic = "/" + device_name + "/" + info_topic_suf;
    string tag_pose_topic = "/poses_acquisition/" + pose_topic_suf;

    image_transport::ImageTransport it(node_handle);
    image_transport::Subscriber img_it_sub = it.subscribe(img_raw_topic, 1000, img_raw_callback,
                                                          image_transport::TransportHints("compressed"));

    ros::Subscriber raw_img_sub  = node_handle.subscribe(img_raw_topic,  1000, img_raw_callback);
    ros::Subscriber comp_img_sub = node_handle.subscribe(img_comp_topic, 1000, img_comp_callback);
    ros::Subscriber cam_info_sub = node_handle.subscribe(cam_info_topic, 1,    cam_info_callback);

    ros::Publisher apriltags_pub = node_handle.advertise<duckietown_msgs::AprilTagDetectionArray>(tag_pose_topic, 20);

    ros::Rate rate(60);
    while (!was_cam_info && ros::ok()) {
        cout << "wait camera_info" << endl;
        ros::spinOnce();
        rate.sleep();
    }

    if (!ros::ok()) {
        return 1;
    }

    save_params(calibr_file);

    vector<thread> img_processors;
    for (int i = 0; i < img_proc_num; i++) {
        marker_tracker_params params;
        params.height = cam_info.height;
        params.width = cam_info.width;
        params.calibr_file = calibr_file;
        params.config_file = config_file;
        params.marker_size = marker_size;
        img_processors.emplace_back(thread(img_processor, params, ref(img_queue), ref(tags_queue)));
    }
    thread pub_thread(publisher, ref(apriltags_pub), ref(device_name));

    ros::spin();

    cout << "!!!!!!!!!!!!SHUTDOWN!!!!!!!!!!!!" << endl;

    tags_queue.close();
    img_queue.close();
    for (int i = 0; i < img_proc_num; i++) {
        img_processors[i].join();
    }
}