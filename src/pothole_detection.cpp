#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <dynamic_reconfigure/server.h>
#include <ltu_actor_route_pothole/PotholeConfig.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>


/* Pothole Detection
 * A Class that detects if there is a pothole in front of the car.
 *
 * Publish:     UInt8 on "~/stop_sign"
 *              UInt32 on "~/sign_size"
 * Subscribe: <params/front_cam_in>
 */
class PotholeDetection
{

public:
    PotholeDetection();

    bool isEnabled() {return enabled;}
    void shutdown() {
        image_sub_ = image_transport::Subscriber();
        enabled =  false;
    }
    void startup(){
        image_sub_ = it_.subscribe(dashcam_topic, 1, &PotholeDetection::imageCb, this);
        enabled = true;
    }
    bool hasSub(){
        return (trigger_pub_.getNumSubscribers() || image_pub_.getNumSubscribers()) > 0;
    }

private:
    // Callbacks
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void configCB(ltu_actor_route_pothole::PotholeConfig &config, uint32_t level);

    // Run sign detection
    void detectAndDisplay(cv::Mat& frame);

    // ROS Objects
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_; // debug pub
    ros::Publisher trigger_pub_;

    std::string dashcam_topic;
    bool enabled = true;

    // Dynamic reconfigure server
    dynamic_reconfigure::Server<ltu_actor_route_pothole::PotholeConfig> server_;
    ltu_actor_route_pothole::PotholeConfig config_;

    bool pothole_detected_;
};

PotholeDetection::PotholeDetection() : nh_("~"), it_(nh_)
{
    // Subscribe to camera
    std::string dashcam_topic;
    if (!nh_.getParam("input", dashcam_topic))
    {
        ROS_ERROR_STREAM("No lidar topic passed to " + dashcam_topic);
        throw std::invalid_argument("Bad lidar topic.");
    }
    
    //ROS_INFO_STREAM("dashcam_topic = " + dashcam_topic);

    image_sub_ = it_.subscribe(dashcam_topic, 1, &PotholeDetection::imageCb, this);

    // Dynamic reconfigure
    server_.setCallback(boost::bind(&PotholeDetection::configCB, this, _1, _2));
    server_.getConfigDefault(config_);

    trigger_pub_ = nh_.advertise<std_msgs::String>("/actor_nav/trigger", 10);
    image_pub_ = it_.advertise("debug", 1);

    pothole_detected_ = false;
}

void PotholeDetection::configCB(ltu_actor_route_pothole::PotholeConfig &config, uint32_t level)
{
    config_ = config;
}

static int clamp(int v, int lo, int hi)
{
    return std::max(lo+1, std::min(hi-1, v));
}

/** @function detectAndDisplay */
void PotholeDetection::detectAndDisplay( cv::Mat& frame )
{
    cv::Mat frame_gray;


    int width = frame.cols;
    int height = frame.rows;


    // Get roi values
    int xlo = clamp(config_.xlo, 0,         width/2);
    int xhi = clamp(config_.xhi, 0,   width);
    int ylo = clamp(config_.ylo, 0,         height/2);
    int yhi = clamp(config_.yhi, 0,  height);

    // Generate b&w roi
    auto roi_rect = cv::Rect(xlo, ylo, xhi, yhi);
    cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );

    //ROS_INFO_STREAM(xlo << ", " << ylo << "; " << xhi << ", " << yhi);
    auto roi = cv::Mat(frame_gray, roi_rect);

    // Count white pixels in roi
    cv::threshold(roi, roi, clamp(config_.bin_thresh, 0, 255), 255, cv::THRESH_TOZERO);
    int count = cv::countNonZero(roi);
    bool found = count > config_.pixel_count_thresh;


    // Display rect
    const int thickness = 6;
    if (found)
    {
        cv::rectangle(frame, roi_rect, cv::Scalar(0,255,0), thickness);
    }
    else
    {
        cv::rectangle(frame, roi_rect, cv::Scalar(255,0,0), thickness/2);
    }
    
    //ROS_INFO_STREAM("count: " << count << ", found: " << found);


    // Count the number of valid consecutive sign frames
    //    Only publish a detection if the number of frames is
    //    above a certain threshold (frame_count_trigger_)
    static int frame_count = 0;

    //ROS_INFO_STREAM("sign frames: " << frame_count);
    if (found) {
        frame_count++;
        //ROS_INFO_STREAM("Found");
        if (frame_count > config_.frame_count_trigger) {
            pothole_detected_ = true;
            //ROS_INFO_STREAM("Pothole Detected");
        }
    } else {
    	//ROS_INFO_STREAM("Not Found");
        pothole_detected_ = false;
        frame_count = 0;
    }

    // Publish sign
    if (pothole_detected_)
    {
        std_msgs::String trigger_msg;
        trigger_msg.data = "POTHOLE";
        trigger_pub_.publish(trigger_msg);
    }


}

void PotholeDetection::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    //Convert to cv image
    ROS_INFO_STREAM("oogly boogly");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    detectAndDisplay(cv_ptr->image);

    //if (image_pub_.getNumSubscribers() > 0) {
    //We don't want it to be efficient, we need it to work.		
    image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg());
    //}

    cv::waitKey(3);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pothole_detection");
    PotholeDetection pd;

    ros::Rate r(10); 

    while (ros::ok()){
        if (pd.hasSub()){
            if (!pd.isEnabled()){
                pd.startup();
            }
        } else {
            if (pd.isEnabled()){
                //pd.shutdown();
            }
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
