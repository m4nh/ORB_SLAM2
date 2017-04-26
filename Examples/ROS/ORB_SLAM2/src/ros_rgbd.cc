/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <map>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "../../../include/KeyFrame.h"

#define TIMINGS_DEBUG

using namespace std;
using namespace ORB_SLAM2;

struct Timings
{
#ifdef TIMINGS_DEBUG
    typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::milliseconds ms;
    typedef std::chrono::microseconds us;
    typedef std::chrono::duration<float> fsec;

    std::map<std::string, std::chrono::time_point<std::chrono::system_clock>> times;
#endif

    void operator()(std::string name)
    {
#ifdef TIMINGS_DEBUG
        startTimer(name);
#endif
    }

    void operator[](std::string name)
    {
#ifdef TIMINGS_DEBUG
        printf("Timer[%s]: %f ms\n", name.c_str(), elapsedMicroseconds(name).count() / 1000.0f);
#endif
    }

#ifdef TIMINGS_DEBUG

    void startTimer(std::string name)
    {
        times[name] = Time::now(); //IS NOT ROS TIME!
    }

    us elapsedMicroseconds(std::string name)
    {
        fsec elaps = Time::now() - times[name];
        return std::chrono::duration_cast<us>(elaps);
    }

    ms elapsedMilliseconds(std::string name)
    {
        fsec elaps = Time::now() - times[name];
        return std::chrono::duration_cast<ms>(elaps);
    }
#endif
} TIMINGS;

//#################################################################################
//#################################################################################
//#################################################################################
//#################################################################################
//#################################################################################
//#################################################################################
//#################################################################################

struct FrameRGBD
{
    cv::Mat rgb;
    cv::Mat depth;
    string timestamp;
    bool is_keyframe;

    FrameRGBD()
    {
        is_keyframe = false;
    }
    FrameRGBD(cv::Mat rgb, cv::Mat depth, double timestamp, bool is_keyframe = false)
    {
        this->rgb = rgb.clone();
        this->depth = depth.clone();
        this->timestamp = KeyFrame::timestampStringConversion(timestamp);
        this->is_keyframe = is_keyframe;
    }
};

struct KeyFrameMap
{
    map<string, FrameRGBD> rgbd_frames;
    map<string, cv::Mat> keyframes_pose;

    void addRGBDFrame(FrameRGBD frame)
    {
        rgbd_frames[frame.timestamp] = frame;
        ROS_INFO("Size: %d", (int)rgbd_frames.size());
    }

    void udpateKeyFrames(std::vector<KeyFrame *> &keyframes)
    {

        for (size_t i = 0; i < keyframes.size(); i++)
        {
            string key = keyframes[i]->getTimeStampString();
            if (this->keyframes_pose.count(key) == 0)
            {
                this->keyframes_pose[key] = keyframes[i]->GetCameraCenter();
                //ROS_INFO("NEW KEYFRAME!");
            }
            else
            {

                cv::Mat new_pose = keyframes[i]->GetCameraCenter();
                cv::Mat old_pose = this->keyframes_pose[key];
                double dist = cv::norm(new_pose, old_pose);
                ROS_INFO("OLD KEYFRAME! %f", dist);
                if (dist > 0.00001)
                {
                    ROS_INFO("#################### UPDATE DETECTED!!");
                }
                this->keyframes_pose[key] = keyframes[i]->GetCameraCenter();
            }
        }
    }

} keyFrameMap;

class ImageGrabber
{
  public:
    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    ORB_SLAM2::System *mpSLAM;
};

ORB_SLAM2::System *SLAM_HANDLE;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if (argc != 3)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);
    SLAM_HANDLE = &SLAM;
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/xtion/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/xtion/depth/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());

    //Test

    TIMINGS("test");
    FrameRGBD rgbd_frame(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    keyFrameMap.addRGBDFrame(rgbd_frame);

    vector<KeyFrame *> keyframes;
    SLAM_HANDLE->GetAllKeyFrames(keyframes);
    keyFrameMap.udpateKeyFrames(keyframes);
    TIMINGS["test"];
    ROS_INFO("Keyframes: %d", int(keyframes.size()));
}
