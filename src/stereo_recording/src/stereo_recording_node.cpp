// MIT License

// Copyright (c) 2022 Hoang Viet Do <hoangvietdo@sju.ac.kr>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
    
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <api/RpcLibClientBase.hpp>
#include <common/VectorMath.hpp>
#include <common/common_utils/FileSystem.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

typedef common_utils::FileSystem FileSystem;
typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

msr::airlib::RpcLibClientBase client;

std::vector<ImageRequest> request = { ImageRequest("Front_Left_Camera", ImageType::Scene), ImageRequest("Front_Right_Camera", ImageType::Scene) };

int main(int argc, char** argv)
{

    client.confirmConnection();

    ros::init(argc, argv, "SJU_stereo_recording_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    rosbag::Bag bagIn;
    std::string rosbag_filename_in = "test.bag";
    std::string rosbag_path_in = "/home/vietdo/Documents/";
    std::string rosbag_in = rosbag_path_in + rosbag_filename_in;
    bagIn.open(rosbag_in, rosbag::bagmode::Read);
    std::cout << "Reading input to: " << rosbag_in << std::endl;

    rosbag::Bag bagOut;
    std::string rosbag_filename_out = "test_out.bag";
    std::string rosbag_path_out = "/home/vietdo/Documents/";
    std::string rosbag_out = rosbag_path_out + rosbag_filename_out;
    bagOut.open(rosbag_out, rosbag::bagmode::Write);
    std::cout << "Storing output to: " << rosbag_out << std::endl;

    std::vector<std::string> topics;
    topics.push_back(std::string("airsim_node/SJU_Vehicle/imu/Imu"));
    topics.push_back(std::string("airsim_node/SJU_Vehicle/odom_local_ned"));

    int count = 0;
    int fail_count = 0;
    auto IMU_hz = 100.0;
    auto Odom_hz = 20.0;
    auto Camera_hz = 20.0;

    rosbag::View view(bagIn, rosbag::TopicQuery(topics));
    nav_msgs::Path pathMsg;
    nav_msgs::Path pathMsg_open_vins;

    std::cout << "Recording Start!" << std::endl;

    foreach (rosbag::MessageInstance const foo, view) {

        sensor_msgs::Imu::ConstPtr imuMsg = foo.instantiate<sensor_msgs::Imu>();

        if (imuMsg != NULL) {
            auto imu_timestamp = imuMsg->header.stamp;
            bagOut.write("SJU_Vehicle/imu", imu_timestamp, imuMsg);
        }

        nav_msgs::Odometry::ConstPtr odomMsg = foo.instantiate<nav_msgs::Odometry>();

        if (odomMsg != NULL) {
            auto odom_timestamp = odomMsg->header.stamp;
            bagOut.write("SJU_Vehicle/odom", odom_timestamp, odomMsg);

            geometry_msgs::PoseStamped pose_;
            pose_.pose.position.x = odomMsg->pose.pose.position.x;
            pose_.pose.position.y = -odomMsg->pose.pose.position.y;
            pose_.pose.position.z = -odomMsg->pose.pose.position.z;

            pose_.pose.orientation.w = odomMsg->pose.pose.orientation.w;
            pose_.pose.orientation.x = odomMsg->pose.pose.orientation.x;
            pose_.pose.orientation.y = odomMsg->pose.pose.orientation.y;
            pose_.pose.orientation.z = odomMsg->pose.pose.orientation.z;

            pose_.header.stamp = odomMsg->header.stamp;
            pose_.header.frame_id = "world";

            pathMsg.poses.push_back(pose_);
            pathMsg.header.stamp = odom_timestamp;
            pathMsg.header.frame_id = "world";

            bagOut.write("SJU_Vehicle/path", odom_timestamp, pathMsg);

            geometry_msgs::PoseStamped pose_open_vins_;
            pose_open_vins_.pose.position.x = odomMsg->pose.pose.position.x;
            pose_open_vins_.pose.position.y = -odomMsg->pose.pose.position.y;
            pose_open_vins_.pose.position.z = -odomMsg->pose.pose.position.z;
            pose_open_vins_.pose.orientation.w = odomMsg->pose.pose.orientation.w;
            pose_open_vins_.pose.orientation.x = odomMsg->pose.pose.orientation.x;
            pose_open_vins_.pose.orientation.y = odomMsg->pose.pose.orientation.y;
            pose_open_vins_.pose.orientation.z = odomMsg->pose.pose.orientation.z;
            pose_open_vins_.header.stamp = odomMsg->header.stamp;
            pose_open_vins_.header.frame_id = "global";

            pathMsg_open_vins.poses.push_back(pose_open_vins_);
            pathMsg_open_vins.header.stamp = odom_timestamp;
            pathMsg_open_vins.header.frame_id = "global";

            bagOut.write("SJU_Vehicle/path_open_vins", odom_timestamp, pathMsg_open_vins);
            // count += 1;

            // if (count == (Odom_hz / Camera_hz)) {
            msr::airlib::Pose pose;
            pose.position.x() = odomMsg->pose.pose.position.x;
            pose.position.y() = odomMsg->pose.pose.position.y;
            pose.position.z() = odomMsg->pose.pose.position.z;

            pose.orientation.w() = odomMsg->pose.pose.orientation.w;
            pose.orientation.x() = odomMsg->pose.pose.orientation.x;
            pose.orientation.y() = odomMsg->pose.pose.orientation.y;
            pose.orientation.z() = odomMsg->pose.pose.orientation.z;

            client.simSetVehiclePose(pose, true);

            client.simPause(true);

            const std::vector<ImageResponse>& response = client.simGetImages(request);

            // Left Camera
            cv_bridge::CvImage cvImageLeft;

            cv::Mat imgLeft = cv::imdecode(response.at(0).image_data_uint8, cv::IMREAD_COLOR);

            cvImageLeft.image = imgLeft;
            cvImageLeft.encoding = sensor_msgs::image_encodings::BGR8;
            cvImageLeft.header.stamp = odom_timestamp;

            // Right Camera
            cv_bridge::CvImage cvImageRight;

            cv::Mat imgRight = cv::imdecode(response.at(1).image_data_uint8, cv::IMREAD_COLOR);

            cvImageRight.image = imgRight;
            cvImageRight.encoding = sensor_msgs::image_encodings::BGR8;
            cvImageRight.header.stamp = odom_timestamp;

            bagOut.write("SJU_Vehicle/left_camera", odom_timestamp, cvImageLeft.toImageMsg());
            bagOut.write("SJU_Vehicle/right_camera", odom_timestamp, cvImageRight.toImageMsg());

            client.simPause(false);

            // count = 0;
            // }
        }
    }

    bagIn.close();
    bagOut.close();

    std::cout << "Recording End!" << std::endl;

    return 0;
}
