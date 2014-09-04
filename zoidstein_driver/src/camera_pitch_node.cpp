/*
Copyright (c) 2014, Jamie Diprose
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the {organization} nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <ros_pololu_servo/MotorCommand.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <angles/angles.h>
#include <limits>

using namespace std;
std::string point_cloud_topic, joint_name, pololu_topic;

ros::Subscriber cloud_sub;
ros::Publisher command_pub;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::VoxelGrid<pcl::PointXYZRGB> VoxelGridFilter;
double leaf_size = 0.09;

double interpolate(double value, double old_min, double old_max, double new_min, double new_max)
{
    // Width of each range
    double old_range = old_max - old_min;
    double new_range = new_max - new_min;

    // Scale old value into range between 0 and 1
    double scaled_value = double(value - old_min) / double(old_range);

    // Convert the scaled value into the new range
    double new_val = new_min + (scaled_value * new_range);

    return new_val;
}

void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(*msg, cloud);

    pcl::PCLPointCloud2::Ptr cloud_ptr(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    *cloud_ptr = cloud;

    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setInputCloud(cloud_ptr);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*cloud_filtered);

    PointCloud final_cloud;
    pcl::fromPCLPointCloud2(*cloud_filtered, final_cloud);

    int num_out_range = 0;
    int num_too_close = 0;
    int num_valid = 0;
    double sum = 0.0;
    double average_distance = 0;
    double position;

    BOOST_FOREACH (const pcl::PointXYZRGB& pt, final_cloud.points)
    {
        if(pt.z == -1)
        {
            num_out_range++;
        }
        else if(pt.z == -2)
        {
            num_too_close++;
        }
        else if(pt.z != 0.0)
        {
            sum += pt.z;
            num_valid++;
        }
    }

    if(num_too_close == 1)
    {
        position = angles::from_degrees(40.0);
    }
    else if(num_out_range == 1 && final_cloud.points.size() == 2)
    {
        position = 0.0;
    }
    else
    {
        average_distance = sum / num_valid;
        position = interpolate(average_distance, 0.0, 1.57, angles::from_degrees(50.0), angles::from_degrees(6.0)); //TODO: clamp values between min / max
        //TODO: get min max rotation from service
    }

    ROS_INFO("dist: %f, position: %f, num_valid %d, num_to_close: %d, num_out_range: %d, size: %u", average_distance, position, num_valid, num_too_close, num_out_range, final_cloud.points.size());

    // Publish motor command
    ros_pololu_servo::MotorCommand command;
    command.joint_name = joint_name;
    command.position = position;
    command.speed = 0.1;
    command.acceleration = 0.05;
    command_pub.publish(command);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "camera_pitch");
	ros::NodeHandle nh;
	ros::NodeHandle nhr("~");

    // Load parameters
    nhr.param<string>("point_cloud_topic", point_cloud_topic, "");
    nhr.param<string>("joint_name", joint_name, "");
    nhr.param<string>("pololu_topic", pololu_topic, "");

    ROS_INFO("point_cloud_topic: %s", point_cloud_topic.c_str());
    ROS_INFO("joint_name: %s", joint_name.c_str());
    ROS_INFO("pololu_topic: %s", pololu_topic.c_str());

    command_pub = nh.advertise<ros_pololu_servo::MotorCommand>(pololu_topic + "/command", 1);
    cloud_sub = nh.subscribe(point_cloud_topic, 1, point_cloud_callback);
    ROS_INFO("Running");
    ros::spin();
    return 0;
}