/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Nagy Laszlo Zoltan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// ROS specific includes
#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>

#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr PCxyzPrevious;

void callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
  // pcl::PCLPointCloud2 cloud_filtered;
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud(input);
  // sor.setLeafSize(0.01, 0.01, 0.01);
  // sor.filter(cloud_filtered);

  pcl::PointCloud<pcl::PointXYZ>::Ptr PCxyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *PCxyz);

  if (PCxyzPrevious)
  {
    ROS_INFO("Calculating transformation...");

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(PCxyz);
    icp.setInputTarget(PCxyzPrevious);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(1);
    // Perform the alignment
    pcl::PointCloud<pcl::PointXYZ> PCxyzAligned;
    icp.align(PCxyzAligned);
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transf4f = icp.getFinalTransformation();
    const Eigen::Affine3d transf3d(transf4f.cast<double>());
    ROS_INFO_STREAM("Affine3d: " << std::endl << transf4f);
    geometry_msgs::Transform transform_msg;
    tf::transformEigenToMsg(transf3d, transform_msg);
  }
  else
  {
    ROS_INFO("First cloud acquired.");
  }
  // Save the cloud for next iteration
  PCxyzPrevious = PCxyz;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("kinect_input", 1, callback);

  // Spin
  ROS_INFO("Node initialized.");

  ros::spin();
  // return 0;
}
