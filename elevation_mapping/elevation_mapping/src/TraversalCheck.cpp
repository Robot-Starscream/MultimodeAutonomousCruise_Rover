/*
 * TraversalCheck.cpp
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/TypeDefs.hpp"

#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/GridMapMsgHelpers.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>

// ROS
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_broadcaster.h>

// STL
#include <limits>
#include <algorithm>
#include <vector>

//pointcloud
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

//pcl point

#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/io/pcd_io.h>

//elevation_mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapping.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>

using namespace grid_map;
using namespace std;

class Traversability
{
    public:
        Traversability(ros::NodeHandle &n);
        ~Traversability();
        ros::NodeHandle& n;
        ros::Subscriber gridMapSub;
        ros::Publisher DiffPublisher;

        double slopeCritical;
        double stepCritical;
        double stepRadiusFirst;
        double stepRadiusSecond;
        int cellsCritical;
        double boundCritical;
        int boundDiffNum;

        void check(const grid_map_msgs::GridMap& gridMapIn);
};

Traversability::~Traversability()
{}

Traversability::Traversability(ros::NodeHandle& n): n(n)    
{
    DiffPublisher = n.advertise<grid_map_msgs::GridMap>("Two_Value_Map", 1, true);// Publish the difference map
    gridMapSub = n.subscribe("/elevation_mapping/elevation_map", 10, &Traversability::check, this);
}

void Traversability::check(const grid_map_msgs::GridMap& gridMapIn)
{
    cout<<"----------------------------"<<endl;
    // Convert the read map type to grid_map type
    grid_map::GridMap localGridMap;
    GridMapRosConverter::fromMessage(gridMapIn, localGridMap);

    double height_data; // Height data of the circular iterator
    double height_center; // Height at the center of the robot
    double huancun = 0; // Used to store heights below the threshold in the circular area
    double height_average; // Calculate the average height of the credible area
    int cishu = 0;

    Eigen::Vector2d center;
    center = localGridMap.getPosition(); // Get the 2D position of the robot center
    double circle_margin = 0; // Threshold within the circular area
    double margin = 0; // Obstacle detection height threshold
    grid_map::Index center_index; // Index value at the center of the submap (robot center)
    localGridMap.getIndex(center, center_index);

    // Add a new layer - binary
    localGridMap.add("two_value_map");

    for (CircleIterator submapIterator(localGridMap, center, stepRadiusFirst);
          !submapIterator.isPastEnd(); ++submapIterator)
    {
        height_center = localGridMap.at("elevation", center_index); // Get the height data at the center of the robot
        // Determine if the data at the center position is nan, if not, keep it as is, if it is, set the height data at that position to 0
        if(isnan(height_center))
        {
          height_center = 0;
        }

        if (!localGridMap.isValid(*submapIterator, "elevation"))
          continue;
        height_data = localGridMap.at("elevation", *submapIterator);
        if (height_data > height_center + circle_margin)
        {
          continue;
        }else{
          huancun += height_data;
        }
        cishu ++;
    }
    height_average = huancun / cishu;

    double height, step;
    for (GridMapIterator iterator(localGridMap); !iterator.isPastEnd(); ++iterator)
    {
      if (!localGridMap.isValid(*iterator, "elevation"))
        continue;
      height = localGridMap.at("elevation", *iterator);

      if (isnan(height))
      {
        height = 0;
      }

      if (height > height_average + margin)
      {
        localGridMap.at("two_value_map", *iterator) = 1;
      }else{
        localGridMap.at("two_value_map", *iterator) = 0;
      }
    }


    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(localGridMap, message);
    DiffPublisher.publish(message);


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traversability");

  ros::NodeHandle n;

  Traversability traversability(n);

  ros::spin();

  return 0;
}