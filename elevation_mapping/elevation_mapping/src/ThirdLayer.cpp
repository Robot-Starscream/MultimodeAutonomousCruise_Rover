/*
 * ThirdLayer.cpp
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

// STL
#include <limits>
#include <algorithm>
#include <vector>
#include <cmath>

//nav
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include <Eigen/Core>

//message
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "elevation_mapping/MotionSafety.h"

//File
#include<sstream>
#include <fstream>
#include <time.h>


using namespace std;

string TitleName;

// Overall logic: 
// Subscribe to the modeID output from the first and second layer decisions, 
// and the flag indicating the start of mode execution. 
// Start recording various data and evaluating indicators. 
// Stop recording when the flag indicating the stop of mode execution is subscribed.
class ThirdLayer
{
private:

    ros::NodeHandle nh;

    ros::Subscriber roverState;

    ros::Subscriber robotPose;

    ros::Subscriber ModeIDsub_;

    ros::Publisher SafeIDpub_;

    vector<double> poseOnce;

    vector<vector<double>> pose;

    double AdditionalLateralAcceleration;

    double AdditionalVerticalAcceleration;

    vector<double> LateralForce, NormalForce;

    double Roll, Pitch, Yaw;

public:
    ThirdLayer();

    void GetPositionZCallback(const nav_msgs::Odometry &msg);

    void GetPoseCallback(const nav_msgs::Odometry &msg);

    void GetModeIDCallback(const geometry_msgs::Vector3 &msg);
};

ThirdLayer::ThirdLayer()
{
    roverState = nh.subscribe("/Rover_States", 10, &ThirdLayer::GetStatesCallback, this);
    robotPose = nh.subscribe("/zed2/zed_node/odom", 10, &ThirdLayer::GetPoseCallback, this);
    ModeIDsub_ = nh.subscribe("/ModeID", 10, &ThirdLayer::GetModeIDCallback, this);
    SafeIDpub_ = nh.advertise<geometry_msgs::Vector3>("/Third_ModeID", 100);

    LateralForce.resize(6);
    NormalForce.resize(6);
}

void ThirdLayer::GetStatesCallback(const elevation_mapping::Rover_State &msg)
{
    AdditionalLateralAcceleration = msg.acc_y;
    AdditionalVerticalAcceleration = msg.acc_z;

    for (i = 0; i < 6; i++) 
    {
        LateralForce[i] = (msg.forces.lateral_force[i]);
        NormalForce[i] = (msg.forces.normal_force[i]);
    }
}

void ThirdLayer::GetPoseCallback(const nav_msgs::Odometry& msg)
{
    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    Roll = roll;
    Pitch = pitch;
    Yaw = yaw;
    poseOnce.push_back(roll);
    poseOnce.push_back(pitch);
    poseOnce.push_back(yaw);
    pose.push_back(poseOnce);
    poseOnce.clear();
}

InputData ThirdLayer::CalInputOfFuzzyLogicSystem(InputData& inData)
{
    inData.pitch = Pitch;
    inData.roll = Roll;
    inData.Acc_L = AdditionalLateralAcceleration;
    inData.Acc_Z = AdditionalVerticalAcceleration;

    inData.F_Llrw = fabs(LateralForce[0] + LateralForce[2] + LateralForce[4] - LateralForce[1] - LateralForce[3] - LateralForce[5]) / 3.0;
    inData.F_Nfrw = 3.0 * fabs(LateralForce[0] + LateralForce[1] - LateralForce[4] - LateralForce[5]);

    double W_max = *std::max_element(normal_force.begin(), normal_force.end());
    double W_min = *std::min_element(normal_force.begin(), normal_force.end());
    double W_average = std::accumulate(normal_force.begin(), normal_force.end(), 0.0) / normal_force.size();
    inData.delta_W_max = W_max - W_min;
    inData.W_fmr = std::max(W_max - W_average, W_average - W_min);
}

void ThirdLayer::GetModeIDCallback(const geometry_msgs::Vector3 &msg)
{
    double ID = msg.y;
    double outputID;
    string riskLevel;
    MotionSafety::MotionSafety FLS_safety;
    geometry_msgs::Vector3 thirdID;

    ofstream outputfile;
    string filepath = "/home/jie/slam_project/map04_ws/src/data";
    string filename = "ThridLayerData";
    filename = filepath + filename + TitleName;

    InputData inData;
    CalInputOfFuzzyLogicSystem(inData);

    if (msg.y == 0 || msg.y == 5.0)
    {
        thirdID.x = msg.y;
    }
    else if(msg.y == 1.1 || msg.y == 2.1 || msg.y == 3.1)
    {
        outputID = FLS_safety.RiskEvaluation_2(inData,ID,riskLevel);
        if(outputID == 5.0)
        {
            thirdID.x = outputID;
        }
        else
        {
            thirdID.x = msg.y;
        }
    }
    else
    {
        outputID = FLS_safety.RiskEvaluation_1(inData,ID,riskLevel);
        if(outputID == 5.0)
        {
            thirdID.x = outputID;
        }
        else
        {
            thirdID.x = msg.y;     
        }
    }

    SafeIDpub_.publish(thirdID);
    outputfile.open(filename, std::ios::out | std::ios::app);
    outputfile << riskLevel << "\t" << thirdID << endl;
    outputfile.close();
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ThirdLayer");

    time_t timep;
    static char name[256] = {0};
    time(&timep);
    strftime( name, sizeof(name), "%Y.%m.%d %H-%M-%S.txt",localtime(&timep) );
    TitleName = name;
    
    ThirdLayer motionSafe;
    ros::spin();
    return 0;
}

