/*
 * Connectmap_pub.cpp
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

#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "geometry_msgs/PoseStamped.h"

#include <math.h>

#include<vector>
#include <map>

#include"elevation_mapping/ConnectAeras.h"
#include "elevation_mapping/SecondLayer.h"

#include<sstream>
#include <fstream>
#include <time.h>

#include <iomanip>
#include <string>
#include "std_msgs/Int32.h"

#include "elevation_mapping/MyMap.h"
#include "elevation_mapping/FirstLayer.h"

using namespace std;
using namespace grid_map;

vector<vector<double>> Path;
string Name;

class Connectmap_pub
{
private:

    ros::NodeHandle n_; 
    ros::Publisher pub_;
    ros::Publisher modeIDPub_;
    ros::Subscriber mapsub_;
    ros::Subscriber anglesub_;
    ros::Subscriber comTestServersub_;

    double angleZ;
    double robotZ;
    double robotPositionX;
    double robotPositionY;

    double roverPositionX;
    double roverPositionY;
    double roverPositionZ;
    double roverRow;
    double roverPitch;
    double roverYaw;

    double roverPositionInitX;
    double roverPositionInitY;
    double roverPositionInitZ;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    struct PATH
    {
        vector<float> x;
        vector<float> y;
        vector<float> z;
    };

    PATH robotPath;
    int motionState;

    int stateOnece;

    OBS myObstacle[5];

public:

    Connectmap_pub()
    {
        pub_ = n_.advertise<elevation_mapping::FirstLayer>("/First_layer/modeID", 100);
        modeIDPub_ = n_.advertise<std_msgs::Int32>("/ModeID", 10); // Motion mode topic publisher

        anglesub_ = n_.subscribe("/odom", 10, &Connectmap_pub::GetRobotAngleCallback, this);

        // Elevation map topic subscriber
        mapsub_ = n_.subscribe("/elevation_mapping/elevation_map", 10, &Connectmap_pub::GetCCLMapCallback, this); 

        // Get the motion mode start/stop flag from the upper computer
        comTestServersub_ = n_.subscribe("/gait_status", 10, &Connectmap_pub::GetMotionStateCallback, this);     
    }

    void ModeIDMapping(string ID, map<string, int> mapping, int& value)
    {
        map<string, int> ::iterator iter;
        iter = mapping.find(ID);
        if (iter != mapping.end())
        {
            value = iter->second;
            return;
        }
        else {
            value = 0;
            return;
        }
    }

    void GetRobotAngleCallback(const geometry_msgs::PoseStamped &odom)
    {
        tf::Quaternion quat;
        double roll, pitch, yaw;
		tf::quaternionMsgToTF(odom.pose.orientation, quat);
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        angleZ = yaw;
        robotZ = odom.pose.position.z;
    }


    void GetMotionStateCallback(const std_msgs::Int32 &msg)
    {
        motionState = 1;
    }

    void StrategyStart(vector<vector<float>> mapData, double resolutionData, double positionData0, double positionData1)
    {
        /************************Message mapping for publishing ID****************************/
        map<string, int> mapping;
        mapping["WRM"] = 0;
        mapping["SLWLM"] = 2;
        mapping["SRWLM"] = 3;
        mapping["BLM"] = 5;
        mapping["DWLM"] = 4;
        mapping["BCM"] = 6;
        mapping["WCM"] = 7;

        map<string, int> mapping_2;
        mapping["WRM"] = 0;
        mapping["SLWLM"] = 2;
        mapping["SLWLM"] = 2;
        mapping["SRWLM"] =3;
        mapping["SRWLM"] = 3;
        mapping["DWLM"] =4;
        mapping["DWLM"] =4;
        mapping["BLM"] = 5;
        mapping["BCM"] = 6;
        mapping["WCM"] = 7;

        ConnectComponentLabeling::ConnectAeras CCLabelMap;
        
        /**************************Set margins*****************************/
        CCLabelMap.Set_Margin(0.05); // Set the height threshold for the connected region analysis algorithm
        CCLabelMap.Set_Map(mapData); // Input the elevation map into the algorithm
        CCLabelMap.SetDistanceMargin(5, 0.3, 0.9); // Set the distance threshold for the algorithm to start
        CCLabelMap.SetAngle(angleZ); // Set the current yaw angle of the robot
        CCLabelMap.SetResolution(resolutionData); // Set the resolution of the map
        CCLabelMap.SetRobotPosition(positionData0, positionData1); // Set the position of the robot
        CCLabelMap.SetObstacleTypeData(0.05, 0.10, 0.15, 0.05, 0.10, 0.15, 0.05, 0.10, 0.15, 0.785); // Set the geometric parameter thresholds for various obstacles
        CCLabelMap.SetRobotParams(0.05, 0.05, 0.10, 0.15, 0.05); // Set the size parameters of the robot
        CCLabelMap.ReadRobotPath(Path);
        CCLabelMap.SetWheelPath();

        /*****************CCL algorithm and get label map******************/
        CCLabelMap.TWO_PASS();
        vector<vector<int>> labelMap_ = CCLabelMap.Get_ConnectMap();
        

        /***************Obtain control parameters of one layer*************/
        vector<int> number_ = CCLabelMap.GetAllLabelInMap();
        ROS_INFO("obstacle number is : %d", number_.size());
        int Obstaclenumber = number_.size();
        int ID;
        int ID_2;
        int terrainMode;
        double AllDistance;
        string SecondPrintID;
        vector<ObstacleMap> Obstacle;
        elevation_mapping::FirstLayer OGF_First;
        std_msgs::Int32 AllID;

        /******************************Data Record************************************/
        ofstream file_1, file_2, file_3;
        string filepath = "/home/user/SYJ/map06_ws/src/data/"; //Path point reading
        string filename_1 = "FirstLayerData";
        string filename_2 = "ObstacleSurfaceState";
        string filename_3 = "SecondLayerData";
        filename_1 = filepath + filename_1 + Name;
        filename_2 = filepath + filename_2 + Name;
        filename_3 = filepath + filename_3 + Name;

        ros::Time Time_t = ros::Time::now();
        double CurrentTime = Time_t.toSec();

        if (!CCLabelMap.FirstLayer().empty())
        {

            Obstacle = CCLabelMap.FirstLayer();
            if(!Obstacle.empty())
            {
                double compareDistance = Obstacle[0].distanceWithRobotX;
                int distanceIndex = 0;
                for (int n = 0; n < Obstacle.size(); n++)
                {
                    if (compareDistance > Obstacle[n].distanceWithRobotX)
                    {
                        compareDistance = Obstacle[n].distanceWithRobotX;
                        distanceIndex = n;
                    }
                    else
                    {
                        continue;
                    }
                }
                

                ROS_INFO("Obstacle size is : %d", Obstacle.size());
                ROS_INFO("Mode ID is : %s", Obstacle[distanceIndex].modeID.c_str());
                OGF_First.modeID = Obstacle[distanceIndex].modeID.c_str();
                
                ModeIDMapping(Obstacle[distanceIndex].modeID, mapping, ID);
                OGF_First.ID = ID;
                AllID.data = ID;
                OGF_First.ObstacleType = Obstacle[distanceIndex].typeOfObstacle.c_str();
                OGF_First.height = Obstacle[distanceIndex].height;
                OGF_First.number = Obstaclenumber;
                OGF_First.wide = Obstacle[distanceIndex].wide;
                OGF_First.length = Obstacle[distanceIndex].length;
                OGF_First.density = CCLabelMap.GetDensity();
                double densityOfObstacle = CCLabelMap.GetDensity();
                AllDistance = CCLabelMap.GetRoverAndObstacleDistance();
                if(ID == 6)
                {
                    terrainMode = 0;
                }
                else
                {
                    terrainMode = 1;
                }

                ROS_INFO("Obstacle type is : %s", Obstacle[distanceIndex].typeOfObstacle.c_str());
                ROS_INFO("Obstacle Height is : %.4f", Obstacle[distanceIndex].height);
                ROS_INFO("Obstacle relation is : %s", Obstacle[distanceIndex].positionRelation.c_str());
                ROS_INFO("distance X is :%.4f, distance Y is :%.4f", Obstacle[distanceIndex].distanceWithRobotX, Obstacle[distanceIndex].distanceWithRobotY);
                ROS_INFO("First End");

                file_1.open(filename_1, std::ios::out | std::ios::app);
                file_1 << setprecision(16)<< CurrentTime << "\t" << Obstacle[distanceIndex].label << "\t" << Obstaclenumber << "\t" << Obstacle[distanceIndex].height << "\t" << Obstacle[distanceIndex].wide << "\t" << Obstacle[distanceIndex].length << "\t" << densityOfObstacle << "\t" << Obstacle[distanceIndex].typeOfObstacle << "\t" << Obstacle[distanceIndex].positionRelation << "\t" << Obstacle[distanceIndex].modeID << "\t" << ID <<"\t"<<AllDistance <<"\t" << terrainMode <<"\t"<< endl;
                file_1.close();


            /* ***************************************************SecondLayer**************************************************/    
                SecondLayer::SecondLayer SecondStage;
                ROS_INFO("Second Begin");
                SecondStage.SetTargetObstacleMap(Obstacle[distanceIndex]);
                SecondStage.SetResolution(resolutionData);
                vector<double> weight(4, 1);
                SecondStage.SetSNMargin(5,10,100);
                SecondStage.SetGRAParams(0.6, weight, 0.5);
                SecondStage.SetRobotPath(Path);
                SecondStage.SetWheelPath();
                SecondStage.SetRobotWide(0.83);
                SecondStage.SetMarginOfSharpness(0.01);
                SecondStage.SetSharpnessradius(2);
                vector<double> targetvector(4, 1);
                SecondStage.SetTargetVector(targetvector);
                SecondStage.SetAreaInPlane(0.2);

                string secondLayerModeID = SecondStage.SecondLayers(Obstacle[distanceIndex].modeID);
                ModeIDMapping(secondLayerModeID, mapping, ID_2);
                AllID.data = ID_2;

                vector<double> planeSa = SecondStage.GetSaInObstacle();
                vector<double> planeLevel = SecondStage.GetPlaneLevelInObstacle();
                vector<double> plane = SecondStage.GetAngleInObstacle();
                vector<double> plane_Out = SecondStage.GetAngleOutObstacle();
                vector<double> greyRelation = SecondStage.GetGreyRelationValue();

                if (secondLayerModeID == "WCM")
                {
                    SecondPrintID = "WCM";
                }
                else if (secondLayerModeID == "DWLM")
                {
                    SecondPrintID = "Contact";
                }
                else{
                    SecondPrintID = "No-Contact";
                }

                for (int x = 0; x < planeSa.size(); x++)
                {
                    file_2.open(filename_2, std::ios::out | std::ios::app);
                    file_2<< setprecision(16) << CurrentTime << "\t" << x << "\t" << planeSa[x] << "\t" << planeLevel[x] << "\t" << plane[x] << "\t" << plane_Out[x] << "\t" << greyRelation[x] << "\t" <<terrainMode<<"\t"<<SecondPrintID<<"\t"<< endl;
                    file_2.close();
                }
                

                vector<double> SaResult = SecondStage.GetSaResult();
                vector<double> PlaneLevelResult = SecondStage.GetPlaneLevelResult();
                vector<double> AngleInResult = SecondStage.GetAngleInResult();
                vector<double> AngleOutResult = SecondStage.GetAngleOutResult();
                vector<double> RelationResult = SecondStage.GetRelationResult();
                string fileWirte[3]={"LTrajectory","RTrajectory","NoneTrajectory"};
                for (int f = 0; f < SaResult.size(); f++)
                {
                    if (SaResult.size()>1)
                    {
                        file_3.open(filename_3, std::ios::out | std::ios::app);
                        file_3<< setprecision(16) << CurrentTime << "\t" << fileWirte[f] << "\t" << SaResult[f] << "\t" << PlaneLevelResult[f] << "\t" << AngleInResult[f] << "\t" << AngleOutResult[f] << "\t" << RelationResult[f] << "\t" <<secondLayerModeID<<"\t"<< terrainMode<<"\t"<<SecondPrintID<<"\t"<< endl;
                        file_3.close();
                    }
                    else{
                        file_3.open(filename_3, std::ios::out | std::ios::app);
                        file_3<< setprecision(16) << CurrentTime << "\t" << fileWirte[2] << "\t" << SaResult[f] << "\t" << PlaneLevelResult[f] << "\t" << AngleInResult[f] << "\t" << AngleOutResult[f] << "\t" << RelationResult[f] << "\t" <<secondLayerModeID<<"\t"<< terrainMode<<"\t"<<SecondPrintID<<"\t"<< endl;
                        file_3.close();
                    }
                }

                ROS_INFO("ID_2 is : %s",secondLayerModeID.c_str() );

                pub_.publish(OGF_First);//Change to publish the mode ID of the nearest obstacle
                modeIDPub_.publish(AllID);
            }
            else
            {
                ROS_INFO("Without Obstacle In Current Robot Range");
            }
        }
        else
        {
            ROS_INFO("Without Obstacle In Current Robot Range");
            file_1.open(filename_1, std::ios::out | std::ios::app);
            file_1<< setprecision(16)  << CurrentTime << "\t" << 0 << "\t" << 0 << "\t" << 0 << "\t" << 0<< "\t" << 0 << "\t" << "None"<< "\t" << "None" << "\t" << "None" << "\t" << "WRM" << "\t" << 0 <<"\t"<<0 <<"\t"<<"None"<<"\t"<< endl;
            file_1.close();
            modeIDPub_.publish(AllID);
        }

    }


    void GetCCLMapCallback(const grid_map_msgs::GridMap& message)
    {
    
        grid_map::GridMap LoadMap;
        GridMapRosConverter::fromMessage(message,LoadMap);
        double lengthX = LoadMap.getLength().x();
        double lengthY = LoadMap.getLength().y();
        double resolution = LoadMap.getResolution();
        const grid_map::Position& initPosition = LoadMap.getPosition();
        const int lengthInXSubmapI = static_cast<int>(lengthX / resolution + 2 * 0.3);
        const int lengthInYSubmapI = static_cast<int>( lengthY/ resolution + 2 *0.3);
        const grid_map::Position topLeftPosition(initPosition(0) + lengthX / 2, initPosition(1) + lengthY / 2);
        grid_map::Index submapTopLeftIndex;
        LoadMap.getIndex(topLeftPosition, submapTopLeftIndex);
        const Eigen::Array2i submapBufferSize(lengthInYSubmapI, lengthInXSubmapI);
        grid_map::Matrix& elevationData = LoadMap["elevation"];

        float heightData;
        float margin_data = 0.1;//m 
        float distanceMargin = 5;//m

        int numRow = 0;
        Index obstacleIndex;

        vector<vector<float>> orignElevationMap;
        vector< vector<int> > labelMap;
        vector<float> temp;
        Position position = LoadMap.getPosition();

        for (grid_map::SubmapIterator iterator(LoadMap, submapTopLeftIndex, submapBufferSize); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        heightData = elevationData(index(0), index(1));
        grid_map::Index size_In_map = iterator.getSubmapIndex();
        
            if (!isnan(heightData))
            {
                temp.push_back(heightData);
            }
            else{
                temp.push_back(0);
            }
        }

        //get 2D OrignElevationMap
        for (int i = 0; i < LoadMap.getSize()(0); i++)
        {
            vector<float> test;
            numRow = i * LoadMap.getSize()(1);
            test.assign(temp.begin() + numRow, temp.begin() + numRow + LoadMap.getSize()(1));
            orignElevationMap.push_back(test);
        }

        for (int i = 0; i < orignElevationMap.size(); i++)
        {
            for (int j = 0; j < orignElevationMap[0].size(); j++)
            {
                cout << orignElevationMap[i][j] << ",";
            }
            cout << endl;
        }
        

        if (motionState != 1)
        {
            StrategyStart(orignElevationMap, resolution, position(0), position(1));
        }
        else 
        {
            ROS_INFO("Waiting for comtestserver state");
        }
        
    }
};

vector<vector<double>> StringToDouble(vector<string> data)
{
    vector<vector<double>> result;
    for (int i = 0; i < data.size(); i++)
    {
        vector<double> tmp;
        for (int j = 0; j < data[i].size(); j++)
        {
            if (data[i][j] == ' ')
            {
                tmp.push_back(stod(data[i].substr(0, j)));
                tmp.push_back((-1)*stod(data[i].substr(j + 1, data[i].size() - j)));
                result.push_back(tmp);
                tmp.clear();
            }
            else
            {
                continue;
            }
        }
    }
    return result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ConnectMappub");
    vector<vector<double>> StringToDouble(vector<string> data);
    vector<double> tmp;

    time_t timep;
    static char name[256] = {0};
    time(&timep);

    strftime( name, sizeof(name), "%Y.%m.%d %H-%M-%S.txt",localtime(&timep) );

    Name = name;

    ofstream File_1, File_2, File_3;
    string filePath = "/home/user/SYJ/map06_ws/src/data/";
    string fileName_1 = "FirstLayerData";
    string fileName_2 = "ObstacleSurfaceState";
    string fileName_3 = "SecondLayerData";
    fileName_1 = filePath + fileName_1 + Name;
    fileName_2 = filePath + fileName_2 + Name;
    fileName_3 = filePath + fileName_3 + Name;
    File_1.open(fileName_1, std::ios::out | std::ios::app);
    File_1 <<"Time:"
           <<"\t" 
           << "Obstacle:"
           << "\t"
           << "ObstacleNumber:"
           << "\t"
           << "ObstacleHeight:"
           << "\t"
           << "ObstacleWide:"
           << "\t"
           << "ObstacleLength:"
           << "\t"
           << "ObstacleDensity:"
           << "\t"
           << "ObstacleType:"
           << "\t"
           << "PositionRelation:"
           << "\t"
           << "ModeID:"
           << "\t"
           <<"Distace:" 
            << "\t" 
           << "Stiffness:"
           <<"\t"<< endl;
    File_1.close();
    File_2.open(fileName_2, std::ios::out | std::ios::app);
    File_2 <<"Time:"
           <<"\t"
           << "ObstacleSurfacePath:"
           << "\t"
           << "PathSa:"
           << "\t"
           << "PathPlaneLevel:"
           << "\t"
           << "PathAngleIn:"
           << "\t"
           << "PathAngleOut"
           << "\t"
           << "GreyRelation:"
            << "\t" 
           << "Stiffness:"
            << "\t" 
           << "ModelType:"
           << "\t" << endl;
    File_2.close();
    File_3.open(fileName_3, std::ios::out | std::ios::app);
    File_3 <<"Time:"
           <<"\t"
           << "ObstacleSurfacePath:"
           << "\t"
           << "SaValue:"
           << "\t"
           << "PlaneLevelValue:"
           << "\t"
           << "AngleInValue:"
           << "\t"
           << "AngleOutValue:"
           << "\t"
           << "RelationResult:"
           << "\t" 
           << "ModeID:"
            << "\t" 
           << "Stiffness:"
            << "\t" 
           << "ModelType:"
           <<"\t"<< endl;
    File_3.close();
    
    Connectmap_pub PubConnectLabelMap;
    ros::spin();

    return 0;
}
