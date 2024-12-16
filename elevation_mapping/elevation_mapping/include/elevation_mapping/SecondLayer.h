/*
 * SecondLayer.h
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#ifndef _SECOND_LAYER_H
#define _SECOND_LAYER_H

#include <vector>
#include <iostream>
#include <map>
#include <algorithm>
#include <cmath>
#include "elevation_mapping/MyMap.h"
#include "elevation_mapping/ConnectAeras.h"
#include "elevation_mapping/GRA.h"
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using namespace Grey;
using namespace ConnectComponentLabeling;

namespace SecondLayer
{
    class SecondLayer
    {
    private:

        ObstacleMap Obstacle; // Target obstacle

        ConnectComponentLabeling::ConnectAeras CCL; // Connected region object

        vector< vector<float> > convolution_map_; // Convolution map

        vector< vector<float> > Map; // Original map

        vector< vector<float> > Map_And_Edge; // Map with edges

        map<char, float> Index; // Neighbor value mapping

        int anchorPoint; // Anchor point value

        float sharpnessMargin; // Sharp bump convolution difference threshold

        double sharpnessRadiusMargin; // Sharp bump radius threshold

        double resolution;

        vector<vector<int>> convolutionLabelSubMap; // Convolution map with labels after connected region analysis

        vector<vector<NewMap>> NewConvolutionMap;

        vector<Bumps> convolutionSubMap;

        int numberOfSharpBumps;

        double SNmargin_1, SNmargin_2, SNmargin_3;

        vector<int> bumpsLabel;

        int areaInPlane;

        vector<double> PlaneSa; // Average sharpness of each plane to be detected

        vector<double> PlaneSaOrigin;

        // Plane matching
        vector<vector<float>> wheelPlane; // Wheel plane

        double robotWide;

        vector<double> PlaneLevel;

        double planeWight_1;
        double planeWight_2;
        double planeWight_3;
        double planeWight_4;

        // Plane fitting
        vector<double> angleIn;

        vector<double> angleOut;

        vector<double> angleInOrigin;

        vector<double> angleOutOrigin;

        GRA gra;

        vector<double> targetSequence;

        double graMargin;

        vector<double> graweight;

        double graRow;

        vector<double> relation;

        vector<PATH> movementPath;

        double moveDistance = 0;

        vector<vector<double>> pathPosition;

	    vector<vector<double>> rightWheelPath;

	    vector<vector<double>> leftWheelPath;

        vector<double> saResult;
        vector<double> planeLevelResult;
        vector<double> angleInResult;
        vector<double> angleOutResult;
        vector<double> relationResult;

    public:
        SecondLayer();

        void SetTargetObstacleMap(ObstacleMap obstacle); // Read the obstacle obtained from the first layer

        void TurnMapType();

        void SetResolution(double resolution_);

        void SetMarginOfSharpness(float sharpness);

        void SetSharpnessradius(double radius);

        void SetAreaInPlane(double wideOfPlane);

        void SetRobotWide(double robotwide_);

        void SetTargetVector(vector<double> targetVector);

        void SetGRAParams(double margin_, vector<double> weight_, double row_);

        void SetRobotPath(vector<vector<double>> position);

        void SetWheelPath();

        void SetSNMargin(double margin_1, double margin_2, double margin_3);

        vector<int> SearchMaxIndex(int label_, float maxValue);

        void SearchMin(float &max, float &min);

        float FindMin(float input_1,float input_2, float input_3,float input_4);

        float HeightMappingCell(float height); // Map height value to pixel value

        void SetAnchorPoint(int anchorPointValue);

        vector< vector<float> > GetConvolutionmap(); // Get convolution map

        float Convalution_Compute(map<char, float> cell); // Compute convolution value

        void ConvalutionSynthesis(); // Associate neighborhood values

        void SetMap(vector< vector<float> > Oragin_map); // Get the original map from outside the class

        bool Edge_(vector< vector<float> > test_Map);

        void Sharpen(); // Sharpening process

        void Convolution();

        void ConvolutionSubMap(int label_, int num);

        double SharpBumpsRadius(int label_, int num);

        float ComputeBumpsHeight(int label_);

        vector<int> SearchMaxHeightIndex(int label_, float maxHeight);

        double forwardOnce(double input);

        bool SharpnessIndex();

        void UpdateNewConvolutionMap();

        void AreaOfSharpBumps(int num);

        double DensityOfBumps(int num, int beginIndex);

        void Steepness(int num);

        vector<int> neighbourhood(int raw, int col, int h_raw, int h_col, int step);

        double AverageSN(int size, int beginIndex);

        double AllDesity(int size, int beginIndex);

        void Sa();
        
        // Plane matching
        void CreateWheelPlane();

        double Function(int size, int y);

        vector<vector<float>> DiffHeight(int beginIndex);

        double ComputePlaneLevel(int beginIndex);

        double ComputePlaneLevel2(int beginIndex);

        float MaxDiff(vector<vector<float>> diffmap);

        float MinDiff(vector<vector<float>> diffmap);

        void Plane();

        bool gFittingPlane(vector<double> x, vector<double> y, vector<double> z, int n, double& a, double& b, double& c);

        vector<double> FittingLevel(int beginindex);

        void Fitting();

        void SurfaceShape();

        string SecondLayers(string ID);

        void UpdateMovementPath();

        vector<PATH> RelationValueMax(vector<PATH> input);

        vector<double> PathDistance(int num);

        vector<double> GetSaInObstacle();

        vector<double> GetPlaneLevelInObstacle();

        vector<double> GetAngleInObstacle();

        vector<double> GetGreyRelationValue();

        vector<double> GetAngleOutObstacle();

        double AverageValue(vector<double> input);

        vector<double> GetSaResult();
        vector<double> GetPlaneLevelResult();
        vector<double> GetAngleInResult();
        vector<double> GetAngleOutResult();
        vector<double> GetRelationResult();
    };

} // namespace ConnectAeras


#endif