/*
 * GRA.h
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#ifndef _GRA_H
#define _GRA_H

#include<iostream>
#include <vector>
#include<map>
#include <algorithm>

using namespace std;

namespace Grey
{
	// Grey relation analysis to evaluate the safety of the rover’s motion while moving on the obstacle
	class GRA
	{
	public:
		GRA();
		void SetTargetSequence(vector<double> target_);
		void SetIndexSequence(vector<vector<double>> index_);
		void SetWeight(vector<double> w);
		double PolarityMin(vector<double> targetIndex_);
		double PolarityMax(vector<double> targetIndex_);
		double IndexMax(vector<double> targetIndex_);
		double IndexMin(vector<double> targetIndex_);
		void ForwardProcessing(vector<double> &index_, int num);
		void MaximumDimensionless(vector<double> &index_);
		void MinimumDimensionless(vector<double>& index_);
		void PushIndex(vector<double> index_);
		vector<vector<double>> TransPose(vector<vector<double>> index_);
		void GreyRelationCompute();
		vector<double> GetGreyRelation();
		vector<vector<double>> GetRelationMatrix();


	private:
		vector<double> weight;
		double row;
		vector<double> targetSequence;
		vector<vector<double>> indexSequence;
		vector<vector<double>> relationMatrix;
		vector<double> relation;

	};
} // namespace GRA



#endif