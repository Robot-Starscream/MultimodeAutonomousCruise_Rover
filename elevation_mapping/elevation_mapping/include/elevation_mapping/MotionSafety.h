/*
 * MotionSafety.h
 *
 *  Created on: Dec 12, 2024
 *	 Institute: Harbin Institute of Technology, State Key Laboratory of Robotics and Systems
 */

#ifndef _MOTION_SAFETY_H_
#define _MOTION_SAFETY_H_

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <assert.h>
#include "elevation_mapping/MyMap.h"

using namespace std;

enum Operation {
	AND, OR, BEGIN, END
};

typedef struct {
	double roll, F_Llrw, pitch, F_Nfrw, delta_W_max = 0, W_fmr = 0, Acc_L = 0, Acc_Z = 0;
}InputData;

typedef struct {
	double A;
	double B;
	double C;
	double D;
	string s_valueName; //Linguistic value name, such as "small"
	double linguisticValue; //Value
}LinguisticValue;

typedef struct {
	double deFuzzyValue;
	vector<LinguisticValue> linguisticValueVector;
}LinguisticVariable;

typedef struct {
	Operation operation;
	LinguisticVariable* p_linguisticVariable;
	string linguisticValueName;
}FuzzyRuleToken;

typedef struct {
	vector<FuzzyRuleToken> v_fuzzyRuleTokens;
}FuzzyRule;


namespace MotionSafety
{
    // A fuzzy logic system to evaluate the safety of the rover’s motion while moving on the obstacle
    class MotionSafety
    {
    private:

        LinguisticVariable lateralForce;
        LinguisticVariable rollAngle;
        LinguisticVariable terrainRoughness;

        LinguisticVariable normalForce;
        LinguisticVariable pitchAngle;
        LinguisticVariable terrainSlope;

        LinguisticVariable complexity;

        LinguisticVariable maxDiffNormalForce;
        LinguisticVariable averageNormalForce;
        LinguisticVariable wheelAdaptability;

        LinguisticVariable additionalLateralAcc;
        LinguisticVariable additionalVerticalAcc;
        LinguisticVariable robotTrackSmoothness;

        LinguisticVariable adventureDegree;

        vector<FuzzyRule> roughnessRules;
        vector<FuzzyRule> slopeRules;
        vector<FuzzyRule> complexityRules;
        vector<FuzzyRule> adaptabilityRules;
        vector<FuzzyRule> smoothnessRules;
        vector<FuzzyRule> adventureRules_WheelAdap;
        vector<FuzzyRule> adventureRules_RobTrackSmooth;


        public:
        MotionSafety();
        
        void ResetAll();
        void ResetLinguisticVariable(LinguisticVariable* p_linguisticVar);
        vector<vector<double>> DesignMembershipFuction(const vector<double>& vec, double upRate);
        vector<vector<double>> DesignMembershipFuction(double leftBorder, double RightBorder, int setNum, double upRate);
        void AddLinguisticValue(LinguisticVariable& p_linguisticVariable, vector<vector<double>>& nums, const vector<string>& valueName);
        void AddTokenToRule(FuzzyRule& p_fuzzyRule, Operation operation, LinguisticVariable* linguisticVariable, string valueName);
        void AddTokensToRule(vector<FuzzyRule>& v_fuzzyRule, LinguisticVariable* first, LinguisticVariable* second, LinguisticVariable* result, const vector<string>& r_str);
        double CalculateLinguisticValue(LinguisticValue* p_linguisticValue, double input);
        void UpdateLinguisticValue(LinguisticValue* p_linguisticValue, double newLinguisticValue);
        LinguisticValue* FindLinguisticValueByName(LinguisticVariable* p_linguisticVariable, string valueName);
        vector<LinguisticValue*> OutputLinguisticValueList(LinguisticVariable& p_linguisticVariable);

        void VectorPushBack(vector<vector<double>>& vec, double a[][4]);
        void VectorPushBack(vector<double>& vec, double b[4]);
        double OrOperator(double A, double B);
        double AndOperator(double A, double B);
        double CalculateTrapezoid(LinguisticValue* p_linguisticValue);

        FuzzyRuleToken* ReasonFuzzyRule(FuzzyRule& v_fuzzyRule);
        double Defuzzy(LinguisticVariable& p_linguisticVar);
        double CalculateFuzzyRules(vector<FuzzyRule> v_fuzzyRules, LinguisticVariable& p_linguisticVar);

        string GetLinguisticValueNameAsOutput(const LinguisticVariable& p_linguisticVar)
        double RiskEvaluation_1(dInputData idata, double id, string &level);
        double RiskEvaluation_2(InputData idata, double id, string &level);
    };

} // namespace MotionSafety

#endif