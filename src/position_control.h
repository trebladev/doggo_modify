#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "ChRt.h"
#include "ODriveArduino.h"

extern THD_WORKING_AREA(waPositionControlThread, 512);
extern THD_FUNCTION(PositionControlThread, arg);

void GetGamma(float L, float theta, float& gamma);
void LegParamsToCartesian(float L, float theta, float& x, float& y);
void CartesianToLegParams(float x, float y, float leg_direction, float& L, float& theta);
void CartesianToThetaGamma(float x, float y, float leg_direction, float& theta, float& gamma);
void SinTrajectory (float t, struct GaitParams params, float gaitOffset, float& x, float& y);
void CoupledMoveLeg(ODriveArduino& odrive, float t, struct GaitParams params, float gait_offset, float leg_direction, struct LegGain gains);
bool IsValidGaitParams(struct GaitParams params);
bool IsValidLegGain(struct LegGain gain);
void SinTrajectoryPosControl();
void gait(struct GaitParams params, float leg0_offset, float leg1_offset, float leg2_offset, float leg3_offset, struct LegGain gains);
void TransitionToDance();
void TransitionToWalk();
void TransitionToTrot();
void TransitionToTurnTrot();
void TransitionToPronk();
void TransitionToBound();
void TransitionToRotate();
void TransitionToHop();
void PrintGaitParams();
void SetODriveCurrentLimits(float limit);
void test();
void hop(struct GaitParams params);
void reset();
void CommandAllLegs(float theta, float gamma, struct LegGain gains);

enum States {
    STOP = 0,
    TROT = 1,
    BOUND = 2,
    WALK = 3,
    PRONK = 4,
    JUMP = 5,
    DANCE = 6,
    HOP = 7,
    TEST = 8,
    ROTATE = 9,
    FLIP = 10,
    TURN_TROT = 11,
    RESET = 12
};

void UpdateStateGaitParams(States curr_state);

extern States state;

struct GaitParams {
    float stance_height = 0.18; // 行走时期望身体距离地面的高度 (m)
    float down_amp = 0.00; // 正弦轨迹中低于stanceheight的峰值振幅 (m)
    float up_amp = 0.06; // 正弦轨迹中，脚在高于stanceheight的峰值 (m)
    float flight_percent = 0.6; // 在步态轨迹的下半部分时间
    float step_length = 0.0; // 整步长度 (m)
    float freq = 1.0; // 一个步态周期的频率 (Hz)
    float step_diff = 0.0; //左右腿部的步长差
};

extern struct GaitParams state_gait_params[13];
extern struct LegGain gait_gains;
extern long rotate_start; // milliseconds when rotate was commanded

#endif
