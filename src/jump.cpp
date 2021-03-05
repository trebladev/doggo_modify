#include "jump.h"
#include "ODriveArduino.h"
#include "globals.h"
#include "position_control.h"

// Privates
float start_time_ = 0.0f;

/**
 * 通知控制线程开始跳跃
 * @param start_time_s The timestamp of when the jump command was sent
 */
void StartJump(float start_time_s) {
    start_time_ = start_time_s;
    state = JUMP;
}

/**
 * 线性增加跳跃的高度
 * @param t
 * @param launchTime
 * @param stanceHeight
 * @param downAMP
 * @param x
 * @param y
 */
void TrajectoryJump(float t, float launchTime, float stanceHeight,
    float downAMP, float& x, float& y) {
    //Need to check if n works
    float n = t/launchTime;
    x = 0;
    y = downAMP*n + stanceHeight;
    //y = downAMP*sin(PI/4 + PI/4*n) + stanceHeight;
}

/**
* Drives the ODrives in an open-loop, position-control sinTrajectory.
*/
void ExecuteJump() {
    // min radius = 0.8
    // max radius = 0.25
    const float prep_time = 0.5f; // 跳跃之前的时间 [s]
    const float launch_time = 0.8f ; // 收起腿之前的时间 [s]
    const float fall_time = 1.0f; //收起腿之后回到正常动作的时间 [s]

    const float stance_height = 0.081f; // 跳跃之前所需要的腿部延伸 [m]
    const float jump_extension = 0.249f; // 最大的腿部延伸 [m]
    const float fall_extension = 0.13f; // 下落时的腿部延伸 [m]

    float t = millis()/1000.0f - start_time_; // 跳跃信息发出之后的秒数

    if (t < prep_time) {
        float x = 0;
        float y = stance_height;
        float theta,gamma;
        CartesianToThetaGamma(x, y, 1.0, theta, gamma);

        // 使用低强度和高阻尼的增益设置
        struct LegGain gains = {50, 1.0, 50, 1.0};
        CommandAllLegs(theta,gamma,gains);
        // Serial << "Prep: +" << t << "s, y: " << y;
    } else if (t >= prep_time && t < prep_time + launch_time) {
        float x = 0;
        float y = jump_extension;
        float theta, gamma;
        CartesianToThetaGamma(x, y, 1.0, theta, gamma);

        // 用高强度和低阻尼来执行跳跃
        struct LegGain gains = {240, 0.5, 240, 0.2};
        CommandAllLegs(theta, gamma, gains);
        // Serial << "Jump: +" << t << "s, y: " << y;
    } else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time) {
        float x = 0;
        float y = fall_extension;
        float theta,gamma;
        CartesianToThetaGamma(x, y, 1.0, theta, gamma);

        // 用低强度和高阻尼来处理跌落
        struct LegGain gains = {50, 1.0, 50, 1.0};

        CommandAllLegs(theta, gamma, gains);
        // Serial << "Retract: +" << t << "s, y: " << y;
    } else {
        state = STOP;
        Serial.println("Jump Complete.");
    }
    // Serial << '\n';
}
