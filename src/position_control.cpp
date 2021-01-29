#include "position_control.h"
#include "Arduino.h"
#include "ODriveArduino.h"
#include "config.h"
#include "globals.h"
#include "jump.h"
#include <math.h>
#include "backflip.h"

//------------------------------------------------------------------------------
// 位置控制线程: 电机位置控制线程
// 定期计算PID控制器的计算结果并且发送给电机
// 当前到Odrive的命令

// TODO: add support for multiple ODrives
// void SetCoupledPositionPrime(ODriveArduino& odrv, float theta, float gamma) {
//   global_debug_values... = theta;
//
//   odrv.SetCoupledPosition(theta,gamma);
// }
THD_WORKING_AREA(waPositionControlThread, 512);

THD_FUNCTION(PositionControlThread, arg) {
    (void)arg;

    SetODriveCurrentLimits(CURRENT_LIM);
    chThdSleepMilliseconds(100);
    SetODriveCurrentLimits(CURRENT_LIM);

    while(true) {

        struct GaitParams gait_params = state_gait_params[state];

        switch(state) {
            case STOP:
                {
                    LegGain stop_gain = {50, 0.5, 50, 0.5};
                    float y1 = 0.15;
                    float y2 = 0.15;
                    float theta1, gamma1, theta2, gamma2;
                    CartesianToThetaGamma(0.0, y1, 1, theta1, gamma1);
                    CartesianToThetaGamma(0.0, y2, 1, theta2, gamma2);

                    odrv0Interface.SetCoupledPosition(theta2, gamma2, stop_gain);
                    odrv1Interface.SetCoupledPosition(theta1, gamma1, stop_gain);
                    odrv2Interface.SetCoupledPosition(theta1, gamma1, stop_gain);
                    odrv3Interface.SetCoupledPosition(theta2, gamma2, stop_gain);
                }
                break;
            case DANCE:
                gait(gait_params, 0.0, 0.5, 0.0, 0.5, gait_gains);
                break;
            case BOUND:
                gait(gait_params, 0.0, 0.5, 0.5, 0.0, gait_gains);
                break;
            case TROT:
                gait(gait_params, 0.0, 0.5, 0.0, 0.5, gait_gains);
                break;
            case TURN_TROT:
                gait(gait_params, 0.0, 0.5, 0.0, 0.5, gait_gains);
                break;
            case WALK:
                gait(gait_params, 0.0, 0.25, 0.75, 0.5, gait_gains);
                break;
            case PRONK:
                gait(gait_params, 0.0, 0.0, 0.0, 0.0, gait_gains);
                break;
            case JUMP:
                ExecuteJump();
                break;
            case ROTATE:
                {
                float theta,gamma;
                CartesianToThetaGamma(0, 0.24, 1.0, theta, gamma);
                float freq = 0.1;
                float phase = freq * (millis() - rotate_start)/1000.0f;
                theta = (-cos(2*PI * phase) + 1.0f) * 0.5 * 2 * PI;
                CommandAllLegs(theta, gamma, gait_gains);
                }
            case HOP:
                hop(gait_params);
                break;
            case FLIP:
                ExecuteFlip(gait_params);
                break;
            case RESET:
                reset();
                break;
            case TEST:
                test();
                break;
        }

        chThdSleepMicroseconds(1000000/POSITION_CONTROL_FREQ);
    }
}
long rotate_start = 0; // 命令旋转时的毫秒数
States state = STOP;

// {stance_height, down_AMP, up_AMP, flight_percent (proportion), step_length, FREQ}
/**
 * 不同步态下的参数
 * 数据格式为{stance_height, down_AMP, up_AMP, flight_percent (proportion), step_length, FREQ}
 */
struct GaitParams state_gait_params[] = {
    //{s.h, d.a., u.a., f.p., s.l., fr., s.d.}
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN}, // STOP
    {0.17, 0.04, 0.06, 0.35, 0.15, 2.0, 0.0}, // TROT
    {0.17, 0.04, 0.06, 0.35, 0.0, 2.0, 0.0}, // BOUND
    {0.15, 0.00, 0.06, 0.25, 0.0, 1.5, 0.0}, // WALK
    {0.12, 0.05, 0.0, 0.75, 0.0, 1.0, 0.0}, // PRONK
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN}, // JUMP
    {0.15, 0.05, 0.05, 0.35, 0.0, 1.5, 0.0}, // DANCE
    {0.15, 0.05, 0.05, 0.2, 0.0, 1.0, 0.0}, // HOP
    {NAN, NAN, NAN, NAN, NAN, 1.0, NAN}, // TEST
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN}, // ROTATE
    {0.15, 0.07, 0.06, 0.2, 0.0, 1.0, 0.0}, // FLIP
    {0.17, 0.04, 0.06, 0.35, 0.1, 2.0, 0.06}, // TURN_TROT
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN} // RESET
};
/**
 * 运动时的控制参数设置
 */
struct LegGain gait_gains = {80, 0.5, 50, 0.5};

/**
 * 对所有Odrive的两个电机设置电流限制
 * @param 限制电流
 * NOTE: sometimes a motor doesn't actually receive the command
 */
void SetODriveCurrentLimits(float limit) {
    odrv0Interface.SetCurrentLims(limit);
    odrv1Interface.SetCurrentLims(limit);
    odrv2Interface.SetCurrentLims(limit);
    odrv3Interface.SetCurrentLims(limit);
}


/**
* 这里设置髋部的角度为弧度，在向下的y方向为0，逆时针方向为正角度
*/

/**
 * 获取腿部参数，并返回腿部的伽马角
 * @param L
 * @param theta
 * @param gamma
 */
void GetGamma(float L, float theta, float& gamma) {
    float L1 = 0.09; // upper leg length (m)
    float L2 = 0.162; // lower leg length (m)
    float cos_param = (pow(L1,2.0) + pow(L,2.0) - pow(L2,2.0)) / (2.0*L1*L);
    if (cos_param < -1.0) {
        gamma = PI;
        #ifdef DEBUG_HIGH
        Serial.println("ERROR: L is too small to find valid alpha and beta!");
        #endif
      } else if (cos_param > 1.0) {
        gamma = 0;
        #ifdef DEBUG_HIGH
        Serial.println("ERROR: L is too large to find valid alpha and beta!");
        #endif
      } else {
        gamma = acos(cos_param);
      }
}

/**
* 将支撑腿的参数L,gamma转换为笛卡尔坐标系（x，y）
* 将x方向设置为1.0&-1.0来改变走路时的方向
*/
void LegParamsToCartesian(float L, float theta, float leg_direction, float& x, float& y) {
    x = leg_direction * L * cos(theta);
    y = L * sin(theta);
}

/**
 * 将笛卡尔坐标系（x, y (m)) 转换为腿部参数（L (m), theta (rad))
 * @param x
 * @param y
 * @param leg_direction
 * @param L
 * @param theta
 */
void CartesianToLegParams(float x, float y, float leg_direction, float& L, float& theta) {
    L = pow((pow(x,2.0) + pow(y,2.0)), 0.5);
    theta = atan2(leg_direction * x, y);
}

/**
 * 正弦轨迹发生器函数，具有以下所述参数的灵活性。可以用这个做4拍，2拍小跑等
 * @param t
 * @param params
 * @param gaitOffset
 * @param x
 * @param y
 */
void SinTrajectory (float t, struct GaitParams params, float gaitOffset, float& x, float& y) {
    static float p = 0;
    static float prev_t = 0;

    float stanceHeight = params.stance_height;     //直立高度
    float downAMP = params.down_amp;               //下幅值
    float upAMP = params.up_amp;                   //上幅值
    float flightPercent = params.flight_percent;   //飞行相占比
    float stepLength = params.step_length;         //整步长
    float FREQ = params.freq;                      //频率

    p += FREQ * (t - prev_t < 0.5 ? t - prev_t : 0); // 当开始一个步态的时候应该减少蹒跚
                                                     // EXP1?EXP2:EXP3 如果EXP1为真，返回EXP2;如果EXP1为假，返回EXP3的值
                                                     // t-prev_t为执行一次控制的时间
    prev_t = t;

    float gp = fmod((p+gaitOffset),1.0); // mod(a,m) 返回a除以m的余数
    if (gp <= flightPercent) {                                     // 下半轨迹
        x = (gp/flightPercent)*stepLength - stepLength/2.0;
        y = -upAMP*sin(PI*gp/flightPercent) + stanceHeight;
    }
    else {                                                         // 上半轨迹
        float percentBack = (gp-flightPercent)/(1.0-flightPercent);
        x = -percentBack*stepLength + stepLength/2.0;
        y = downAMP*sin(PI*percentBack) + stanceHeight;
    }
}

/**
 * 将笛卡尔坐标系的坐标转换为θ和γ
 * @param x
 * @param y
 * @param leg_direction
 * @param theta
 * @param gamma
 */
void CartesianToThetaGamma(float x, float y, float leg_direction, float& theta, float& gamma) {
    float L = 0.0;
    CartesianToLegParams(x, y, leg_direction, L, theta);   //将x，y转换为虚拟腿长L和虚拟腿长相对于坐标轴的角度θ
    GetGamma(L, theta, gamma);                                //通过L和θ计算得到γ
    //Serial << "Th, Gam: " << theta << " " << gamma << '\n';
}

/**
 * 对Odirve耦合增益执行全面检查以防止明显的危险和不稳定性
 * @param  需要检查的腿部增益
 * @return  如果检查无误返回true，如果检查有误返回false
 */
bool IsValidLegGain(struct LegGain gains) {
    // check for unstable gains
    bool bad =  gains.kp_theta < 0 || gains.kd_theta < 0 ||
                gains.kp_gamma < 0 || gains.kd_gamma < 0;
    if (bad) {
        Serial.println("Invalid gains: <0");
        return false;
    }
    // check for instability / sensor noise amplification
    bad = bad || gains.kp_theta > 320 || gains.kd_theta > 10 ||
                 gains.kp_gamma > 320 || gains.kd_gamma > 10;
    if (bad) {
        Serial.println("Invalid gains: too high.");
        return false;
    }
    // check for underdamping -> instability
    bad = bad || (gains.kp_theta > 200 && gains.kd_theta < 0.1);
    bad = bad || (gains.kp_gamma > 200 && gains.kd_gamma < 0.1);
    if (bad) {
        Serial.println("Invalid gains: underdamped");
        return false;
    }
    return true;
}

/**
 * 对控制参数进行检查
 * @param params
 * @return 所有参数符合标准返回true，有不符合的参数返回false
 */
bool IsValidGaitParams(struct GaitParams params) {
    const float maxL = 0.25;
    const float minL = 0.08;

    float stanceHeight = params.stance_height;
    float downAMP = params.down_amp;
    float upAMP = params.up_amp;
    float flightPercent = params.flight_percent;
    float stepLength = params.step_length;
    float FREQ = params.freq;

    if (stanceHeight + downAMP > maxL || sqrt(pow(stanceHeight, 2) + pow(stepLength / 2.0, 2)) > maxL) {
        Serial.println("Gait overextends leg");
        return false;
    }
    if (stanceHeight - upAMP < minL) {
        Serial.println("Gait underextends leg");
        return false;
    }

    if (flightPercent <= 0 || flightPercent > 1.0) {
        Serial.println("Flight percent is invalid");
        return false;
    }

    if (FREQ < 0) {
        Serial.println("Frequency cannot be negative");
        return false;
    }

    if (FREQ > 10.0) {
        Serial.println("Frequency is too high (>10)");
        return false;
    }

    return true;
}

/**
 * 命令给定的对象沿着正弦轨迹动作
 * TODO finish documentation
 * @param odrive        [控制的时哪个odrive]
 * @param t             [系统时间信号，控制生成步态信号的时钟精度]
 * @param params        [步态的参数]
 * @param gait_offset   [腿部的轨迹偏移]
 * @param leg_direction [腿的正方向，等于1时为正方向，-1时为负方向]
 * @param gains         [角度pd控制的参数]
 * @param theta         [θ角]
 * @param gamma         [γ角]
 */
void CoupledMoveLeg(ODriveArduino& odrive, float t, struct GaitParams params,
                    float gait_offset, float leg_direction, struct LegGain gains,
                    float& theta, float& gamma) {
    float x; // float x for leg 0 to be set by the sin trajectory
    float y;
    SinTrajectory(t, params, gait_offset, x, y);
    CartesianToThetaGamma(x, y, leg_direction, theta, gamma);
    odrive.SetCoupledPosition(theta, gamma, gains);
}

/**
 * 设置具体的步态
 * @param params        [步态中的具体的参数，站立高度，频率等]
 * @param leg0_offset   [0号腿的步态偏置]
 * @param leg1_offset   [1号腿的步态偏置]
 * @param leg2_offset   [2号腿的步态偏置]
 * @param leg3_offset   [3号腿的步态偏置]
 * @param gains         [具体步态]
 */
void gait(struct GaitParams params,
                float leg0_offset, float leg1_offset,
                float leg2_offset, float leg3_offset,
                struct LegGain gains) {

    struct GaitParams paramsR = params;
    struct GaitParams paramsL = params;
    paramsR.step_length -= params.step_diff;
    paramsL.step_length += params.step_diff;

    if (!IsValidGaitParams(paramsR) || !IsValidGaitParams(paramsL) || !IsValidLegGain(gains)) {
        return;
    }

    float t = millis()/1000.0;

    const float leg0_direction = -1.0;
    CoupledMoveLeg(odrv0Interface, t, paramsL, leg0_offset, leg0_direction, gains,
        global_debug_values.odrv0.sp_theta, global_debug_values.odrv0.sp_gamma);

    const float leg1_direction = -1.0;
    CoupledMoveLeg(odrv1Interface, t, paramsL, leg1_offset, leg1_direction, gains,
        global_debug_values.odrv1.sp_theta, global_debug_values.odrv1.sp_gamma);

    const float leg2_direction = 1.0;
    CoupledMoveLeg(odrv2Interface, t, paramsR, leg2_offset, leg2_direction, gains,
        global_debug_values.odrv2.sp_theta, global_debug_values.odrv2.sp_gamma);

    const float leg3_direction = 1.0;
    CoupledMoveLeg(odrv3Interface, t, paramsR, leg3_offset, leg3_direction, gains,
        global_debug_values.odrv3.sp_theta, global_debug_values.odrv3.sp_gamma);
}

/**
 * 同时控制所有腿的状态
 * @param theta [θ角]
 * @param gamma [γ角]
 * @param gains [步态]
 */
void CommandAllLegs(float theta, float gamma, LegGain gains) {
    odrv0Interface.SetCoupledPosition(theta, gamma, gains);
    odrv1Interface.SetCoupledPosition(theta, gamma, gains);
    odrv2Interface.SetCoupledPosition(theta, gamma, gains);
    odrv3Interface.SetCoupledPosition(theta, gamma, gains);
    global_debug_values.odrv0.sp_theta = theta;
    global_debug_values.odrv0.sp_gamma = gamma;
    global_debug_values.odrv1.sp_theta = theta;
    global_debug_values.odrv1.sp_gamma = gamma;
    global_debug_values.odrv2.sp_theta = theta;
    global_debug_values.odrv2.sp_gamma = gamma;
    global_debug_values.odrv3.sp_theta = theta;
    global_debug_values.odrv3.sp_gamma = gamma;
}


/**
 * 更新Stop状态的步态参数
 * @param curr_state
 */
void UpdateStateGaitParams(States curr_state) {
    if (!isnan(state_gait_params[STOP].stance_height)) {
        state_gait_params[curr_state].stance_height = state_gait_params[STOP].stance_height;
        state_gait_params[STOP].stance_height = NAN;
    }
    if (!isnan(state_gait_params[STOP].down_amp)) {
        state_gait_params[curr_state].down_amp = state_gait_params[STOP].down_amp;
        state_gait_params[STOP].down_amp = NAN;
    }
    if (!isnan(state_gait_params[STOP].up_amp)) {
        state_gait_params[curr_state].up_amp = state_gait_params[STOP].up_amp;
        state_gait_params[STOP].up_amp = NAN;
    }
    if (!isnan(state_gait_params[STOP].flight_percent)) {
        state_gait_params[curr_state].flight_percent = state_gait_params[STOP].flight_percent;
        state_gait_params[STOP].flight_percent = NAN;
    }
    if (!isnan(state_gait_params[STOP].step_length)) {
        state_gait_params[curr_state].step_length = state_gait_params[STOP].step_length;
        state_gait_params[STOP].step_length = NAN;
    }
    if (!isnan(state_gait_params[STOP].freq)) {
        state_gait_params[curr_state].freq = state_gait_params[STOP].freq;
        state_gait_params[STOP].freq = NAN;
    }
    if (!isnan(state_gait_params[STOP].step_diff)) {
        state_gait_params[curr_state].step_diff = state_gait_params[STOP].step_diff;
        state_gait_params[STOP].step_diff = NAN;
    }
}

/**
 * 过渡到舞蹈步态的参数
 */
void TransitionToDance() {
    state = DANCE;
    Serial.println("DANCE");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.15, 0.05, 0.05, 0.35, 0.0, 1.5};
    UpdateStateGaitParams(DANCE);
    gait_gains = {50, 0.5, 30, 0.5};
    PrintGaitParams();
}
/**
* 过渡到普朗克步态的参数
*/
void TransitionToPronk() {
    state = PRONK;
    Serial.println("PRONK");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.12, 0.05, 0.0, 0.75, 0.0, 1.0};
    UpdateStateGaitParams(PRONK);
    gait_gains = {80, 0.50, 50, 0.50};
    PrintGaitParams();
}

/**
* 过渡原地步态的参数
*/
void TransitionToBound() {
    state = BOUND;
    Serial.println("BOUND");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.17, 0.04, 0.06, 0.35, 0.0, 2.0};
    UpdateStateGaitParams(BOUND);
    gait_gains = {80, 0.5, 50, 0.5};
    PrintGaitParams();
}

/**
 * 过渡走路步态的参数
 */
void TransitionToWalk() {
    state = WALK;
    Serial.println("WALK");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.15, 0.00, 0.06, 0.25, 0.0, 1.5};
    UpdateStateGaitParams(WALK);
    gait_gains = {80, 0.5, 50, 0.5};
    PrintGaitParams();
}

/**
* 过渡小跑步态的参数
*/
void TransitionToTrot() {
    state = TROT;
    Serial.println("TROT");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.17, 0.04, 0.06, 0.35, 0.15, 2.0};
    UpdateStateGaitParams(TROT);
    state_gait_params[TROT].step_diff = 0.0; // TROT should always go straight
    gait_gains = {80, 0.5, 50, 0.5};
    PrintGaitParams();
}

/**
* 过渡小跑转弯的步态的参数
*/
void TransitionToTurnTrot() {
    state = TURN_TROT;
    Serial.println("TURN_TROT");
    //            {s.h, d.a., u.a., f.p., s.l., fr., sd.}
    //gait_params = {0.17, 0.04, 0.06, 0.35, 0.1, 2.0, 0.06};
    UpdateStateGaitParams(TURN_TROT);
    gait_gains = {80, 0.5, 80, 0.5};
    PrintGaitParams();
}

/**
 * 过渡到Rotate步态的参数
 */
void TransitionToRotate() {
    state = ROTATE;
    rotate_start = millis();
    Serial.println("ROTATE");
    gait_gains = {30,0.5,30,0.5};
}

/**
 * 过渡到Hop步态的参数
 */
void TransitionToHop() {
    state = HOP;
    Serial.println("HOP");
    //            {s.h, d.a., u.a., f.p., s.l., fr.}
    //gait_params = {0.15, 0.05, 0.05, 0.2, 0, 1.0};
    UpdateStateGaitParams(HOP);
    PrintGaitParams();
}

void test() {
    /* Downwards force test */
    // struct LegGain gains = {0.0, 0.0, 40.0, 0.5};
    // odrv0Interface.SetCoupledPosition(0, PI/3.0f, gains);
    // odrv0Interface.ReadCurrents();

    /* Upwards weight test */
    // struct LegGain gains = {0.0, 0.0, 40.0, 0.5};
    // odrv0Interface.SetCoupledPosition(0, 2.0f*PI/3.0f, gains);
    // odrv0Interface.ReadCurrents();

    /* Step function force test */
    // float low = 20.0f; // corresponds to 5.23A if error is pi/6
    // float high = 80.0f; // corresponds to 20.94A if error is pi/6
    // float mid = (low + high)/2.0f;
    // float amp = high - mid;
    // struct LegGain gains = {0.0, 0.0, low + amp * ((int)(millis()/2000) % 2), 0.5};
    // odrv0Interface.SetCoupledPosition(0, 2.0*PI/3.0, gains);
    // odrv0Interface.ReadCurrents();

    // float low = 20.0f; // corresponds to 5.23A if error is pi/6
    // float high = 80.0f; // corresponds to 20.94A if error is pi/6
    // float mid = (low + high)/2.0f;
    // float amp = high - mid;
    // float phase = millis()/1000.0 * 2 * PI * state_gait_params[state].freq;
    // float gamma_kp = mid + amp * sin(phase);
    // struct LegGain gains = {0.0, 0.0, gamma_kp, 0.5};
    // odrv0Interface.SetCoupledPosition(0, 2.0*PI/3.0, gains);
    //
    // global_debug_values.odrv0.sp_theta = 0;
    // global_debug_values.odrv0.sp_gamma = 2.0*PI/3.0;
    //
    // float gamma_err = global_debug_values.odrv0.sp_gamma - global_debug_values.odrv0.est_gamma;
    // float gamma_torque = gamma_kp * gamma_err;
    //
    // gamma_torque = constrain(gamma_torque, -CURRENT_LIM*2.0f, CURRENT_LIM * 2.0f);

    // gamma_current = gamma*kp
    // gamma_current = 10.47
    // link_1_force = gamma_current/2 * 3 * 0.028 / 0.09
    // link_1_force = 0.467 * gamma_current
    // link_1_force at pi/6 error = kp * pi/6 * 0.467
    // link_1_force = 0.245* kp
    // min force = 4.9N, max force = 19.6N
    //
    // float I_m0 = gamma_torque*0.5; // motor 0 current
    // float I_m1 = gamma_torque*0.5;
    //
    // // If both motors are pushing down with 1A, then the leg force is 1.867N
    // float I_to_foot_force = 1.867;
    // float I_to_link_force = I_to_foot_force/2.0;
    // float link_force = I_to_link_force * I_m0;
    //
    // // NOTE: printing here
    // Serial << "Kp_I_F:\t" << gamma_kp << "\t" << gamma_torque << "\t" << link_force << "\n";


    float phase = millis()/1000.0 * 2 * PI * state_gait_params[state].freq;
    float amp = 1.0;
    float current = amp * sin(phase);
    odrv0Interface.SetCurrent(0, 1.0);
    Serial.println(current);
}

void hop(struct GaitParams params) {
    float freq = params.freq;
    struct LegGain hop_gains = {120, 1, 80, 1};
    struct LegGain land_gains = {120, 2, 20, 2};
    float theta, gamma;

    CartesianToThetaGamma(0, params.stance_height - params.up_amp, 1, theta, gamma);
    CommandAllLegs(theta, gamma, land_gains);
    chThdSleepMicroseconds(1000000*0.2/freq);

    CartesianToThetaGamma(0, params.stance_height + params.down_amp, 1, theta, gamma);
    CommandAllLegs(theta, gamma, hop_gains);
    chThdSleepMicroseconds(1000000*params.flight_percent/freq);

    CartesianToThetaGamma(0, params.stance_height, 1, theta, gamma);
    CommandAllLegs(theta, gamma, land_gains);
    chThdSleepMicroseconds(1000000*(0.8-params.flight_percent)/freq);


}

void reset() {
    gait_gains = {80, 0.5, 50, 0.5};

    struct LegGain retract_gains = {0, 0.5, 6, 0.1};
    float theta, gamma;
    CartesianToThetaGamma(0, 0.08, 1, theta, gamma);

    CommandAllLegs(theta, gamma, retract_gains);
    chThdSleepMilliseconds(4000);

    struct LegGain rotate_gains = {6, 0.1, 2, 1};
    CommandAllLegs(theta, gamma, rotate_gains);
    chThdSleepMilliseconds(4000);

    CartesianToThetaGamma(0, 0.17, 1, theta, gamma);
    struct LegGain extend_gains = {6, 0.1, 6, 0.1};
    CommandAllLegs(theta, gamma, extend_gains);
    chThdSleepMilliseconds(4000);

    Serial.println("4");
    chThdSleepMilliseconds(1000);
    Serial.println("3");
    chThdSleepMilliseconds(1000);
    Serial.println("2");
    chThdSleepMilliseconds(1000);
    Serial.println("1");
    chThdSleepMilliseconds(1000);

    state = STOP;
}

/**
 * 输出步态的参数，串口打印
 */
void PrintGaitParams() {
    Serial << ("(f)req: ") << state_gait_params[state].freq << '\n';
    Serial << ("step (l)ength: ") << state_gait_params[state].step_length << '\n';
    Serial << ("stance (h)eight: ") << state_gait_params[state].stance_height << '\n';
    Serial << ("(d)own amplitude: ") << state_gait_params[state].down_amp << '\n';
    Serial << "(u)p amplitude: " << state_gait_params[state].up_amp << '\n';
    Serial << ("flight (p)roportion: ") << state_gait_params[state].flight_percent << '\n';
    Serial << ("(s)tep difference: ") << state_gait_params[state].step_diff << '\n';
    Serial << "Theta: " << gait_gains.kp_theta << " " << gait_gains.kd_theta << '\n';
    Serial << "Gamma: " << gait_gains.kp_gamma << " " << gait_gains.kd_gamma << '\n';
}
