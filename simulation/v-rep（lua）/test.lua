---
--- Generated by EmmyLua(https://github.com/EmmyLua)
--- Created by TerrorBlade.
--- DateTime: 2021/1/29 22:34
---

require("math")
require("socket")

function sleep(s)
    local ntime = socket.gettime() + s
    repeat until socket.gettime() > ntime
end
L1 = 0.09
L2 = 0.162
p = 0

GaitParams = {}
GaitParams["stance_height"] = 0.18
GaitParams["down_amp"] = 0.00
GaitParams["up_amp"] = 0.06
GaitParams["flight_percent"] = 0.6
GaitParams["step_length"] = 0.00
GaitParams["freq"] = 1.0
GaitParams["step_diff"] = 0.00

state_gait_params = {
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN}, -- STOP
    {0.17, 0.04, 0.06, 0.35, 0.15, 2.0, 0.0}, -- TROT
    {0.17, 0.04, 0.06, 0.35, 0.0, 2.0, 0.0}, -- BOUND
    {0.15, 0.00, 0.06, 0.25, 0.0, 1.5, 0.0}, -- WALK
    {0.12, 0.05, 0.0, 0.75, 0.0, 1.0, 0.0}, -- PRONK
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN}, -- JUMP
    {0.15, 0.05, 0.05, 0.35, 0.0, 1.5, 0.0}, -- DANCE
    {0.15, 0.05, 0.05, 0.2, 0.0, 1.0, 0.0}, -- HOP
    {NAN, NAN, NAN, NAN, NAN, 1.0, NAN}, -- TEST
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN}, -- ROTATE
    {0.15, 0.07, 0.06, 0.2, 0.0, 1.0, 0.0}, -- FLIP
    {0.17, 0.04, 0.06, 0.35, 0.1, 2.0, 0.06}, -- TURN_TROT
    {NAN, NAN, NAN, NAN, NAN, NAN, NAN} -- RESET
}




function changeGaitParams(params)
    GaitParams["stance_height"] = params[1]
    GaitParams["down_amp"] = params[2]
    GaitParams["up_amp"] = params[3]
    GaitParams["flight_percent"] = params[4]
    GaitParams["step_length"] = params[5]
    GaitParams["freq"] = params[6]
    GaitParams["step_diff"] = params[7]
end


--正弦函数生成器
--返回笛卡尔坐标中x，y
function SinTrajectory(params)
    local t_diff = 0.02
    local stanceHeight = params.stance_height
    local downAMP = params.down_amp
    local upAMP = params.up_amp
    local flightPercent = params.flight_percent
    local stepLength = params.step_length
    local FREQ = params.freq

    local x,y

    p = p+FREQ*t_diff
    local gp = math.fmod(p,1.0)

    if(gp <= flightPercent)
    then
         x = (gp/flightPercent)*stepLength - stepLength/2.0
         y = -upAMP*math.sin(math.pi*gp/flightPercent) + stanceHeight
    else
         local percentBack = (gp-flightPercent)/(1.0-flightPercent)
         x = -percentBack*stepLength + stepLength/2.0
         y = downAMP*math.sin(math.pi*percentBack) + stanceHeight
    end
    --print(p)
    return x,y

end

--将笛卡尔坐标系中的x，y转换成腿部参数L和theta
--返回L和theta
function CartesianToLegParams(x,y,leg_direction)
    local L,theta
    L = math.pow((math.pow(x,2.0) + math.pow(y,2.0)), 0.5)
    theta = math.atan2(leg_direction * x, y)
    return L,theta
end

--将腿部参数L，theta转换成电机的gamma值
--返回gamma
function GetGamma(L)
    local gamma
    local cos_param = (math.pow(L1,2.0) + math.pow(L,2.0) - math.pow(L2,2.0)) / (2.0*L1*L)
    if(cos_param < -1.0) then
        gamma = math.pi
    elseif cos_param > 1.0 then
        gamma = 0
    else
        gamma = math.acos(cos_param)

    end
    return gamma
end

function GetAngleWhenXequal0(x,y)
    local cos_angle
    local angle
    cos_angle = (math.pow(L1,2.0)+math.pow(y,2.0)-math.pow(L2,2.0)) / (2.0*L1*L)
    angle = math.acos(cos_angle)

    return angle


end

function straight_line()
    local x,y,t_diff
    x = 0
    p = p+t_diff

    if(y<0.17and y>-0.17)then
        y = y+0.02
    end
    if(y>0.17)then
        y = y- 0.02
    end
    if(y<-0.17)then
        y = y+0.02
    end

    return x,y
end


--将笛卡尔坐标系中x，y转换成对应电机的theta和gamma
--返回theta和gamma
function CartesianToThetaGamma(x,y,leg_direction)
    local L,theta
    local gamma
    L,theta = CartesianToLegParams(x,y,leg_direction)
    theta,gamma = GetGamma(L)

    return theta,gamma
end


changeGaitParams(state_gait_params[2])

while (true)
do
    local x,y
    local theta,gamma
    x,y = SinTrajectory(GaitParams)
    theta,gamma = CartesianToThetaGamma(x,y,1)
    print("theta =",x,"gamma =",y)
    --socket.sleep(1)

end

