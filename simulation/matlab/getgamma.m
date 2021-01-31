function [theta1,gamma1] = getgamma(x,y)
%UNTITLED 此处显示有关此函数的摘要
%   将x，y转换为极坐标的形式
    L1 = 0.09;
    L2 = 0.162;
    L = power((power(x,2.0)+power(y,2.0)),0.5);
    theta1 = atan2(x,y);
    cos_param = ( power(L1,2.0) + power(L,2.0) - power(L2,2.0))/(2.0*L1*L);
    gamma1 = acos(cos_param);
end

