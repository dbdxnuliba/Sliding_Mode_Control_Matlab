function [sys,x0,str,ts] = sat_sign_controller(t, x, u, flag)
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;  % 调用初始化子函数
  case 1,
    sys=[];
  case 2,
    sys=[];
  case 3,
    sys=mdlOutputs(t,x,u);    %计算输出子函数
  case 4,
    sys=[];   %计算下一仿真时刻子函数
  case 9,
    sys=[];    %终止仿真子函数
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes   %初始化子函数

sizes = simsizes;

sizes.NumContStates  = 0;  %连续状态变量个数
sizes.NumDiscStates  = 0;  %离散状态变量个数
sizes.NumOutputs     = 1;  %输出变量个数
sizes.NumInputs      = 3;   %输入变量个数
sizes.DirFeedthrough = 1;   %输入信号是否在输出子函数中出现
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %初始值
str = [];   
ts  = [];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';

function sys=mdlOutputs(t,x,u)   %计算输出子函数
thd = u(1);
dthd = 0.1*cos(t);
ddthd = -0.1*sin(t);

x1 = u(2);
x2 = u(3);

mc = 1;
m = 0.1;
l = 0.5;
g = 9.8;

T = l*(4/3-m*cos(x1)*cos(x1)/(mc+m));
fx = g*sin(x1)-m*l*x2*x2*cos(x1)*sin(x1)/(mc+m);
fx = fx/T;
gx = cos(x1)/(mc+m);
gx = gx/T;

c = 1.5;
e = thd - x1;
de = dthd - x2;
s = c*e + de;

%使用sign控制输出抖动较为厉害，sat比较好
flag = 1;
ita = 20;

if flag == 0
    ut = 1/gx*(-fx+ddthd+c*de+ita*sign(s));
elseif flag == 1
    delta = 0.05;
    if abs(s) > delta
        ut = 1/gx*(-fx+ddthd+c*de+ita*sign(s));
    else
        ut = 1/gx*(-fx+ddthd+c*de+ita*1/delta*s);
    end
end

sys(1) = ut;


