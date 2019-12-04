function [sys,x0,str,ts] = global_terminal_controller(t, x, u, flag)
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
sizes.NumInputs      = 2;   %输入变量个数
sizes.DirFeedthrough = 1;   %输入信号是否在输出子函数中出现
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %初始值
str = [];   
ts  = [];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';

function sys=mdlOutputs(t,x,u)   %计算输出子函数
x1 = u(1);
x2 = u(2);

alpha0 = 2;
beta0 = 1;
p0 = 7;
q0 = 5;
phi = 10;
gamma = 10;
p = 3;
q = 1;

fx = cos(x1);
gx = x1^2+1;

s0 = x1;
ds0 = x2;
s1 = ds0 + alpha0*s0 + beta0*abs(s0)^(q0/p0)*sign(s0);

T1 = abs(x1)^((q0-p0)/p0)*sign(x1);
T2 = abs(s1)^(q/p)*sign(s1);
ut = -(1/gx)*(fx + alpha0*x2 + beta0*q/p*T1*x2 + phi*s1 + gamma*T2);
sys(1) = ut;


