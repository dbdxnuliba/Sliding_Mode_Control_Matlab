function [sys,x0,str,ts] = robustness_backstepping_controller(t, x, u, flag)
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
sizes.NumOutputs     = 3;  %输出变量个数
sizes.NumInputs      = 3;   %输入变量个数
sizes.DirFeedthrough = 1;   %输入信号是否在输出端出现
sizes.NumSampleTimes = 0;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [];   %初始值
str = [];   
ts  = [];   %[0 0]用于连续系统，[-1 0]表示继承其前的采样时间设置
simStateCompliance = 'UnknownSimState';

function sys=mdlOutputs(t,x,u)   %计算输出子函数
thd = u(1);
dthd = cos(t);
ddthd = -sin(t);

x1 = u(2);
dx1 = u(3);
f = x1^5-x1^6;
x2 = dx1 - f;
df = 5*x1^4*dx1-6*x1^5*dx1;

z1 = x1 - thd;
dz1 = dx1 - dthd;

k1 = 50;
h = 20;
beta = 1.5;
c1 = 70;
F_bar = 2;

z2 = x2 + c1*z1 + f - dthd;
sigma = k1*z1 + z2;

ut = -k1*(f + x2 - dthd) - F_bar*sign(sigma) + ddthd - c1*dz1 - df - h*(sigma + beta*sign(sigma));

sys(1) = ut;
sys(2) = x2;
sys(3) = z1;


