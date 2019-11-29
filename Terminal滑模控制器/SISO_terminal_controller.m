function [sys,x0,str,ts] = SISO_terminal_controller(t, x, u, flag)
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
sizes.NumInputs      = 4;   %输入变量个数
sizes.DirFeedthrough = 1;   %输入信号是否在输出子函数中出现
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

f = -25*u(2);
b = 133;
D = 3;
F = 0;
K = 10;
C = [4.9, 1];

th = u(2);
dth = u(3);
ddth = u(4);
e = th - thd;
de = dth - dthd;
dde = ddth - ddthd;

T = 1;
if(t > T)
    p = 0;
    dp = 0;
    ddp = 0;
else
    A1 = 10/(T^3)*e + 6/(T^2)*de + 3/(2*T)*dde;
    B1 = 15/(T^4)*e + 8/(T^3)*de + 3/(2*(T^2))*dde;
    C1 = 6/(T^5)*e + 3/(T^4)*de + 1/(2*(T^3))*dde;
    p = e + de*t + 1/2*dde*t^2 - A1*t^3 + B1*t^4 - C1*t^5;
    dp = de + dde*t - 3*A1*t^2 + 4*B1*t^3 - 5*C1*t^4;
    ddp = dde - 6*A1*t + 12*B1*t^2 - 20*C1*t^3;
end

E = [e; de];
P = [p; dp];
s = C*E - C*P;
ut = -1/b*(f - ddthd - ddp + 1/(C(2))*C(1)*(de - dp)) - 1/b*sign(C(2)*s)*(F+D+K);

sys(1) = ut;
sys(2) = e;
sys(3) = de;




