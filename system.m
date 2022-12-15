%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%s函数实现对输入的信号滤波模板

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [sys,x0,str,ts,simStateCompliance] = system(t,x,u,flag,brake)

%输入参数
%   t,x,u分别对应时间/状态/输入信号 
%   flag为标志位
%   simStateCompliance采用内建模块的方法保存和重建连续状态、工作向量等

%输出参数
%   sys为一个通用的返回参数值，其数值根据flag的不同而不同
%   x0为状态初始数值
%   一般str=[]
%   ts为一个两列的矩阵，包含采样时间和偏移量两个参数

switch flag

  case 0   % 系统进行初始化，调用mdlInitializeSizeshan函数
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
    
  case 1   % 计算连续状态变量的导数
    sys=mdlDerivatives(t,x,u);
    
  case 2   % 更新离散状态变量
    sys=mdlUpdate(t,x,u,brake);
    
  case 3   %计算S函数的输出
    sys=mdlOutputs(t,x,u);

  case 4  % 计算下一仿真时刻
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9  % 仿真结束
    sys=mdlTerminate(t,x,u);

 otherwise  %其他情况用户自定义
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
% 系统初始化子函数

sizes = simsizes;

sizes.NumContStates  = 0;   %连续状态个数
sizes.NumDiscStates  = 2;   %离散状态个数
sizes.NumOutputs     = 2;   %输出个数
sizes.NumInputs      = 2;   %输入个数
sizes.DirFeedthrough = 0;   %是否直接馈通
sizes.NumSampleTimes = 1;   %采样时间个数，至少一个

sys = simsizes(sizes);     %将size结构传到sys中
global P; %定义协方差
P=[1,0;0,1];

% initialize the initial conditions
x0  = [0;25];

% str is always an empty matrix
str = [];

% initialize the array of sample times
ts  = [0 0];
% 表示该模块采样时间继承其前的模块采样时间设置

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'DefaultSimState';

%进行连续变量的更新
function sys=mdlDerivatives(t,x,u)

sys = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%进行离散状态变量的更新
function sys=mdlUpdate(t,x,u,brake)
global P;
p_noise = 0.05 + 0.5*randn(1,1);%系统过程位置噪声 均值为0.05 标准差0.05
v_noise = 0.05 + 0.5*randn(1,1);%系统过程速度噪声 均值为0.05 标准差0.05
W = [p_noise; v_noise];%过程噪声
F=[1,0.1;0,1];  %系统状态方程的系数矩阵
B=[0.1*0.1*0.5; 0.1];
H=[1,0;0,1];  %观测方程的系数矩阵
Q=[0.25, 0;0, 0.25];  %过程噪声方差值
R=0.25;  %测量噪声方差值
    
    %xpre=F*x+B*u;                      %状态预测
    %Ppre=F*P*F'+Q;                    %协方差预测
    %Kg=Ppre*H'*inv(H*Ppre*H'+R);       %计算卡尔曼增益
    %e=u-H*xpre;               %u是输入的观测值，在此计算新息
    %xnew=xpre+Kg*e;          %状态更新
    %P=(eye(1)-Kg*H)*Ppre;    %协方差更新
    
    X_ = F*x + B*brake;%先验估计  计算公式 状态转移项 + 控制量项+误差
    P_ = F*P*F'+ Q;%先验估计 协方差矩阵
    K  = P_*H'/(H*P_*H' + R);%计算卡尔曼增益系数
    xnew = X_ + K*(u - H*X_);%最优估计  利用观察值对估计值进行修正
    P = (eye(2) - K*H)*P_;%更新最优估计的协方差矩阵
    
sys = xnew;   %将计算的结果返回主函数

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%求取系统的输出信号
function sys=mdlOutputs(t,x,u)

sys =x;
%把算得的模块输出向量赋给sys

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%计算下一仿真时刻，由sys返回
function sys=mdlGetTimeOfNextVarHit(t,x,u)
   % 此处设置下一仿真时刻为1s之后.
sys = 0.01+t;


%结束仿真子系统
function sys=mdlTerminate(t,x,u)

sys = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
