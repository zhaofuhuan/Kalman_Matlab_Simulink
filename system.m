%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%s����ʵ�ֶ�������ź��˲�ģ��

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [sys,x0,str,ts,simStateCompliance] = system(t,x,u,flag,brake)

%�������
%   t,x,u�ֱ��Ӧʱ��/״̬/�����ź� 
%   flagΪ��־λ
%   simStateCompliance�����ڽ�ģ��ķ���������ؽ�����״̬������������

%�������
%   sysΪһ��ͨ�õķ��ز���ֵ������ֵ����flag�Ĳ�ͬ����ͬ
%   x0Ϊ״̬��ʼ��ֵ
%   һ��str=[]
%   tsΪһ�����еľ��󣬰�������ʱ���ƫ������������

switch flag

  case 0   % ϵͳ���г�ʼ��������mdlInitializeSizeshan����
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
    
  case 1   % ��������״̬�����ĵ���
    sys=mdlDerivatives(t,x,u);
    
  case 2   % ������ɢ״̬����
    sys=mdlUpdate(t,x,u,brake);
    
  case 3   %����S���������
    sys=mdlOutputs(t,x,u);

  case 4  % ������һ����ʱ��
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9  % �������
    sys=mdlTerminate(t,x,u);

 otherwise  %��������û��Զ���
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
% ϵͳ��ʼ���Ӻ���

sizes = simsizes;

sizes.NumContStates  = 0;   %����״̬����
sizes.NumDiscStates  = 2;   %��ɢ״̬����
sizes.NumOutputs     = 2;   %�������
sizes.NumInputs      = 2;   %�������
sizes.DirFeedthrough = 0;   %�Ƿ�ֱ����ͨ
sizes.NumSampleTimes = 1;   %����ʱ�����������һ��

sys = simsizes(sizes);     %��size�ṹ����sys��
global P; %����Э����
P=[1,0;0,1];

% initialize the initial conditions
x0  = [0;25];

% str is always an empty matrix
str = [];

% initialize the array of sample times
ts  = [0 0];
% ��ʾ��ģ�����ʱ��̳���ǰ��ģ�����ʱ������

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'DefaultSimState';

%�������������ĸ���
function sys=mdlDerivatives(t,x,u)

sys = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%������ɢ״̬�����ĸ���
function sys=mdlUpdate(t,x,u,brake)
global P;
p_noise = 0.05 + 0.5*randn(1,1);%ϵͳ����λ������ ��ֵΪ0.05 ��׼��0.05
v_noise = 0.05 + 0.5*randn(1,1);%ϵͳ�����ٶ����� ��ֵΪ0.05 ��׼��0.05
W = [p_noise; v_noise];%��������
F=[1,0.1;0,1];  %ϵͳ״̬���̵�ϵ������
B=[0.1*0.1*0.5; 0.1];
H=[1,0;0,1];  %�۲ⷽ�̵�ϵ������
Q=[0.25, 0;0, 0.25];  %������������ֵ
R=0.25;  %������������ֵ
    
    %xpre=F*x+B*u;                      %״̬Ԥ��
    %Ppre=F*P*F'+Q;                    %Э����Ԥ��
    %Kg=Ppre*H'*inv(H*Ppre*H'+R);       %���㿨��������
    %e=u-H*xpre;               %u������Ĺ۲�ֵ���ڴ˼�����Ϣ
    %xnew=xpre+Kg*e;          %״̬����
    %P=(eye(1)-Kg*H)*Ppre;    %Э�������
    
    X_ = F*x + B*brake;%�������  ���㹫ʽ ״̬ת���� + ��������+���
    P_ = F*P*F'+ Q;%������� Э�������
    K  = P_*H'/(H*P_*H' + R);%���㿨��������ϵ��
    xnew = X_ + K*(u - H*X_);%���Ź���  ���ù۲�ֵ�Թ���ֵ��������
    P = (eye(2) - K*H)*P_;%�������Ź��Ƶ�Э�������
    
sys = xnew;   %������Ľ������������

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%��ȡϵͳ������ź�
function sys=mdlOutputs(t,x,u)

sys =x;
%����õ�ģ�������������sys

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%������һ����ʱ�̣���sys����
function sys=mdlGetTimeOfNextVarHit(t,x,u)
   % �˴�������һ����ʱ��Ϊ1s֮��.
sys = 0.01+t;


%����������ϵͳ
function sys=mdlTerminate(t,x,u)

sys = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
