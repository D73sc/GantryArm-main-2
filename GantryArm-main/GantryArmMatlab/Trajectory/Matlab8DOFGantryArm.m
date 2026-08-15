clc
clear

%蒙特卡洛法仿真机械臂可达工作空间
radian1 = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%定义DH参数%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

d0=0;
d1=0;
d2=0;
d3 = 160+135.5;
d4 = 0;
d5 = 381;
d6 = 0;
d7 = 229;

%机械臂连杆长度参数a
a0 = 0;
a1 = 0;
a2 = 0;
a3 = 90;
a4 = 200;
a5 = 0;
a6 = 0;
a7 = 0;

%机械臂关节偏角参数alpha
alpha0 =  90* radian1;
alpha1 =  90* radian1;
alpha2 =  90* radian1;
alpha3 =   0 * radian1;
alpha4 =  90* radian1;
alpha5 =  90 * radian1;
alpha6 =  90 * radian1;
alpha7 =  90 * radian1;

%theta
theta0 =  90* radian1;
theta1 =  -90* radian1;
theta2 =  90* radian1;

%定义关节角度限制
lim0_min = 0 ; lim0_max = 250 ; %关节1(-170，170)
lim1_min = 0 ; lim1_max = 300 ; %关节1(-170，170)
lim2_min = 0 ; lim2_max = 200 ; %关节1(-170，170)
lim3_min = -180 * radian1; lim3_max = 180 * radian1; %关节1(-170，170)
lim4_min = -75 * radian1; lim4_max =   75 * radian1; %关节2(-132，0)
lim5_min = -180 * radian1; lim5_max = 180 * radian1; %关节3(1，141)
lim6_min = -120 * radian1; lim6_max = 120 * radian1; %关节5(-105，105)
lim7_min = -180 * radian1; lim7_max = 180 * radian1; %关节5(-105，105)

%定义关节旋转范围
lim0 = lim0_max - lim0_min;
lim1 = lim1_max - lim1_min;
lim2 = lim2_max - lim2_min;
lim3 = lim3_max - lim3_min;
lim4 = lim4_max - lim4_min;
lim5 = lim5_max - lim5_min;
lim6 = lim6_max - lim6_min;
lim7 = lim7_max - lim7_min;


% 定义各个连杆以及关节类型，默认为转动关节
%           θ  d    a    α        角度限制    关节偏置
L(1)=Link([ theta0  , d0  , a0 ,  alpha0 ,1],'modified'); L(1).qlim=[lim0_min,lim0_max];
L(2)=Link([ theta1   d1   a1   alpha1   1],'modified'); L(2).qlim=[lim1_min,lim1_max];
L(3)=Link([ theta2   d2   a2   alpha2   1],'modified'); L(3).qlim=[lim2_min,lim2_max];
L(4)=Link([ 0   d3   a3   alpha3],'modified'); L(4).qlim=[lim3_min,lim3_max];
L(5)=Link([ 0   d4   a4   alpha4],'modified'); L(5).qlim=[lim4_min,lim4_max];L(5).offset=pi/2;   
L(6)=Link([ 0   d5   a5   alpha5],'modified'); L(6).qlim=[lim5_min,lim5_max];L(6).offset=pi; 
L(7)=Link([ 0   d6   a6   alpha6],'modified'); L(7).qlim=[lim6_min,lim6_max];L(7).offset=pi;
L(8)=Link([ 0   d7   a7   alpha7],'modified'); L(8).qlim=[lim7_min,lim7_max];L(8).offset=pi;

%组合上述连杆
AR3=SerialLink(L,'name','8DOF');
%显示机械臂关节数以及D_H参数列表
%AR3:: 6 axis, RRRRRR, stdDH, slowRNE
AR3.display();
 
%使用蒙特卡洛法绘制机械臂的工作空间
N=10;
d0 = ( lim0_min + (lim0 * rand(N,1)) );
d1 = ( lim1_min + (lim1 * rand(N,1)) );
d2 = ( lim2_min + (lim2 * rand(N,1)) );
theta3 = ( lim3_min + (lim3 * rand(N,1)) );
theta4 = ( lim4_min + (lim4 * rand(N,1)) );
theta5 = ( lim5_min + (lim5 * rand(N,1)) );
theta6 = ( lim6_min + (lim6 * rand(N,1)) );
theta7 = ( lim7_min + (lim7 * rand(N,1)) );


data = table(); % 初始化空表

% 指定Excel文件名
filename = 'D:\Git\5DOFGantryArm\5DOFGantryArmMatlab\data_output.xlsx';

%filename = 'data_output.xlsx';
for n = 1:N
    theta = [d0(n),d1(n),d2(n),theta3(n),theta4(n),theta5(n),theta6(n),theta7(n)];
    workspace = AR3.fkine(theta);
    plot3(workspace.t(1),workspace.t(2),workspace.t(3),'b.','markersize',1);
    % 将数据放入表格中
    data(n,:) = table(n,d0(n),d1(n),d2(n),theta3(n),theta4(n),theta5(n),theta6(n),theta7(n),workspace.t(1),workspace.t(2),workspace.t(3), 'VariableNames', {'Index','d0','d1','d2','theta3','theta4','theta5','theta6','theta7', 'X', 'Y', 'Z'});

    hold on;
end

% 将数据写入Excel文件
writetable(data, filename);
disp(['数据已导入到 ', filename]);



AR3.plot(theta);  %动画显示
%进行机器人的实时展示
AR3.teach('rpy/zyx');