clc
clear
close all
load path.mat

%% 初始参数
dt = 0.1;   % 时间步长
L = 2.9;    % 轴距
max_steer =60 * pi/180; % in rad
target_v =30.0 / 3.6;

%% 参考轨迹的相关参数
% 定义参考轨迹
refPos = path;
refPos_x = refPos(:,1);
refPos_y = refPos(:,2);

% 计算航向角和曲率
diff_x = diff(refPos_x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y);
diff_y(end+1) = diff_y(end);
refHeading = atan2(diff_y , diff_x);                   % 航向角
derivative1 = gradient(refPos_y) ./ abs(diff_x);       % 一阶导数
derivative2 = del2(refPos_y) ./ abs(diff_x);           % 二阶导数
refK = abs(derivative2) ./ (1+derivative1.^2).^(3/2);  % 计算曲率

% 根据阿克曼转向原理，计算参考前轮转角
refDelta = atan(L*refK);

% 绘图
figure
plot(refPos_x,refPos_y,'r-')
hold on

%% 主程序
x = refPos_x(1)+0.5; 
y = refPos_y(1) + 0.5; 
yaw = refHeading(1)+0.02; 
v = 0.1;
U = [0.01;0.01];
idx =0;
pos_actual = [refPos_x,refPos_y];
latError_MPC = [];

sizeOfPath = length(refPos_x);
while idx<sizeOfPath-1
    % 调用MPC控制器
    [Delta,v,idx,latError,U] = mpc_control(x,y,yaw,refPos_x,refPos_y,refHeading,refDelta,dt,L,U,target_v) ;
    
    % 误差太大，退出程序
    if abs(latError) > 3
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 更新状态量
    [x,y,yaw] = updateState(x,y,yaw,v , Delta, dt,L, max_steer); 
    
    % 保存每一步的实际量
    pos_actual(end+1,:) = [x,y];
    latError_MPC(end+1,:) = [idx,latError];
    
    % 画跟踪轨迹图
    scatter(x,y,150,'b.')
    pause(0.01);
end

%% 保存
path_MPC = pos_actual;
save path_MPC.mat path_MPC
save latError_MPC.mat latError_MPC
