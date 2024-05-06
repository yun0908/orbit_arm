
%% 1 建模 初始化
clc
clear
close
% 大加中加小 DH参数
du = pi/180; rad = 180/pi; 
L(1) = Link( 'd',0.0  ,  'a',0.25,   'alpha',pi/2, 'qlim',[-90,90]);
    L(2) = Link( 'd',0.0  ,  'a',0.25,   'alpha',-pi/2, 'qlim',[-90,90]);
    for i = 3:4
        L(i) = Link( 'd',0.152 ,  'a',0.20,   'alpha',pi/2, 'qlim',[-90,90]);
    end
    for i = 5:6
        L(i) = Link( 'd',0.152  ,  'a',0.20,   'alpha',pi/2, 'qlim',[-90,90]);
    end
    L(7) = Link( 'd',0  , 'a',0.081,   'alpha',pi/2, 'qlim',[-90,90]);
    for i = 8:16
        L(i) = Link( 'd',0  ,  'a',0.081,   'alpha',pi/2*(-1)^(i-1), 'qlim',[-90,90] );
    end

% arm_snake
snake=SerialLink(L,'name','[no]wrist' );
%snake.display();
%snake.teach();

%% 2 初始构型
piping_adjust_x = 0.95-0.15;
piping_adjust_y = 0.10-0.2;
R_adjust = 0.05;
z1 = 0.0675; % 管道高度

% x0 = [1.4694 , 0.1074, -0.0175]; % 目标点 根据最终构型的第七个关节的坐标点确定
x0 = [ 1.0060   -0.2032   -0.0453]; % 目标点 根据最终构型的第七个关节的坐标点确定
tf_g = rpy2tr(-10, 0, 30);  % 末端姿态，欧拉角转姿态齐次矩阵
Tg = transl(x0(1:3)) * tf_g ;  % 末端位姿
% disp(Tg);
    
q0 = snake.ikine(Tg);          % 机械臂处于最终姿态时十个关节的关节角度
disp(q0);

%% 3 障碍物设置
    

    % 第一个直管道
    L_1 = 0.35;  % 管道长度
    L_2 = 0.35; % 检测长度
    R = 0.20 + R_adjust;  % 半径
    z1 = 0.0675 + 0.03; % 管道高度
    cen0 = [0.36 + piping_adjust_x-0.1, -0.05+piping_adjust_y, z1, R];  % 管道入口的中心点
    pipe_fun2(cen0, L_1, 'show');  % 返回障碍物直线管道部分
    cen1 = [1.11 + piping_adjust_x + R_adjust*2-0.1, -0.14+piping_adjust_y-R_adjust, z1 , R];   % 弯管道二用
    cen2 = [1.11 + piping_adjust_x + R_adjust*2-0.1, -0.64+piping_adjust_y-R_adjust, z1 , R];   % 直管道二用
    
    % 2、弯曲管道
    % 1~15的center为连接管道，16~45为斜管道
    center1 = pipe_1_14(cen0, L_1);
    center2 = pipe_2_14(cen1, L_1);
    
    %center1 = [center1;center2];
    
    % 3、直管道3
    pipe_fun2(cen2, L_1 + 10, 'show');  % 返回障碍物直线管道部分    


% 拿到各个关节点坐标
thick0 = 0.034;  % 机械臂直径
% 计算起始构型关节点位置
[~, T_all] = snake.fkine(q0);  % 返回所有关节点位置
joints = zeros(snake.n,3);
for j = 1:size(T_all,2)
    joints(j,:) = T_all(j).t';  % 存放关节点位置信息
end
axis([-0.1 2.9  -1.5 0.9 -0.6 0.7])


%%  7 冗余机械臂 自运动，抖动法，产生新构型
% qsave = ziyundon(snake, q0);
% disp(qsave);
% snake.plot(qsave, 'nojoints');
% 
% axis([-0.1 2.9  -1.5 0.9 -1.0 0.7]);

        
%% 仿真
%q0 = [ 2.4353    8.7404   -3.1716  -20.9515  -27.0824  -10.7072  -49.9820    6.9482   -8.1201    4.9625  -13.1722  2.6932  -19.0586    0.1605  -25.2095   -2.4647];
% arm_snake 仿真运动
snake.plot(q0(:,:), 'nojoints');