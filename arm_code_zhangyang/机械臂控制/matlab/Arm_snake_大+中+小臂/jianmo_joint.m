clc
clear
close
% 中加小 DH参数
du = pi/180; rad = 180/pi; 
for i = 1:2
   L(i) = Link( 'd',0.0  ,  'a',0,   'alpha',pi/2*(-1)^(i-1));
end
%     for i = 3:4
%         L(i) = Link( 'd',0.152 ,  'a',0.20,   'alpha',pi/2);
%     end
for i = 3:4
    L(i) = Link( 'd',0.181  ,  'a',0.081,   'alpha',pi/2);
end
for i = 5:6
    L(i) = Link( 'd',0.181  ,  'a',0.081,   'alpha',pi/2);
end
L(7) = Link( 'd',0  , 'a',0.081,   'alpha',pi/2);
% for i = 8:16
%    L(i) = Link( 'd',0  ,  'a',0.081,   'alpha',pi/2*(-1)^(i-1) );
% end



    
robot=SerialLink(L,'name','Arm6' );
Theta=[0 0 0 0 0 0 0];
Theta=Theta/180*pi; %换算成弧度
forwarda=robot.fkine(Theta); %求正解的齐次变换矩阵
x0 = [0.935 + 1.2 -0.395 0.05 0 0 0];  % 目标点 （x,y,z,俯仰，偏航，翻滚）
tf_g = rpy2tr(10, 0, -90);  % 末端姿态，欧拉角转姿态齐次矩阵
Tg = transl(x0(1:3)) * tf_g ;  % 末端位姿

q0 = robot.ikine(Tg);          % 机械臂处于最终姿态时十个关节的关节角度


%% 3 障碍物设置
    
% other
% piping_adjust_x = 0.15;
piping_adjust_x =  0.85;
piping_adjust_y = 0.10;
R_adjust = 0.05;
z1 = 0.0675; % 管道高度

x0 = [0.26 + piping_adjust_x, -0.05+piping_adjust_y, z1]; % 目标点 （x,y,z,俯仰，偏航，翻滚）
tf_g = rpy2tr(10, 50, 0);  % 末端姿态，欧拉角转姿态齐次矩阵
Tg = transl(x0(1:3)) * tf_g ;  % 末端位姿

% 第一个直管道
L_1 = 0.35;  % 管道长度
L_2 = 0.35; % 检测长度
R = 0.20 + R_adjust;  % 半径
z1 = 0.0675; % 管道高度
cen0 = [0.36 + piping_adjust_x, -0.05+piping_adjust_y, z1, R];  % 管道入口的中心点
pipe_fun2(cen0, L_1, 'show');  % 返回障碍物直线管道部分
cen1 = [1.11 + piping_adjust_x + R_adjust*2, -0.27+piping_adjust_y-R_adjust*2, z1 , R];   % 弯管道二用
cen2 = [1.11 + piping_adjust_x + R_adjust*2, -0.67+piping_adjust_y-R_adjust*4, z1 , R];   % 直管道二用

% 2、弯曲管道
% 1~15的center为连接管道，16~45为斜管道
center1 = pipe_1_14(cen0, L_1);
center2 = pipe_2_14(cen1, L_1);
% 3、直管道3
pipe_fun2(cen2, L_1 + 10, 'show');  % 返回障碍物直线管道部分


% W=[-1000,+1000,-1000,+1000,0,+1000];
% robot.plot(Theta,'tilesize',1,'workspace'); %显示三维动画
robot.teach(); %显示roll/pitch/yaw angles，GUI可调界面