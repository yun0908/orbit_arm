clear,clc,%close all;
%% 建立机器人DH参数，初始状态为竖直状态
 
L1=Link('d',144,'a',0,  'alpha',pi/2, 'standard');
L2=Link('d',0,  'a',264,'alpha',0,    'offset',pi/2,'standard');
L3=Link('d',0,  'a',236,'alpha',0,    'standard');
L4=Link('d',106,'a',0,  'alpha',-pi/2,'offset',-pi/2,'standard');
L5=Link('d',114,'a',0,  'alpha',pi/2, 'standard');
L6=Link('d',67, 'a',0,  'alpha',0,    'standard');
 
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','Arm6')
Theta=[0 0 0 0 0 0];
Theta=Theta/180*pi; %换算成弧度
forwarda=robot.fkine(Theta) %求正解的齐次变换矩阵
W=[-1000,+1000,-1000,+1000,0,+1000];
robot.plot(Theta,'tilesize',150,'workspace',W, 'jointdiam',2); %显示三维动画
% robot.teach(forwarda,'rpy' ) %显示roll/pitch/yaw angles，GUI可调界面