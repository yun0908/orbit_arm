function [q_final] = main()
    %% 1 建模 初始化
    clc
    clear
    %close
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
    % 末端达到目标点的初始构型
    x0 = [0.935+piping_adjust_x -0.455+piping_adjust_y-0.15 0.05 0 0 0];  % 目标点 （x,y,z,俯仰，偏航，翻滚）
    tf_g = rpy2tr(10, 0, -90);  % 末端姿态，欧拉角转姿态齐次矩阵
    Tg = transl(x0(1:3)) * tf_g ;  % 末端位姿
    % disp(Tg);
    
    q0 = snake.ikine(Tg);          % 机械臂处于最终姿态时十个关节的关节角度
    %disp(q0);
    
    %% 3 障碍物设置
    
    % other
    % piping_adjust_x = 0.15;
    
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
    
    %% 6 计算构型q0到管道的距离
    LL = -inf;
    
    % 直线管道计算
    for i = 1:size(joints,1)
        if joints(i,1)>cen0(1) && joints(i,1)<cen0(1)+L_1 
            if norm(joints(i,2:3)-cen0(2:3))> (cen0(4)-thick0 )
                %碰撞
                LL = max(LL,norm(joints(i,2:3)-cen0(2:3)));
            end
        else
            % 在管道外
        end
    end
    
    % 连接管道检测
    for i = 1:size(joints,1)
        if joints(i,1)>cen0(1)+L_1 && joints(i, 2) > cen0(2)-R % x 在直管道外
            if norm(joints(i,1:3) - [cen0(1)+L_1 cen0(2)-R cen0(3)])> 2*R-thick0
                %碰撞
                LL = max(LL,norm(joints(i,1:3)-[cen0(1)+L_1 cen0(2)-R cen0(3)])-R);
            end
        else
            % 在管道外
        end
    end
    
    % 斜管道检测
    Q1 = center1(61,:); Q2 = center1(90,:);
    for i = 1:size(joints,1)
        P = joints(i,:);
        if  joints(i,1) > cen0(1)+L_1 && joints(i, 2) < cen0(2)-R% x 在直管道外
            if norm(cross(Q2-Q1,P-Q1))/norm(Q2-Q1)> R-thick0
                %碰撞
                LL = max(LL,norm(cross(Q2-Q1,P-Q1))/norm(Q2-Q1));
            end
        else
            % 在管道外
        end
    end
    
    %%  7 冗余机械臂 自运动，抖动法，产生新构型
    
    % 参数设置
    k = 10;  % 确定抖动次数 
    delta_t = 1; 
    I = eye(snake.n);   % I = eye(n) 返回一个主对角线元素为 1 且其他位置元素为 0 的 n×n 单位矩阵。
    s = q0;             % q0: 机械臂处于最终姿态时十个关节的关节角度
    %s  = [-4.0508    1.0180   -6.9878   -9.8322    8.2991   -9.0883   -4.1462    6.7229   -7.4836    4.0358  -12.5474 1.4212  -19.3593   -1.5871  -27.7412   -5.1646];
    %v = 0.22;           % 即公式里面的*h
    shutdown = 0;
    cishu1 = 0;
    cishu2 = 1;
    qsave = zeros(20*k,snake.n) ;  
    index = [];
    
    while 1
        % 方式二
        %v = a + (b-a) * rand(1,1);
        %if abs(v) > 0.6
        %   v = 0.1;
        %end
        
        % 方式一
        %cache = rand(1,5);
        %v = cache(1);
        
        % 方式三    明天在循环里面再加上一个末端点到目标点的距离检测，如果距离大于一定值，就重新置s=q0
        %v = 0.16;  % v值越大，自运动时动的幅度越大
        v = 0.16;
        
        cishu1 = cishu1 + 1;
        if 1  % 1碰撞，0free
            for i = 1:k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 1);
                % disp(state1);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);    % s：机械臂处于最终姿态时十个关节的关节角度
                q_v = [adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];   % q_v即*h   
                detq =  (I-pinv(J)*J)*q_v;  %   B = pinv(A) 返回矩阵 A 的 Moore-Penrose 伪逆
                s = s + detq'*delta_t;      %   detq': detq的转置
                qsave(i,:) = s;
            end
            s = q0;
            for i = k+1:2*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 1);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [-adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 2*k+1:3*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 2);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                J = snake.jacob0(s);
                q_v = [0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 3*k+1:4*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 2);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 4*k+1:5*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 3);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 5*k+1:6*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 3);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 6*k+1:7*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 4);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 7*k+1:8*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 4);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 8*k+1:9*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 5);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 9*k+1:10*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 5);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 10*k+1:11*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 6);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 11*k+1:12*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 6);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 12*k+1:13*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 7);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 13*k+1:14*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 7);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 14*k+1:15*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 8);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 15*k+1:16*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 8);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 16*k+1:17*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 9);
                %disp(state1);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 17*k+1:18*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 9);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 18*k+1:19*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                % disp(state1);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 19*k+1:20*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 20*k+1:21*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 21*k+1:22*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 22*k+1:23*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 23*k+1:24*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 24*k+1:25*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 25*k+1:26*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 26*k+1:27*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 27*k+1:28*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 28*k+1:29*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 29*k+1:30*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 30*k+1:31*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
            for i = 31*k+1:32*k
                adjust = v;
                state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
                %if state1 == 0
                    %adjust = 0;
                %end
                J = snake.jacob0(s);
                q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust];
                detq =  (I-pinv(J)*J)*q_v;
                s = s + detq'*delta_t;
                qsave(i,:) = s;
            end
            s = q0;
        end
        
        
        %% 绘画
        % 直管道
        %pipe_fun2(cen0, L_1, 'show');  % 返回障碍物
        % 弯曲管道
        % 1~15的center为连接管道，16~45为斜管道
        %center1 = pipe_1_14(cen0, L_1);
        disp(size(center1));
        %center2 = pipe_2_14(cen1, L_1);
        %center1 = [center1;center2];
        
        %pipe_fun2(cen2, L_1, 'show');  % 返回障碍物
        
        axis([-0.1 2.9  -1.5 0.9 -0.6 0.7])
    
        % q01 = [  -0.5630    1.1703    7.2465    3.2162    2.1534    1.4559    7.1032    6.0847  -48.4275    4.0571  -11.1320 3.1266  -32.0299   -5.2706];
        % arm_snake 仿真运动
        W=[-5,+5,-5,+5,-5,+5];
        snake.plot(qsave(:,:),'tilesize',0.1,'workspace',W, 'nojoints', 'jointdiam',0.05);
        %snake.plot(q01(:,:), 'nojoints');
        
        %%  筛选新构型
        for i = 1:size(qsave,1)
            a_flag = 0;
            % 计算构型关节点位置
            [~, T_all] = snake.fkine(qsave(i,:));  % 返回所有关节点位置
            
            for j = 1:size(T_all,2)
                joints(j,:) = T_all(j).t';  % 存放关节点位置信息
            end
            
            % 判断构型是否碰撞
            collision = 0;
            % 判断构型是否在管道里
            state = 0;
            % 直线管道检测
            for j = 1:size(joints,1)
            if (joints(j,1)>cen0(1)-L_2) && (joints(j,1)<cen0(1)+L_1 )
                if (norm(joints(j,2:3)-cen0(2:3))) > (cen0(4)-thick0 )
                    %碰撞
                    fprintf("第%d个构型第%d个关节：直线管道，碰！\n", i, j);
                    collision = 1;
                end
            else
                state = 1; % 在管道外
            end
            end
        
            % 连接管道检测
            for j = 1:size(joints,1)
            if (joints(j,1)>cen0(1)+L_1) && (joints(j, 2) > cen0(2)-R) % x 在直管道外
                if norm(joints(j,1:3) - [cen0(1)+L_1 cen0(2)-R cen0(3)])> 2*R-thick0
                    %碰撞
                    fprintf("第%d个构型：连接管道，碰！\n", j);
                    collision = 1;
                end
            else
                state = 1;  % 在管道外
            end
            end
        
            % 斜管道检测
            Q1 = center1(61,:); Q2 = center1(90,:);
            for j = 1:size(joints,1)
            P = joints(j,:);
            if  (joints(j,1) > cen0(1)+L_1) && (joints(j, 2) < cen0(2)-R + 0.06) % x 在直管道外
                if (norm(cross(Q2-Q1,P-Q1))/norm(Q2-Q1))> R-thick0
                    %碰撞
                    fprintf("第%d个构型：斜管道，碰！\n", j);
                    collision = 1;
                end
            else
                state = 1;  % 在管道外
            end
            end
    
            % 判断是否找到无碰撞
            %if collision == 0 &&  state == 0
            
            if collision == 0
                fprintf("成功找到无碰撞构型,循环次数%d，第%d个构型\n", cishu1, i);
                snake.plot(qsave(i,:), 'nojoints');
                disp(qsave(i,:));
                a_flag = a_flag+1;
                %snake.plot(qsave(i,:));
                last = i;
                if size(index,2) == cishu1-1
                    index = [index i];
                    shutdown = 1;
                else
                    index = [index last_i i];
                if a_flag == 3
                    shutdown = 1;
                end
                end
    
                break
            end
        
            %若有碰撞，计算距离，选出距离最小的构型
            LLL = -inf;
        
            % 直线管道计算
            for j = 1:size(joints,1)
                if joints(j,1)>cen0(1) && joints(j,1)<cen0(1)+L_1 
                    if norm(joints(j,2:3)-cen0(2:3))> (cen0(4)-thick0 )
                        % 选出超出管道且离管道最远的点，计算点到管道距离
                        LLL = max(LLL,norm(joints(j,2:3)-cen0(2:3)));
                    end
                else
                    % 在管道外
                end
            end
        
            % 连接管道计算
            for j = 1:size(joints,1)
            if joints(j,1)>cen0(1)+L_1 && joints(j, 2) > cen0(2)-R % x 在直管道外
                if norm(joints(j,1:3) - [cen0(1)+L_1 cen0(2)-R cen0(3)])> 2*R-thick0
                    %碰撞
                    LLL = max(LLL,norm(joints(j,1:3)-[cen0(1)+L_1 cen0(2)-R cen0(3)])-R);
                end
            else
                % 在管道外
            end
            end
        
            % 斜管道计算
            Q1 = center1(61,:); Q2 = center1(90,:);
            for j = 1:size(joints,1)
            P = joints(j,:);
            if  joints(j,1) > cen0(1)+L_1 && joints(j, 2) < cen0(2)-R% x 在直管道外
                if norm(cross(Q2-Q1,P-Q1))/norm(Q2-Q1)> R-thick0
                    %碰撞
                    LLL = max(LLL,norm(cross(Q2-Q1,P-Q1))/norm(Q2-Q1));
                end
            else
                % 在管道外
            end
            end
        
            if LLL < LL
                q0 = qsave(i,:);
                LL = LLL;
                if cishu2 ~= cishu1
                    index = [index last_i];
                    cishu2 = cishu2 + 1;
                end
                last_i = i;
            end
    
        end
        
        disp("输出坐标点位置");
        disp(joints);
        
        if shutdown
            %disp(shutdown); 
            break;
        end
    
        fprintf("*h的值：%d\n", v);
        fprintf("次数：%d\n", cishu1);
    
        if cishu1 > 1000
            break;
        end
    end
    %snake.teach();
    if collision == 0
        q_final = qsave(last,:)*180/pi;
    end
end
