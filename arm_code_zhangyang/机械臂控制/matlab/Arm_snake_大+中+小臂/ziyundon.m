function [qsave] = ziyundon(snake, q0)
    %% 2 初始构型
    piping_adjust_x = 0.95;
    piping_adjust_y = 0.10;
    R_adjust = 0.05;
    % 末端达到目标点的初始构型
    x0 = [0.935+piping_adjust_x -0.505+piping_adjust_y 0.05 0 0 0];  % 目标点 （x,y,z,俯仰，偏航，翻滚）

    %disp(q0);
    
    %% 3 障碍物设置
    
    % other
    % piping_adjust_x = 0.15;
    
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
    
    %% 生成新构型
    % 方式三    明天在循环里面再加上一个末端点到目标点的距离检测，如果距离大于一定值，就重新置s=q0
    v = 0.16;
    snake_n = 16;
    % 参数设置
    k = 10;  % 确定抖动次数 
    delta_t = 1; 
    I = eye(snake_n);   % I = eye(n) 返回一个主对角线元素为 1 且其他位置元素为 0 的 n×n 单位矩阵。
    s = q0;             % q0: 机械臂处于最终姿态时十个关节的关节角度
    %v = 0.22;           % 即公式里面的*h
    shutdown = 0;
    cishu1 = 0;
    cishu2 = 1;
    
    qsave = zeros(20*k,snake_n) ; 

    if 1  % 1碰撞，0free
        for i = 1:k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 1);
            % disp(state1);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);    % s：机械臂处于最终姿态时十个关节的关节角度
            q_v = [0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0];   % q_v即*h   
            detq =  (I-pinv(J)*J)*q_v;  %   B = pinv(A) 返回矩阵 A 的 Moore-Penrose 伪逆
            s = s + detq'*delta_t;      %   detq': detq的转置
            qsave(i,:) = s;
        end
        s = q0;
        for i = k+1:2*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 1);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 2*k+1:3*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 2);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 3*k+1:4*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 2);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 4*k+1:5*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 3);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 5*k+1:6*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 3);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 6*k+1:7*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 4);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 7*k+1:8*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 4);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 8*k+1:9*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 5);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 9*k+1:10*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 5);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 10*k+1:11*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 6);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 11*k+1:12*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 6);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 12*k+1:13*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 7);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 13*k+1:14*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 7);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 14*k+1:15*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 8);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 15*k+1:16*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 8);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 16*k+1:17*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 9);
            %disp(state1);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 17*k+1:18*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 9);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 18*k+1:19*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            % disp(state1);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; adjust];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 19*k+1:20*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; -adjust];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 20*k+1:21*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 21*k+1:22*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [-adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 22*k+1:23*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 23*k+1:24*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 24*k+1:25*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 25*k+1:26*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 26*k+1:27*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 27*k+1:28*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 28*k+1:29*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 29*k+1:30*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 30*k+1:31*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
        for i = 31*k+1:32*k
            adjust = v;
            state1 = ceyan_collided_16(qsave, center1, L_1, L_2, cen0, R, k, 10);
            if state1 == 0
                adjust = 0;
            end
            J = snake.jacob0(s);
            q_v = [0; 0; 0; 0; 0; -adjust; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
            detq =  (I-pinv(J)*J)*q_v;
            s = s + detq'*delta_t;
            qsave(i,:) = s;
        end
        s = q0;
    end
end