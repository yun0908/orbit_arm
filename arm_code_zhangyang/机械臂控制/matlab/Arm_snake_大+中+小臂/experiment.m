function [] = experiment(q_end)
    clc
    clear
    %close
    %% 1 建模 初始化
    % 标准DH参数
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
    
    snake=SerialLink(L,'name','snake');
    
    %%  构型设置
    % U型构型：-1.5708   0.01745    0.5236   0.01745   -1.0471   0.01745   1.0471    0.01745   0.01745    0.01745    0.01745   0.01745    0.01745   0.01745   0.01745    0.01745
    %  初始构型
    q_mid = [-1.5708   0.01745    0.5236   0.01745   -1.0471   0.01745   1.0471    0.01745   0.01745    0.01745    0.01745   0.01745    0.01745   0.01745   0.01745    0.01745];
    %  进入管道构型
    q_ready = [-1.4199    0.8615    0.3670   -0.8464   -1.5372    0.4921   -0.2759    0.1313   -0.0409    0.3222    0.1308  0.3204    0.1875    0.1593    0.0817   -0.1201];
    % 目标构型
    q_end1 = [ -2.2811    2.1617  -12.2691   -9.4150   -5.2161   -7.7449   -7.6321    5.4983   -9.8441    3.3911  -13.9977 1.5677  -20.3302   -0.3739  -28.9110   -2.7876];
    q_end = q_end1*du;
    % snake.plot(ready,'nojoints')
    % snake.todegrees(ready)
    [~,T_all] = snake.fkine(q_end);
    T_enter = T_all(6);  % 进入管道的构型
    
    %%  轨迹规划

    % 阶段一，从初始构型到达管道口
    t = (0: 0.13: 1); solb = zeros(size(t,2)+1, snake.n);
    sola = jtraj(q_mid, q_ready, t);
    solq = sola;
    %snake.plot(solq, 'nojoints')
    
    % 阶段二，到达管道口
    T0 = snake.fkine(solq(end,:));  % T0 输出机器人末端的齐次变换矩阵（齐次变换矩阵由各关节角度值决定）
    solb(1,:) = solq(end,:);
    Ts = ctraj(T0, T_enter, t);
    for i = 1:size(Ts, 2)
        sol = snake.ikcon(Ts(i), solq(end,:));
        solb(i+1,:) = sol;
    end
    solq = [solq; solb];
    % snake.plot(solq, 'nojoints');
    
    % 阶段三，末端两个关节爬行到轨迹7、8节处
    sola = jtraj(solq(end,15:16), q_end(7:8), t);
    snake1 = SerialLink(L(1:14),'name','snake1');
    T0 = snake1.fkine(solq(end,1:14));  % 
    Ts = ctraj(T0, T_enter, t);
    solb = zeros(size(t,2), snake1.n);
    for i = 1:size(Ts, 2)
        sol = snake1.ikcon(Ts(i), solq(end,1:14));
        solb(i,:) = sol;
    end
    sol_new = [solb, sola];
    solq = [solq; sol_new];
    % snake.plot(solq, 'nojoints')
    
    % 
    sola = jtraj(solq(end,13:16), q_end(7:10), t);
    snake1 = SerialLink(L(1:12),'name','snake2');
    T0 = snake1.fkine(solq(end,1:12));  %  逆解，让现在的前十二个关节角度向最终构型的前十二个关节角度靠近
    Ts = ctraj(T0, T_enter, t);
    solb = zeros(size(t,2), snake1.n);
    for i = 1:size(Ts, 2)
        sol = snake1.ikcon(Ts(i), solq(end,1:12));  
        solb(i,:) = sol;
    end
    sol_new = [solb, sola];
    solq = [solq; sol_new];
    % snake.plot(solq, 'nojoints')
    

    
    % 阶段三，末端两个关节爬行到轨迹7、8节处
    sola = jtraj(solq(end,11:16), q_end(7:12), t);
    snake1 = SerialLink(L(1:10),'name','snake3');
    T0 = snake1.fkine(solq(end,1:10));  % 
    Ts = ctraj(T0, T_enter, t);
    solb = zeros(size(t,2), snake1.n);
    for i = 1:size(Ts, 2)
        sol = snake1.ikcon(Ts(i), solq(end,1:10));
        solb(i,:) = sol;
    end
    sol_new = [solb, sola];
    solq = [solq; sol_new];
    % snake.plot(solq, 'nojoints')
    
    % 阶段三，末端两个关节爬行到轨迹7、8节处
    sola = jtraj(solq(end,9:16), q_end(7:14), t);
    snake1 = SerialLink(L(1:8),'name','snake4');
    T0 = snake1.fkine(solq(end,1:8));  % 
    Ts = ctraj(T0, T_enter, t);
    solb = zeros(size(t,2), snake1.n);
    for i = 1:size(Ts, 2)
        sol = snake1.ikcon(Ts(i), solq(end,1:8));
        solb(i,:) = sol;
    end
    sol_new = [solb, sola];
    solq = [solq; sol_new];
    % snake.plot(solq, 'nojoints')
    
    % 阶段三，末端两个关节爬行到轨迹7、8节处
    sola = jtraj(solq(end,7:16), q_end(7:16), t);
    snake1 = SerialLink(L(1:6),'name','snake5');
    T0 = snake1.fkine(solq(end,1:6));  % 
    Ts = ctraj(T0, T_enter, t);
    solb = zeros(size(t,2), snake1.n);
    for i = 1:size(Ts, 2)
        sol = snake1.ikcon(Ts(i), solq(end,1:6));
        solb(i,:) = sol;
    end
    sol_new = [solb, sola];
    solq = [solq; sol_new];
    %snake.plot(solq, 'nojoints')
    



%     % 阶段四，末端三四关节循环
%     sola = jtraj(solq(end,7:10), q_end(7:10), t);
%     snake2 = SerialLink(L(1:6),'name','snake2');
%     T0 = snake2.fkine(solq(end,1:6));  % 
%     Ts = ctraj(T0, T_enter, t);
%     solb = zeros(size(t,2), snake2.n);
%     for i = 1:size(Ts, 2)
%         sol = snake2.ikcon(Ts(i), solq(end,1:6));
%         solb(i,:) = sol;
%     end
%     sol_new = [solb, sola];
%     solq = [solq; sol_new];
%     
    %% 绘画
    piping_adjust_x = 0.95-0.15;
    piping_adjust_y = 0.10-0.2;
    R_adjust = 0.05;
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


    axis([-0.1 2.9  -1.5 0.9 -1.0 0.7]);
    % solq = q_end1;
    snake.plot(solq, 'nojoints');
    % snake.plot(q_end, 'nojoints');
    
    [~, theory] = snake.fkine(q_end);
    [~, fact] = snake.fkine(solq(end,:));
end