function [collision] = ceyan_collided_16(qsave, center, L_1, L_2, cen0, R, k, motor_number)
    % 标准DH参数
    du = pi/180; rad = 180/pi; 
    for i = 1:2
        L(i) = Link( 'd',0.201  ,  'a',0.35,   'alpha',pi/2);
    end
    for i = 3:4
        L(i) = Link( 'd',0.181  ,  'a',0.20,   'alpha',pi/2);
    end
    for i = 5:6
        L(i) = Link( 'd',0.181  ,  'a',0.20,   'alpha',pi/2);
    end
    L(7) = Link( 'd',0  , 'a',0.081,   'alpha',pi/2);
    for i = 8:16
        L(i) = Link( 'd',0  ,  'a',0.081,   'alpha',pi/2*(-1)^(i-1) );
    end
    snake=SerialLink(L,'name','[no]wrist' );
    
    
    thick0 = 0.034;  % 机械臂直径 
    
    
    %  筛选新构型
    LL = -inf;
        
    % 计算构型关节点位置
    [~, T_all] = snake.fkine(qsave(motor_number,:));  % 返回所有关节点位置
            
    for j = 1:size(T_all,2)
        joints(j,:) = T_all(j).t';  % 存放关节点位置信息
    end
            
    % 判断构型是否碰撞
    collision = 0;
    % 判断构型是否在管道里
    state = 0;
    % 直线管道检测
    for j = 1:size(joints,1)
        if joints(j,1)>cen0(1)-L_2 && joints(j,1)<cen0(1)+L_1 
            if norm(joints(j,2:3)-cen0(2:3))> (cen0(4)-thick0 )
                %碰撞
                collision = 1;
            end
        else
            state = 1; % 在管道外
        end
    end
            
    % 连接管道检测
    for j = 1:size(joints,1)
        if joints(j,1)>cen0(1)+L_1 && joints(j, 2) > cen0(2)-R % x 在直管道外
            if norm(joints(j,1:3) - [cen0(1)+L_1 cen0(2)-R cen0(3)])> 2*R-thick0
                %碰撞
                collision = 1;
            end
        else
            state = 1;  % 在管道外
        end
    end
            
    % 斜管道检测
    Q1 = center(61,:); Q2 = center(90,:);
    for j = 1:size(joints,1)
        P = joints(j,:);
        if  joints(j,1) > cen0(1)+L_1 && joints(j, 2) < cen0(2)-R% x 在直管道外
            if norm(cross(Q2-Q1,P-Q1))/norm(Q2-Q1)> R-thick0
                    %碰撞
                collision = 1;
            end
        else
            state = 1;  % 在管道外
        end
    end
        
    
end