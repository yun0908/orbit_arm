function [center1] = pipe_2_14(cen0,L_1)
    %% 管道连接处（斜管道）
    r1 = 0.25;  % 平直段的半径为
    r2 = 0.25;  % 连接管道的半径
    z1 = cen0(3); 
    
    center1 = zeros(90, 3);
    x = linspace(-180, -90, 60);
    % x = linspace(-90, 0, 60);
    for i = 1:size(x,2)
        n = [-1 tand(x(i)) 0];
        c2=[cen0(1)+r2*cosd(90-x(i)) cen0(2)-(r2-r2*sind(90-x(i))) z1]; %圆心的坐标
        axis([-0.6 0.6  -0.7 0.7 -0.7 0.7]);
        circle2(c2, r2, n);
        center1(i,:) = c2; 
    end
    
    %% 第二个直管道
    
%     % 位置
%     n=[0 1 0]; %法向量n
%     c1=[cen0(1)+L_1+r1  cen0(2)-r2  z1]; %圆心的坐标
%     center1(61,:) = c1;
%     circle2(c1, r1, n);
%     
%     y = linspace(cen0(2)-r2, -r2-0.08, 30);
%     
%     for i = 1:29
%         circle2([c1(1) y(i) z1],r1,n);
%         center1(i+61,:) =  [c1(1) y(i) z1];
%     end
    
end

