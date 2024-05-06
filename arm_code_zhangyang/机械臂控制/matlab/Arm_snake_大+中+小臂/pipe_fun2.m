function [] = pipe_fun2(cen, h, varargin)
    %%  设计圆柱管道
%     h=0.5;  % 圆柱高度
    R = cen(4);  % 半径
    a = cen(1);  % 原点x坐标
    b = cen(2);  % 原点y坐标
    c = cen(3);  % 原点z坐标
    

    % 临时测试
%     R = 0.23;  % 半径
%     a = 0.5;  % 原点x坐标
%     b = 0;  % 原点y坐标
%     c = 0.4;  % 原点z坐标
%     h = 0.5;
    radius = R;
    m=100;  % 分割线的条数
    
    [x,y,z]=cylinder1(R,m);%创建以(0,0)为圆心，高度为[0,1]，半径为R的圆柱 cylinder
    
    x = x * h + a;      %平移x轴
    y = y + b;        %平移y轴，改为(a,b)为底圆的圆心
    z = z + c;        %高度放大h倍
    
%     figure(1)
%     mesh(x,y,z)%重新绘图
%     hold on;
    
    %% 取出圆柱的圆
    % 时间跨度
    t = (0:0.5:16)'; 
    n1 = 7;  % 割出的圆的数目  
    center = zeros(n1, 3);
    for i = 1: n1
        center(i,:) = [a+(i-1)*h/n1 b c];
    end

    % 将圆画出来
    theta = t*(2*pi/t(end));
    points = zeros(3,33,n1);
    for i = 1:size(points,3)
        points(:,:,i) =(center(i,:) + radius*[zeros(size(theta)) cos(theta) sin(theta)])';
    end
    
%     for i = 1:size(points,3)
%         plot3(points(1,:,i),points(2,:,i),points(3,:,i),'r');
%         hold on;
%     end
    
    
    %% 可视化
    if size(varargin) ~= 0
        if varargin{1} == 'show'
            % 画筒
            axis([-0.7 0.7  -0.7 0.7 -0.7 0.7]);
            % mesh(x,y,z,'edgecolor','g')%重新绘图
            plot3(x,y,z,'y');
            hold on;
            
%             % 画圆
%             for i = 1:size(points,3)
%                 plot3(points(1,:,i),points(2,:,i),points(3,:,i),'r');
%                 hold on;
%             end
% 
%             % 画框大小
%             axis([-0.7 1.2  -0.7 0.7 -0.7 1]);
        end
    end

end

