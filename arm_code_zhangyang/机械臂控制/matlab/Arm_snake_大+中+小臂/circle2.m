function [center] = circle2(c,r,n)
%     n=[-1 1 0]; %法向量n
%     r=0.085; %圆的半径为1
%     c=[cen0(1)+L_1+r*cos(pi/4) -(r-r*cos(pi/4)) z1]; %圆心的坐标
    
    theta=(0:2*pi/100:2*pi)'; %theta角从0到2*pi
    a=cross(n,[1 0 0]); %n与i叉乘，求取a向量
    if ~any(a) %如果a为零向量，将n与j叉乘
        a=cross(n,[0 1 0]);
    end
    b=cross(n,a); %求取b向量
    a=a/norm(a); %单位化a向量
    b=b/norm(b); %单位化b向量
    
    c1=c(1)*ones(size(theta,1),1);
    c2=c(2)*ones(size(theta,1),1);
    c3=c(3)*ones(size(theta,1),1);
    
    x=c1+r*a(1)*cos(theta)+r*b(1)*sin(theta);%圆上各点的x坐标
    y=c2+r*a(2)*cos(theta)+r*b(2)*sin(theta);%圆上各点的y坐标
    z=c3+r*a(3)*cos(theta)+r*b(3)*sin(theta);%圆上各点的z坐标
    
    % plot3(x,y,z,'m')
    % plot3(x,y,z,'Color',[240, 248, 255]);
    plot3(x,y,z,'y');
    xlabel('x轴')
    ylabel('y轴')
    zlabel('z轴')

    center = [x y z];
end

