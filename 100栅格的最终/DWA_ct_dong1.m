
function a = DWA_ct_dong1(Obs_Closed,Obs_dong,Obs_dong1,Obs_dong2,Obs_d_j,Area_MAX,Goal,Line_path,path_node,Start0,angle_node,Kinematic,evalParam)
figure
num_obc=size(Obs_Closed,1); %  计算障碍物的数量
num_path=size(path_node,1);
xTarget=path_node(num_path,1);
yTarget=path_node(num_path,2);
%  path=[];
%  for i=1:1:(num_path-1)
%         pa=Line_pa(path_node(i,1),path_node(i,2),path_node(i+1,1),path_node(i+1,2));
%         path=[path;pa];
%  end

xm=path_node(1,1);
ym=path_node(1,2);
% 初始位置坐标
%angle_S=pi;
%=sn_angle(path_node(1,1),path_node(1,2),path_node(2,1),path_node(2,2));

%zhuangjiao_node=angle_S-angle_node;
x=[xm ym angle_node 0 0]';% 机器人的初期状态 x=[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
x_du=x;

%G_goal=path_node(num_path,:);% 目标点位置 [x(m),y(m)]



obstacleR=3.0;% 冲突判定用的障碍物半径
global dt;  %   全局变量
dt=0.5;%   时间[s]%增加无人艇速度

% 机器人运动学模型
% [   最高速度[m/s], 最高旋转速度[rad/s], 加速度[m/ss], 旋转加速度[rad/ss], 速度分辨率[m/s], 转速分辨率[rad/s]  ]
% Kinematic=[2.0, toRadian(40.0), 0.4, toRadian(100.0), 0.01, toRadian(1)]; Kinematic=[1, toRadian(20.0), 0.28, toRadian(55), 0.01, toRadian(1)];
%Kinematic=[1, toRadian(20.0), 0.2, toRadian(50), 0.01, toRadian(1)];
%Kinematic=[2.0, toRadian(40.0), 0.4, toRadian(100.0), 0.01, toRadian(1)];
% [1.0, toRadian(20.0), 0.2, toRadian(50.0), 0.01, toRadian(1)];
% 评价函数参数 [heading,dist,velocity,predictDT][方位角偏差系数， 障碍物距离， 当前速度大小, 预测是时间 ]
%evalParam=[0.3,  0.5,  0.2,  3.0];%0.3 0.1 0.1   [0.05,  0.2,  0.1,  3.0]
MAX_X=Area_MAX(1,1);
MAX_Y=Area_MAX(1,2);
% 模拟区域范围 [xmin xmax ymin ymax]
% 模拟实验的结果
dong_guiji=[ ];
dong_guiji1=[ ];
dong_guiji2=[ ];
result.x=[];
goal=path_node(2,:);
ob_dong_num=size(Obs_dong,1);
ob_dong_num1=size(Obs_dong1,1);
ob_dong_num2=size(Obs_dong2,1);
tic;
% movcount=0;
% Main loop
for i=1:5000
    % DWA参数输入
    %goal=path_node(num_path,:);
    %%
    obstacle1=[];
    obstacle2=[];
    obstacle3=[];
    obstacle4=[];
    if i<=ob_dong_num
        obstacle1=[Obs_Closed;Obs_d_j;Obs_dong(i,1) Obs_dong(i,2)];
        ob_i=i;
        dong_guiji=[dong_guiji;Obs_dong(i,1) Obs_dong(i,2)];
    else
        obstacle2=[Obs_Closed;Obs_d_j;Obs_dong(ob_dong_num,1) Obs_dong(ob_dong_num,2)];
        ob_i=ob_dong_num;
    end
    
    %% 循环2
    if i<=ob_dong_num1
        obstacle3=[Obs_Closed;Obs_d_j;Obs_dong1(i,1) Obs_dong1(i,2);obstacle1];
        ob_i1=i;
        dong_guiji1=[dong_guiji1;Obs_dong1(i,1) Obs_dong1(i,2)];
    else
        obstacle4=[Obs_Closed;Obs_d_j;Obs_dong1(ob_dong_num1,1) Obs_dong1(ob_dong_num1,2);obstacle2];
        ob_i1=ob_dong_num1;
    end
    
    %% 循环3
    if i<=ob_dong_num2
        obstacle=[Obs_Closed;Obs_d_j;Obs_dong2(i,1) Obs_dong2(i,2);obstacle3];
        ob_i2=i;
        dong_guiji2=[dong_guiji2;Obs_dong2(i,1) Obs_dong2(i,2)];
    else
        obstacle=[Obs_Closed;Obs_d_j;Obs_dong2(ob_dong_num2,1) Obs_dong2(ob_dong_num2,2); obstacle4];
        ob_i2=ob_dong_num2;
    end
    
    
    
    %    num_result=size(result.x,2);
    dang_node=[x(1,1) x(2,1)];
    dis_ng=distance(dang_node(1,1),dang_node(1,2),xTarget,yTarget);
    dis_x_du=distance(x_du(1,1),x_du(2,1),goal(1,1),goal(1,2));
    if num_path==2||dis_ng<2
        Ggoal=[xTarget yTarget];
    else
        Ggoal=Target_node(dang_node,path_node,Obs_dong,xTarget,yTarget,goal,dis_x_du);
    end
    goal=Ggoal;
    % obstacle=OBSTACLE(Obs_Closed,Obs_dong,path_node);
    
    
    
    [u,traj]=DynamicWindowApproach_ct(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    % u = [ 速度 转速 ] traj=[ 3s内的所有状态轨迹 ]
    x=f(x,u);% 机器人移动到下一个时刻
    
    x_du=f_du(x,u);% 机器人移动到下一个时刻
    result.x=[result.x; x'];
    % 模拟结果的保存
    % 是否到达目的地
    %if norm(x(1:2)-G_goal')<0.2
    if dis_ng<0.2
        disp('Arrive Goal!!');break;
    end
    
    %====Animation====
    hold off;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  画图
    for i_obs=1:1:num_obc
        x_obs=Obs_Closed(i_obs,1);
        y_obs=Obs_Closed(i_obs,2);
        fill([x_obs,x_obs+1,x_obs+1,x_obs],[y_obs,y_obs,y_obs+1,y_obs+1],'k');hold on;
    end
    plot( Line_path(:,1)+.5, Line_path(:,2)+.5,'b:','linewidth',1);
    plot(Start0(1,1)+.5,Start0(1,2)+.5,'b^');
    % text(Start0(1,1)+1,Start0(1,2)+1.5,'S','fontsize',18')
    plot(Goal(1,1)+.5,Goal(1,2)+.5,'bo');
    
    %% 第一个障碍物
    xval=25;%floor(xval);起点
    yval=77;%floor(yval);
    Obst_xS=xval;%X Coordinate of the Target
    Obst_yS=yval;%Y Coordinate of the Target
    xval=1;%floor(xval);终点
    yval=100;%floor(yval);
    Obst_xT=xval;%Starting Position
    Obst_yT=yval;%Starting Position
    
    
    dy0 = (yval-Obst_yS);
    dx0 = (xval-Obst_xS);
    if(xval>=Obst_xS&&yval>Obst_yS)
        angle10 =  atand(dy0/dx0);
    elseif(xval<Obst_xS&&yval>=Obst_yS)
        angle10 = 180 + atand(dy0/dx0);
    elseif(xval<=Obst_xS&&yval<Obst_yS)
        angle10 =  atand(dy0/dx0)+180;
    elseif(xval>Obst_xS&&yval<=Obst_yS)
        angle10 =  360+atand(dy0/dx0);
    else
        angle10 =  0;
    end
    angle10=-((360-angle10)/180)*pi;
    angle_node=angle10;
    dxx=[5/8,3/4,5/8,-0.25,-0.25,5/8]';dyy=[1/14,0,-1/14,-1/14,1/14,1/14]';
    x10 = Obs_dong(ob_i,1)+cos(angle10) .* dxx- sin(angle10) .* dyy;
    y10 = Obs_dong(ob_i,2)+cos(angle10) .* dyy+ sin(angle10) .* dxx;
    fill(x10+0.5,y10+0.5,'y');
    
    %% 第二个障碍物
    xval1=62;%floor(xval1);起点
    yval1=64;%floor(yval1);
    Obst_xS1=xval1;%X Coordinate of the Target
    Obst_yS1=yval1;%Y Coordinate of the Target
    xval1=95;%floor(xval1);终点
    yval1=1;%floor(yval1);
    dy1 = (yval1-Obst_yS1);
    dx1 = (xval1-Obst_xS1);
    if(xval1>=Obst_xS1&&yval1>Obst_yS1)
        angle12 =  atand(dy1/dx1);
    elseif(xval1<Obst_xS1&&yval1>=Obst_yS1)
        angle12 = 180 + atand(dy1/dx1);
    elseif(xval1<=Obst_xS1&&yval1<Obst_yS1)
        angle12 =  atand(dy1/dx1)+180;
    elseif(xval1>Obst_xS1&&yval1<=Obst_yS1)
        angle12 =  360+atand(dy1/dx1);
    else
        angle12 =  0;
    end
    angle12=-((360-angle12)/180)*pi;
    angle_node=angle12;
    dxx=[5/8,3/4,5/8,-0.25,-0.25,5/8]';dyy=[1/14,0,-1/14,-1/14,1/14,1/14]';
    x12 = Obs_dong1(ob_i1,1)+cos(angle12) .* dxx- sin(angle12) .* dyy;
    y12 = Obs_dong1(ob_i1,2)+cos(angle12) .* dyy+ sin(angle12) .* dxx;
    fill(x12+0.5,y12+0.5,'y');
    
    
    %% 第三个障碍物
    xval2=30;%floor(xval2);起点
    yval2=21;%floor(yval2);
    Obst_xS2=xval2;%X Coordinate of the Target
    Obst_yS2=yval2;%Y Coordinate of the Target
    xval2=96;%floor(xval2);终点
    yval2=21;%floor(yval2);
    Obst_xT2=xval2;%Starting Position
    Obst_yT2=yval2;%Starting Position
    dy3 = (yval2-Obst_yS2);
    dx3 = (xval2-Obst_xS2);
    if(xval2>=Obst_xS2&&yval2>Obst_yS2)
        angle13 =  atand(dy3/dx3);
    elseif(xval2<Obst_xS2&&yval2>=Obst_yS2)
        angle13 = 180 + atand(dy3/dx3);
    elseif(xval2<=Obst_xS2&&yval2<Obst_yS2)
        angle13 =  atand(dy3/dx3)+180;
    elseif(xval2>Obst_xS2&&yval2<=Obst_yS2)
        angle13 =  360+atand(dy3/dx3);
    else
        angle13 =  0;
    end
    angle13=-((360-angle13)/180)*pi;
    angle_node=angle13;
    dxx=[5/8,3/4,5/8,-0.25,-0.25,5/8]';dyy=[1/14,0,-1/14,-1/14,1/14,1/14]';
    x13 = Obs_dong2(ob_i2,1)+cos(angle13) .* dxx- sin(angle13) .* dyy;
    y13 = Obs_dong2(ob_i2,2)+cos(angle13) .* dyy+ sin(angle13) .* dxx;
    fill(x13+0.5,y13+0.5,'y');
    plot( dong_guiji(:,1)+.5, dong_guiji(:,2)+.5,'k:','linewidth',1);
    plot( dong_guiji1(:,1)+.5, dong_guiji1(:,2)+.5,'k:','linewidth',1);
    plot( dong_guiji2(:,1)+.5, dong_guiji2(:,2)+.5,'k:','linewidth',1);
    hold on
    
    dong_num=size(Obs_d_j,1);
    for i_d=1:1:dong_num
        x_do=Obs_d_j(i_d,1);
        y_do=Obs_d_j(i_d,2);
        fill([x_do,x_do+1,x_do+1,x_do],[y_do,y_do,y_do+1,y_do+1],[0 0 0]);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    % 本船画图
    dxx=[5/8,3/4,5/8,-0.25,-0.25,5/8]';dyy=[1/14,0,-1/14,-1/14,1/14,1/14]';
    %     dxx=dxx.*0.5;dyy=dyy.*0.5;
    x11 = x(1)+cos(x(3)) .* dxx- sin(x(3)) .* dyy;
    y11 = x(2)+cos(x(3)) .* dyy+ sin(x(3)) .* dxx;
    fill(x11+0.5,y11+0.5,'r');
    %     quiver(x(1)+0.5,   x(2)+0.5,  ArrowLength*cos(x(3)),  ArrowLength*sin(x(3)),'ok');
    hold on;
    %  x=[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
    plot(result.x(:,1)+0.5, result.x(:,2)+0.5,'-b');hold on;
    plot(goal(1)+0.5,goal(2)+0.5,'*r');hold on;
    
    %     画探索轨迹！！！
    %      % traj = [ （第一组5行 v w ）3s内的所有状态轨迹 31个点 1-5行31列；
    %                 （第二组5行 v w ）3s内的所有状态轨迹 31个点 6-10行31列；
    %               。。。。。。]
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5 %
            ind=1+(it-1)*5;
            plot(traj(ind,:)+0.5,traj(ind+1,:)+0.5,'-g','linewidth',0.5);hold on;%%模拟轨迹
        end
    end
    % % %         axis(area);
    grid off; 
    drawnow;

    
    

    %movcount=movcount+1;
    %mov(movcount) = getframe(gcf);%
  

     
end
a=result.x;
%% 画局部动态图



toc
%movie2avi(mov,'movie.avi');
