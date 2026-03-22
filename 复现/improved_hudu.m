%% 构建栅格地图
clear;clc
load ditu.mat% G 地形图为01矩阵，如果为1，表示障碍物
G=ditu;
h=rot90(abs(peaks(100)));
global MM
global Dir 
global Lgrid 
%Lgrid = input('请输入栅格粒径：');
Lgrid = 1;

MM=size(G,1); %MM为矩阵维数
figure(1);   
for i=1:MM
  for j=1:MM
  x1=(j-1)*Lgrid;y1=(MM-i)*Lgrid; 
  x2=j*Lgrid;y2=(MM-i)*Lgrid; 
  x3=j*Lgrid;y3=(MM-i+1)*Lgrid; 
  x4=(j-1)*Lgrid;y4=(MM-i+1)*Lgrid; 
  f=(max(max(h))-h(i,j))/max(max(h));
    if G(i,j)==1 
        fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.2,0.2,0.2]); hold on %栅格为1，填充为黑色
    else 
        fill([x1,x2,x3,x4],[y1,y2,y3,y4],[f,1,f]); hold on %栅格为0，填充为白色
    end 
  end 
end
axis([0,MM*Lgrid,0,MM*Lgrid]) 
grid on
%% 初始化地图信息
%{
Xinitial = input('请输入初始点的X坐标：');
Yinitial = input('请输入初始点的Y坐标：');
%}
Xinitial = 0;
Yinitial = 99;
[initial,ij_initial]= modify(Xinitial,Yinitial);
if max(ij_initial)>MM||G(ij_initial(1),ij_initial(2))==1
    error('初始点不能设在障碍物上或超出范围');
end
%{
Xdestination = input('请输入目标点的X坐标：');
Ydestination = input('请输入目标点的Y坐标：');
    %}
Xdestination = 99;
Ydestination = 0;
[destination,ij_destination]= modify(Xdestination,Ydestination);
if max(ij_destination)>MM||G(ij_destination(1),ij_destination(2))==1
    error('目标点不能设在障碍物上或超出范围');
end
%% 计算距离启发矩阵dis
dis = zeros(MM,MM);
for i=1:MM
  for j=1:MM
   x = (j-0.5)*Lgrid;
   y = (MM-i+0.5)*Lgrid;
   dis(i,j) = sqrt(sum(([x y]-destination).^2));
  end
end
%% 计算距离转移矩阵D
D=zeros(MM^2,8);   %行号表示栅格标号，列号表示邻接的8个方向的栅格号
Dir = [-MM-1,-1,MM-1,MM,MM+1,1,1-MM,-MM];
 for i = 1:MM^2     %8方向转移距离矩阵初步构建
     Dirn = Dir+i;
     if G(i)==1
             D(i,:)=inf;
             continue
     end
         for j = 1:8
             if  Dirn(j)<=0||Dirn(j)>MM^2        %出界的情况，暂且为0
                 continue 
             end
             if G(Dirn(j))==1
                 D(i,j) = inf;
             elseif mod(j,2)==0         %偶数方向为上下左右方向
                 D(i,j) = 1;
             elseif j==1 %左上方向的情况，保证路线不会擦障碍物边沿走过
                 if (G(Dirn(2))+G(Dirn(8))==0)
                   D(i,j) = 1.4; 
                 else
                   D(i,j) = inf;   
                 end
             elseif (Dirn(j-1)<=0||Dirn(j-1)>MM^2)||(Dirn(j+1)<=0||Dirn(j+1)>MM^2)%排除掉垂直方向的栅格出界的情况
                 continue
             elseif G(Dirn(j-1))+G(Dirn(j+1))==0    %其余三个斜方向
                 D(i,j) = 1.4;
             else
                 D(i,j) = inf;
             end
         end
     
 end
%% 创造边界
 num = 1:MM^2;
 obs_up = find(mod(num,MM)==1);
 obs_up = obs_up(2:end-1);
 D(obs_up,[1,2,3])=inf;
 obs_down = find(mod(num,MM)==0);
 obs_down = obs_down(2:end-1);
 D(obs_down,[5,6,7])=inf;
 D(2:MM-1,[1,7,8]) = inf;
 D(MM^2-MM+2:MM^2-1,[3,4,5])=inf;
 D(1,[1,2,3,7,8])=inf;
 D(MM,[1,5,6,7,8])=inf;
 D(MM^2-MM+1,[1,2,3,4,5])=inf;
 D(MM^2,[3,4,5,6,7])=inf;
%% 参数初始化
tic
NC_max=50; m=50;  Rho=0.3; Q=100; Omega=10; Mu=1;  u=10; Tau_min=10; Tau_max=40; Rho_min=0.2;
%% 绘制找到的最优路径
[R_best,F_best,L_best,T_best,S_best,S_ave,Shortest_Route,Shortest_Length]=improved(D,initial,destination,dis,h,NC_max,m,Rho,Omega,Mu,Q,u,Tau_min,Tau_max,Rho_min); %函数调用
%绘制找到的最优路径
j = ceil(Shortest_Route/MM);
i = mod(Shortest_Route,MM);
i(i==0) = MM;
x = (j-0.5)*Lgrid;
y = (MM-i+0.5)*Lgrid;
x = [initial(1) x destination(1)];
y = [initial(2) y destination(2)];
figure(1);
plot(x,y,'-r');
xlabel('x'); ylabel('y'); title('最佳路径');
%grid on 
hold on
toc  %计算运行时间

% 使用样条插值平滑路径
t = linspace(0, 1, numel(x)); % 在0到1之间生成足够数量的点，用于样条插值
tx = csapi(t, x); % x坐标的样条插值
ty = csapi(t, y); % y坐标的样条插值

% 在绘图时使用插值结果
tFine = linspace(0, 1, 10*numel(x)); % 使用更密集的点来绘制平滑路径
xFine = fnval(tx, tFine); % 获取更平滑的x坐标
yFine = fnval(ty, tFine); % 获取更平滑的y坐标

% 绘制平滑路径
plot(xFine, yFine, 'r', 'LineWidth', 2); % 使用蓝色线条绘制粗路径，线宽为2
hold on; % 保持图形窗口，使得后续绘制的路径不会清除当前图形


% 标注起点和终点
plot(initial(1), initial(2), 'o', 'MarkerSize', 5, 'MarkerEdgeColor', 'r', 'MarkerEdgeColor','r', 'LineWidth', 1.5); % 起点处用蓝色空心小圆圈表示
plot(destination(1), destination(2), 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'LineWidth', 1.5); % 终点处用红色实心小圆圈表示



%% 绘制收敛曲线
figure(2); iter=1:length(L_best);
plot(iter,L_best,'-r','LineWidth',1)
xlabel('迭代次数'); ylabel('各代最佳路线的长度');
axis([0,NC_max,25,90]);
%grid on;
hold on
figure(4); iter=1:length(L_best);
plot(iter,T_best,'-r','LineWidth',1)
xlabel('迭代次数'); ylabel('各代最佳路线的转弯次数'); 
axis([0,NC_max,5,50]);
%grid on;
hold on

% 假设路径点的横坐标存储在数组 x 中，纵坐标存储在数组 y 中
% 这里假设 x 和 y 是已知的路径坐标数组

% 计算路径关于参数 t 的一阶导数
dx = gradient(x); % x 的一阶导数
dy = gradient(y); % y 的一阶导数

% 计算路径关于参数 t 的二阶导数
d2x = gradient(dx); % x 的二阶导数
d2y = gradient(dy); % y 的二阶导数

% 计算曲率
curvature = abs(dx .* d2y - dy .* d2x) ./ (dx.^2 + dy.^2).^(3/2);

% 显示曲率
figure;
plot(curvature, 'r', 'LineWidth', 1); % 用红色线条绘制曲率曲线，线宽为 2
xlabel('路径点');
ylabel('曲率');
title('路径曲率');
grid on;


% 假设路径点的横坐标存储在数组 x 中，纵坐标存储在数组 y 中
% 这里假设 x 和 y 是已知的路径坐标数组

total_length = 0; % 初始化路径总长度

% 遍历路径上的所有点，计算每个线段的长度并累加
for i = 1:length(x)-1
    segment_length = sqrt((x(i+1) - x(i))^2 + (y(i+1) - y(i))^2); % 计算当前线段的长度
    total_length = total_length + segment_length; % 累加到总长度中
end

disp(['最后一次迭代的路径长度为：', num2str(total_length)]);

% 假设路径点的横坐标存储在数组 x 中，纵坐标存储在数组 y 中
% 这里假设 x 和 y 是已知的路径坐标数组

turn_threshold = 30; % 转弯阈值，单位为度
turn_count = 0; % 初始化转弯次数

% 计算路径上的转弯次数
for i = 2:length(x)-1
    % 计算当前点与前一个点的方向
    direction_prev = atan2(y(i)-y(i-1), x(i)-x(i-1)) * 180 / pi;
    % 计算当前点与后一个点的方向
    direction_next = atan2(y(i+1)-y(i), x(i+1)-x(i)) * 180 / pi;
    % 计算当前方向与前一个方向的夹角
    angle_diff = abs(direction_next - direction_prev);
    % 判断是否发生了转弯
    if angle_diff > turn_threshold
        turn_count = turn_count + 1; % 转弯次数加1
    end
end

disp(['最后一次迭代的转弯次数为：', num2str(turn_count)]);












