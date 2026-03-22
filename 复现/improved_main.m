%% 构建栅格地图
clear;clc;close all;
global angle10 angle12 angle13 x
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
grid off
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
NC_max=30; m=50;  Rho=0.3; Q=100; Omega=10; Mu=1;  u=10; Tau_min=10; Tau_max=40; Rho_min=0.2;
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
% %% 绘制收敛曲线
% figure(2); iter=1:length(L_best);
% plot(iter,L_best,'-r','LineWidth',1)
% xlabel('迭代次数'); ylabel('各代最佳路线的长度');
% axis([0,NC_max,25,90]);
% %grid on;
% hold on
% figure(3); iter=1:length(L_best);
% plot(iter,F_best*100,'-r','LineWidth',1)
% xlabel('迭代次数'); ylabel('各代最佳路线的高度均方差*100');
% axis([0,NC_max,0,30]);
% %grid on;
% hold on
% figure(4); iter=1:length(L_best);
% plot(iter,T_best,'-r','LineWidth',1)
% xlabel('迭代次数'); ylabel('各代最佳路线的转弯次数');
% axis([0,NC_max,5,50]);
% %grid on;
% hold on
% figure(5); iter=1:length(L_best);
% plot(iter,S_best,'r',iter,S_ave,'b');
% xlabel('迭代次数');ylabel('各代最佳路线的综合指标及平均综合指标');
% title('收敛性分析曲线')
% axis([0,NC_max,70,200]);
% %grid on;
% hold on




figure(2);
%% 画图
rgb=imread("黑白海图.jpg");
I=rgb2gray(rgb);
gmag = imgradient(I);
se = strel('disk',13);
Io = imopen(I,se);
Ie = imerode(I,se);
Iobr = imreconstruct(Ie,I);
Ioc = imclose(Io,se);
Iobrd = imdilate(Iobr,se);
Iobrcbr = imreconstruct(imcomplement(Iobrd),imcomplement(Iobr));
Iobrcbr = imcomplement(Iobrcbr);
a=100;
b=100;
l=1;    %网格边长
B = imresize(Iobrcbr,[a/l b/l]); %   将数字矩阵转为规定的大小
graph=double(B);
xuyao=find(B>0);
B(xuyao)=1;
%% 算法
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  1  设置 地图、起始点、目标点
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
B(find(B==0))=2;
B(find(B==1))=0;
B(find(B==2))=1;

%%%只能设置正方形矩阵，行和列相等，否则旋转时会出现错误
%%%只能设置正方形矩阵，行和列相等，否则旋转时会出现错误
%  0 表示无障碍物  1表示有障碍物
MAX0 = B;
MAX=rot90(MAX0,3);      %%%设置0,1摆放的图像与存入的数组不一样，需要先逆时针旋转90*3=270度给数组，最后输出来的图像就是自己编排的图像
MAX_X=size(MAX,2);                                %%%  获取列数，即x轴长度
MAX_Y=size(MAX,1);                                %%%  获取行数，即y轴长度
MAX_VAL=10;                              %%%   返回由数字组成的字符表达式的数字值，就是函数用于将数值字符串转换为数值

axis([1 MAX_X+1, 1 MAX_Y+1])                %%%  设置x，y轴上下限

xlabel('经度/(°)')
ylabel('纬度/(°)')
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',...
    'xGrid','on','yGrid','on')
aaa=linspace(121.7260,122.0380,4);
xticks([1.0000   34.3333   67.6667  101.0000])
xticklabels({'121.7260'  '121.8300'  '121.9340'  '122.0380'});
set(gca,'XTickLabelRotation',360);
bbb=linspace(29.9130,30.1410,4);
yticks([1.0000   34.3333   67.6667  101.0000])
yticklabels({'29.9130'   '29.9890'   '30.0650'   '30.1410'});
grid off;                                   %%%  在画图的时候添加网格线
hold on;                                   %%%  当前轴及图像保持而不被刷新，准备接受此后将绘制的图形，多图共存
n=0;%Number of Obstacles                   %%%  障碍的数量
k=1;          %%%% 将所有障碍物放在关闭列表中；障碍点的值为1;并且显示障碍点
CLOSED=[0 0];
for jj=1:MAX_X
    for ii=1:MAX_Y
        if (MAX(ii,jj)==1)
            %%plot(i+.5,j+.5,'ks','MarkerFaceColor','b'); 原来是红点圆表示
            fill([ii,ii+1,ii+1,ii],[jj,jj,jj+1,jj+1],'k');  %%%改成 用黑方块来表示障碍物
            CLOSED(k,1)=ii;  %%% 将障碍点保存到CLOSE数组中
            CLOSED(k,2)=jj;
            k=k+1;
        else
            fill([ii,ii+1,ii+1,ii],[jj,jj,jj+1,jj+1],[1 1 1]);
        end
    end
end
Area_MAX(1,1)=MAX_X;
Area_MAX(1,2)=MAX_Y;
Obs_Closed=CLOSED;
Num_obs=size(CLOSED,1); %%%存储障碍物的数量  *********************************************************************
xval=100;%floor(xval);                                              %%%  floor（）取不大于传入值的最大整数，向下取整
yval=1;%floor(yval);
xTarget=xval;%X Coordinate of the Target                       %%%   目标的坐标
yTarget=yval;%Y Coordinate of the Target
Target(1,1)=xTarget;
Target(1,2)=yTarget;
MAP(xval,yval) = -1 ;                      %%%   目标坐标点位置的值设为-1                                 %%%   目标点颜色b 蓝色 g 绿色 k 黑色 w白色 r 红色 y黄色 m紫红色 c蓝绿色
xStart=1;%Starting Position
yStart=100;%Starting Position
Start(1,1)=xStart;
Start(1,2)=yStart;
MAP(xval,yval)=2;                                                 %%%   起始点位置的值设置为1；目标点为0，障碍点为-1，其余空白点为2
hold on;
% 绘图
ROUT=Shortest_Route;
LENROUT=length(ROUT);
Rx=x;
Ry=y;
for in=1:LENROUT
    Plan_path(in,1)=Rx(in)+0.5;
    Plan_path(in,2)=Ry(in)+0.5;
end
disp('全局静态规划时间')

%单画蚁群
plot(xTarget+0.5,yTarget+0.5,'bo');
plot(xStart+0.5,yStart+0.5,'b^');
plot(Plan_path(:,1)+.5,Plan_path(:,2)+.5,'m','linewidth',2); %5%绘线 'b',
S=Distance_path(Plan_path);
disp('全局规划的路径长度')
S
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              4      路径平滑度优化
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ip=1;
for i=LENROUT:-1:2
    
    Optimal_pathq(ip,1)=Plan_path(i,1);
    Optimal_pathq(ip,2)=Plan_path(i,2);
    Optimal_pathq(ip,3)=Plan_path(i-1,1);
    Optimal_pathq(ip,4)=Plan_path(i-1,2);
    ip=ip+1;
end
Num_Opt=size(Optimal_pathq,1);
%%%  优化折线
Optimal_path_one=Line_OPEN_ST(Optimal_pathq,Obs_Closed,Num_obs,Num_Opt);
%%%%%%%%%%%%%%%   把路径提取出来  %%%%%%%%%%%%%%%%%%%%%
Optimal_path_try=[1 1 1 1];
Optimal_path=[1 1 ];node_l=[1 1];
i=1;q=1;
x_g=Optimal_path_one(Num_Opt,3);
y_g=Optimal_path_one(Num_Opt,4);
Optimal_path_try(i,1)= Optimal_path_one(q,1);
Optimal_path_try(i,2)= Optimal_path_one(q,2);
Optimal_path_try(i,3)= Optimal_path_one(q,3);
Optimal_path_try(i,4)= Optimal_path_one(q,4);

while (Optimal_path_try(i,3)~=x_g || Optimal_path_try(i,4)~=y_g)
    i=i+1;
    q=Optimal_index(Optimal_path_one,Optimal_path_one(q,3),Optimal_path_one(q,4));
    
    Optimal_path_try(i,1)= Optimal_path_one(q,1);
    Optimal_path_try(i,2)= Optimal_path_one(q,2);
    Optimal_path_try(i,3)= Optimal_path_one(q,3);
    Optimal_path_try(i,4)= Optimal_path_one(q,4);
    
end
%%%%%%%%%%%%%%%     反过来排列路线节点      %%%%%%%%%%%%%%%%%%%%%

n=size(Optimal_path_try,1);
for i=1:1:n
    Optimal_path(i,1)=Optimal_path_try(n,3);
    Optimal_path(i,2)=Optimal_path_try(n,4);
    %        Optimal_path(i,3)=Optimal_path_try(n,1);
    %        Optimal_path(i,4)=Optimal_path_try(n,2);
    n=n-1;
end
num_op=size(Optimal_path,1)+1;
Optimal_path(num_op,1)=Optimal_path_try(1,1);
Optimal_path(num_op,2)=Optimal_path_try(1,2);
%  plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5,'linewidth',1); %5%绘线
% 二次折线优化

Optimal_path_two=Line_OPEN_STtwo(Optimal_path,CLOSED,Num_obs,num_op);
num_optwo=size(Optimal_path_two,1)+1;
Optimal_path_two(num_optwo,1)=xStart;
Optimal_path_two(num_optwo,2)=yStart;
%  % plot(Optimal_path_two(:,1)+.5,Optimal_path_two(:,2)+.5,'linewidth',1); %5%绘线
% 三次折线
j=num_optwo;
Optimal_path_two2=[xStart yStart];
for i=1:1:num_optwo
    Optimal_path_two2(i,1)=Optimal_path_two(j,1);
    Optimal_path_two2(i,2)=Optimal_path_two(j,2);
    j=j-1;
end
Optimal_path_three=Line_OPEN_STtwo(Optimal_path_two2,CLOSED,Num_obs,num_optwo);
num_opthree=size(Optimal_path_three,1)+1;
Optimal_path_three(num_opthree,1)=xTarget;
Optimal_path_three(num_opthree,2)=yTarget;
Plan_path2=Optimal_path_three;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%自己调整全局航线
% Plan_path2(3,:)=[42,22];
% Plan_path2(4,:)=[80,21];
% hou=Plan_path2(3:end,:);
% qian=Plan_path2(1,:);
% hangdao=[25,76;28,35];
% 
% Plan_path2=[1,100;hangdao;42,22;80,21;100,1];

figure(3)
axis([1 MAX_X+1, 1 MAX_Y+1])                %%%  设置x，y轴上下限
grid off; 
hold on;
num_obc=size(Obs_Closed,1);
for i_obs=1:1:num_obc
    x_obs=Obs_Closed(i_obs,1);
    y_obs=Obs_Closed(i_obs,2);
    fill([x_obs,x_obs+1,x_obs+1,x_obs],[y_obs,y_obs,y_obs+1,y_obs+1],'k');hold on;
end
plot( Plan_path(:,1)+.5, Plan_path(:,2)+.5,'m:','linewidth',2);
plot(xStart+.5,yStart+.5,'b^');
text(xStart+1,yStart+1.5,'S','fontsize',18')
plot(xTarget+.5,yTarget+.5,'co');
text(xTarget+1,yTarget+1.5,'T','fontsize',18')
plot( Plan_path2(:,1)+.5, Plan_path2(:,2)+.5,'b','linewidth',2);
%
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',...
    'xGrid','on','yGrid','on')
aaa=linspace(121.7260,122.0380,4);
xticks([1.0000   34.3333   67.6667  101.0000])
xticklabels({'121.7260'  '121.8300'  '121.9340'  '122.0380'});
set(gca,'XTickLabelRotation',360);
bbb=linspace(29.9130,30.1410,4);
yticks([1.0000   34.3333   67.6667  101.0000])
yticklabels({'29.9130'   '29.9890'   '30.0650'   '30.1410'});
xlabel('经度/(°)')
ylabel('纬度/(°)')




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Line_path=Plan_path2;


%%                                     局部避障
Start=[xStart yStart];
Goal=[xTarget yTarget];
figure(4)
axis([1 MAX_X+1, 1 MAX_Y+1])                %%%  设置x，y轴上下限
% set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',...
%     'xGrid','on','yGrid','on')
% aaa=linspace(121.7260,122.0380,4);
% xticklabels({[aaa(1,1)],[],[],[],[],[],[],[],[],[],[],[aaa(1,2)],[],[],[],[],[],[],[],[],[],[],[],[aaa(1,3)],[],[],[],[],[],[],[],[],[],[],[],[aaa(1,4)]});
% set(gca,'XTickLabelRotation',360);
% bbb=linspace(29.9130,30.1410,4);
% yticklabels({[bbb(1,1)],[],[],[],[],[],[],[],[],[],[],[bbb(1,2)],[],[],[],[],[],[],[],[],[],[],[],[bbb(1,3)],[],[],[],[],[],[],[],[],[],[],[],[bbb(1,4)]});
xlabel('经度/(°)')
ylabel('纬度/(°)')
grid off; 
hold on;
num_obc=size(Obs_Closed,1);
for i_obs=1:1:num_obc
    x_obs=Obs_Closed(i_obs,1);
    y_obs=Obs_Closed(i_obs,2);
    fill([x_obs,x_obs+1,x_obs+1,x_obs],[y_obs,y_obs,y_obs+1,y_obs+1],'k');hold on;
end
plot( Plan_path2(:,1)+.5, Plan_path2(:,2)+.5,'b:','linewidth',2);
plot(xStart+.5,yStart+.5,'b^');
plot(Goal(1,1)+.5,Goal(1,2)+.5,'bo');

%% 第一个障碍物
xval=25;%floor(xval);起点
yval=77;%floor(yval);
Obst_xS=xval;%X Coordinate of the Target
Obst_yS=yval;%Y Coordinate of the Target
plot(Obst_xS+0.5,Obst_yS+0.5,'k^');
xval=1;%floor(xval);终点
yval=100;%floor(yval);
Obst_xT=xval;%Starting Position
Obst_yT=yval;%Starting Position
plot(Obst_xT+0.5,Obst_yT+0.5,'ko');
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
Obst_d_d_St=[Obst_xS Obst_yS];
Obst_d_d_Ta=[Obst_xT Obst_yT];
[Obst_d_path,Obst_d_distance_x,Obst_d_OPEN_num]=Astar_G(Obs_Closed,Obst_d_d_St,Obst_d_d_Ta,MAX_X,MAX_Y);
Obst_d_path_X=[Obst_d_path;Obst_d_d_Ta];
L_obst=0.1;%0.016;% 设置移动障碍物的速度 0.1s内运动 L_obst m  速度为10*L_obst m/s
Obst_d_d_line=Line_obst(Obst_d_path_X,L_obst);
plot( Obst_d_d_line(:,1)+.5, Obst_d_d_line(:,2)+.5,'r','linewidth',1);

%% 第二个
xval1=25;%floor(xval1);起点
yval1=76;%floor(yval1);
Obst_xS1=xval1;%X Coordinate of the Target
Obst_yS1=yval1;%Y Coordinate of the Target
plot(xval1+.5,yval1+.5,'k^');
xval1=27;%floor(xval1);终点
yval1=33;%floor(yval1);
plot(xval1+.5,yval1+.5,'ko');
%计算航行角度

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
Obst_xT1=xval1;%Starting Position
Obst_yT1=yval1;%Starting Position
Obst_d_d_St1=[Obst_xS1 Obst_yS1];
Obst_d_d_Ta1=[Obst_xT1 Obst_yT1];
[Obst_d_path1,Obst_d_distance_x,Obst_d_OPEN_num]=Astar_G(Obs_Closed,Obst_d_d_St1,Obst_d_d_Ta1,MAX_X,MAX_Y);
Obst_d_path_X1=[Obst_d_path1;Obst_d_d_Ta1];
L_obst=0.005;%0.016;% 设置移动障碍物的速度 0.1s内运动 L_obst m  速度为10*L_obst m/s
Obst_d_d_line1=Line_obst(Obst_d_path_X1,L_obst);
plot( Obst_d_d_line1(:,1)+.5, Obst_d_d_line1(:,2)+.5,'r','linewidth',1);
hold on
%% 第三个
xval2=80;%floor(xval2);起点
yval2=23;%floor(yval2);
Obst_xS2=xval2;%X Coordinate of the Target
Obst_yS2=yval2;%Y Coordinate of the Target
plot(Obst_xS2+.5,Obst_yS2+.5,'k^');
xval2=36;%floor(xval2);终点
yval2=16;%floor(yval2);
Obst_xT2=xval2;%Starting Position
Obst_yT2=yval2;%Starting Position
plot(Obst_xT2+.5,Obst_yT2+.5,'ko');
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
Obst_d_d_St2=[Obst_xS2 Obst_yS2];
Obst_d_d_Ta2=[Obst_xT2 Obst_yT2];
[Obst_d_path2,Obst_d_distance_x,Obst_d_OPEN_num]=Astar_G(Obs_Closed,Obst_d_d_St2,Obst_d_d_Ta2,MAX_X,MAX_Y);
Obst_d_path_X2=[Obst_d_path2;Obst_d_d_Ta2];
L_obst2=0.04;%0.016;% 设置移动障碍物的速度 0.1s内运动 L_obst m  速度为10*L_obst m/s
Obst_d_d_line2=Line_obst(Obst_d_path_X2,L_obst2);
plot( Obst_d_d_line2(:,1)+.5, Obst_d_d_line2(:,2)+.5,'r','linewidth',1);

dg=0;%Dummy counter
Obs_d_j=[0 0];
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAX(i,j) == 8)
            dg=dg+1;
            Obs_d_j(dg,1)=i;
            Obs_d_j(dg,2)=j;
        end
    end
end

%% 机器人运动学模型
% 机器人初始方向角度 s_du
%计算初始角度
x1=xStart;
y1=yStart;
x2=Plan_path2(2,1);
y2=Plan_path2(2,2);
dy = (y2-y1);
dx = (x2-x1);
if(x2>=x1&&y2>y1)
    angle1 =  atand(dy/dx);
elseif(x2<x1&&y2>=y1)
    angle1 = 180 + atand(dy/dx);
elseif(x2<=x1&&y2<y1)
    angle1 =  atand(dy/dx)+180;
elseif(x2>x1&&y2<=y1)
    angle1 =  360+atand(dy/dx);
else
    angle1 =  0;
end
angle1=-((360-angle1)/180)*pi;
angle_node=angle1;

% 机器人速度参数
% Kinematic = [   最高速度[m/s], 最高旋转速度[rad/s], 加速度[m/ss], 旋转加速度[rad/ss], 速度分辨率[m/s], 转速分辨率[rad/s]  ]
%速度分辨率为0.05不要动，动了速度会呈现锯齿变化
yuce=15;
Kinematic=[7.7175,toRadian(0.3244*10*yuce),0.1642,toRadian(0.0167*10*yuce),0.01,toRadian(0.5)];%5秒转25度，根据模型计算，给予模型三向适当力矩，求得船舶在航行时的转艏加速度
% 评价函数系数设置 [heading,dist,velocity,predictDT]
%  [方位角评价函数系数， 障碍物距离评价函数系数， 当前速度大小评价函数系数, 预测是时间 （不变）]
% %% 方位角系数自适应
% % alpha1=[];
% % alpha_min=0.1;
% % alpha_max=0.3;
% % theta_1=angle_node*180/pi;
% % rourou=-1*(theta_1-90);
% % if 90>=theta_1 && theta_1>=-90
% %     alpha=alpha_min+abs(0.5*alpha_max/180)*abs(rourou);
% % else
% %     alpha=alpha_max-abs(0.5*alpha_max/180)*abs(rourou);
% % end
% %       alpha1=[alpha;alpha1];
% % distancetostart=S;
% % xishu=1;
% % Dx=xishu*(Kinematic(1)/Kinematic(3));
% alpha_min=0.1;
% alpha_max=0.5;
% % if Dx>=abs(dist)
%
% theta_1=angle_node*180/pi;
% rourou=(theta_1-90);
% if 90>=theta_1 && theta_1>=-90
%     alpha=alpha_min+abs(0.5*alpha_max/180)*abs(rourou);
% else
%     alpha=alpha_max-abs(0.5*alpha_max/180)*abs(rourou);
% end
% % else
% %     alpha=alpha_max;
% % end
% % 根据Alpha动态调整Beta和Gama
% alpha_threshold = 0.3; % Alpha的阈值，用于区分高低Alpha值
%
% betal=[];
% if alpha > alpha_threshold
%     % 当需要大幅度调整方向时，增加对障碍物的敏感度
%     beta = 0.5; % Beta值较高
% else
%     % 当方向调整较小，可以加速接近目标
%     beta = 0.1; % Beta值较低
% end
% betal=[beta;betal];
% % Gama的调整逻辑与Beta相似，但是方向相反
% gamal=[];
% if alpha > alpha_threshold
%     % 当需要大幅度调整方向时，减少速度，避免冲过目标或撞上障碍物
%     gama = 0.1; % Gama值较低
% else
%     % 当方向调整较小，可以安全加速
%     gama = 0.5; % Gama值较高
% end
%
%
%
%
% %% 第二种自适应参数方法
%
% % beta=0.2;
% % %  beta =0.1.*(x(4)+x(5))*(distancetostart./dist);%安全间隙评价
% % %            if beta<0.1
% % %                beta=0.1;
% % %            elseif beta>0.3
% % %                beta=0.3;
% % %            else
% % %                beta =0.1.*(x(4)+x(5))*(distancetostart./dist);
% % %            end
% %
% %
% %
% %
% %           gama_max=1.1;
% %             gama_min=0.1;
% %             if Dx>=abs(dist)
% %                 gama=gama_min+(gama_max-gama_min)*(abs(dist)/Dx);
% %             else
% %                 gama=gama_max;
% %             end

%%  评价系数
evalParam=[0.1,0.6,0.3,0.5,yuce];%最后一个参数看预测轨迹提前多少秒
path_node=Plan_path2;
Result_x=DWA_ct_dong(Obs_Closed,Obst_d_d_line,Obst_d_d_line1,Obst_d_d_line2,Obs_d_j,Area_MAX,Goal,Plan_path2,path_node,Start,angle_node,Kinematic,evalParam);




%%%%%%%%%%% 画图
axis([1 MAX_X+1, 1 MAX_Y+1])                %%%  设置x，y轴上下限
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',...
    'xGrid','on','yGrid','on')
aaa=linspace(121.7260,122.0380,4);
xticks([1.0000   34.3333   67.6667  101.0000])
xticklabels({'121.7260'  '121.8300'  '121.9340'  '122.0380'});
set(gca,'XTickLabelRotation',360);
bbb=linspace(29.9130,30.1410,4);
yticks([1.0000   34.3333   67.6667  101.0000])
yticklabels({'29.9130'   '29.9890'   '30.0650'   '30.1410'});
xlabel('经度/(°)')
ylabel('纬度/(°)')

grid off; 
hold on;

for i_obs=1:1:num_obc
    x_obs=Obs_Closed(i_obs,1);
    y_obs=Obs_Closed(i_obs,2);
    fill([x_obs,x_obs+1,x_obs+1,x_obs],[y_obs,y_obs,y_obs+1,y_obs+1],'k');hold on;
end
plot( Plan_path2(:,1)+.5, Plan_path2(:,2)+.5,'b:','linewidth',1.5);
plot(xStart+.5,yStart+.5,'b^');
plot(Goal(1,1)+.5,Goal(1,2)+.5,'bo');



%%
num_o=size(Obst_d_d_line,1);
x_do=Obst_d_d_line(num_o,1);
y_do=Obst_d_d_line(num_o,2);
%  fill([x_do+0.15,x_do+0.85,x_do+0.85,x_do+0.15],[y_do+0.15,y_do+0.15,y_do+0.85,y_do+0.85],'m');
num_lin=size(Plan_path2,1);
for i_lin=2:1:num_lin-1
    plot(Plan_path2(i_lin,1)+.5,Plan_path2(i_lin,2)+.5,'r*');
end


num_x=size(Result_x,1);
Result_plot=[Result_x;Goal(1,1) Goal(1,2) Result_x(num_x,3) 0 0];
plot(Result_x(:,1)+0.5, Result_x(:,2)+0.5,'b','linewidth',2);hold on;
num_p=num_x+1;
ti=1:1:num_p;
figure
plot(ti,Result_plot(:,3),'-b');hold on;
legend('姿态角度')
figure
plot(ti,Result_plot(:,4),'-b');hold on;
plot(ti,Result_plot(:,5),'-r');hold on;
legend('线速度','角速度')
S=0;
for i=1:1:num_x  %%%% 求路径所用的实际长度
    Dist=sqrt( ( Result_plot(i,1) - Result_plot(i+1,1) )^2 + ( Result_plot(i,2) - Result_plot(i+1,2))^2);
    S=S+Dist;
end
disp('路径长度')
S
%
%  % 机器人的状态Result_x=[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
%  i=1;
%  figure
%  axis([0 2000, -0.4 0.8])                %%%  设置x，y轴上下限
%  set(gca,'xtick',0:100:2100,'ytick',-0.4:0.2:0.8,'GridLineStyle','-',...
%     'xGrid','on','yGrid','on')
%  grid off;
% xlabel('控制节点个数');hold on
% ylabel('线速度(m/s) 角速度(rad/s)');hold on
%
% plot(ti,Result_plot(:,4),'-b','linewidth',1.5);hold on;
% plot(ti,Result_plot(:,5),':r','linewidth',1.5);hold on;


