clear;
clc;
close all;
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
a=35; 
b=35; 
l=1;    %网格边长
B = imresize(Iobrcbr,[a/l b/l]);%   将数字矩阵转为规定的大小
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
B(7,12)=1;
B(8,13)=1;
B(6,14:15)=1;
B(14,22)=1;
%%%只能设置正方形矩阵，行和列相等，否则旋转时会出现错误
%%%只能设置正方形矩阵，行和列相等，否则旋转时会出现错误
%  0 表示无障碍物  1表示有障碍物
MAX0 = B;
MAX=rot90(MAX0,3);      %%%设置0,1摆放的图像与存入的数组不一样，需要先逆时针旋转90*3=270度给数组，最后输出来的图像就是自己编排的图像
MAX_X=size(MAX,2);                                %%%  获取列数，即x轴长度
MAX_Y=size(MAX,1);                                %%%  获取行数，即y轴长度
MAX_VAL=10;                              %%%   返回由数字组成的字符表达式的数字值，就是函数用于将数值字符串转换为数值
x_val = 1;
y_val = 1;
start_sub=[x_val,y_val];%起点坐标
axis([1 MAX_X+1, 1 MAX_Y+1])                %%%  设置x，y轴上下限

xlabel('经度/(°)')
ylabel('纬度/(°)')
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',... 
    'xGrid','on','yGrid','on')
aaa=linspace(121.7260,122.0380,4);
xticklabels({[aaa(1,1)],[],[],[],[],[],[],[],[],[],[],[aaa(1,2)],[],[],[],[],[],[],[],[],[],[],[],[aaa(1,3)],[],[],[],[],[],[],[],[],[],[],[],[aaa(1,4)]});
set(gca,'XTickLabelRotation',360);
bbb=linspace(30.1410,29.9130,4);
yticklabels({[bbb(1,1)],[],[],[],[],[],[],[],[],[],[],[bbb(1,2)],[],[],[],[],[],[],[],[],[],[],[],[bbb(1,3)],[],[],[],[],[],[],[],[],[],[],[],[bbb(1,4)]});
grid on;                                   %%%  在画图的时候添加网格线
hold on;                                   %%%  当前轴及图像保持而不被刷新，准备接受此后将绘制的图形，多图共存
n=0;%Number of Obstacles                   %%%  障碍的数量
k=1;          %%%% 将所有障碍物放在关闭列表中；障碍点的值为1;并且显示障碍点
CLOSED=[0 0];
for j=1:MAX_X
    for i=1:MAX_Y
        if (MAX(i,j)==1)
          %%plot(i+.5,j+.5,'ks','MarkerFaceColor','b'); 原来是红点圆表示
          fill([i,i+1,i+1,i],[j,j,j+1,j+1],'k');  %%%改成 用黑方块来表示障碍物
          CLOSED(k,1)=i;  %%% 将障碍点保存到CLOSE数组中
          CLOSED(k,2)=j; 
          k=k+1;
        end
    end
end
Area_MAX(1,1)=MAX_X;
Area_MAX(1,2)=MAX_Y;
Obs_Closed=CLOSED;
Num_obs=size(CLOSED,1); %%%存储障碍物的数量  *********************************************************************
xval=35;%floor(xval);                                              %%%  floor（）取不大于传入值的最大整数，向下取整
yval=1;%floor(yval);
goal=[30,1];%终点坐标
xTarget=xval;%X Coordinate of the Target                       %%%   目标的坐标
yTarget=yval;%Y Coordinate of the Target
Target(1,1)=xTarget;
Target(1,2)=yTarget;
MAP(xval,yval) = -1 ;                      %%%   目标坐标点位置的值设为-1                                 %%%   目标点颜色b 蓝色 g 绿色 k 黑色 w白色 r 红色 y黄色 m紫红色 c蓝绿色
xStart=1;%Starting Position
yStart=35;%Starting Position
Start(1,1)=xStart;
Start(1,2)=yStart;
MAP(xval,yval)=2;                                                 %%%   起始点位置的值设置为1；目标点为0，障碍点为-1，其余空白点为2
hold on;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              3      全局静态路径规划 蚁群算法
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
G=MAX0;
MM=MAX_X;
Tau=ones(MM*MM,MM*MM);             % Tau 初始信息素矩阵
Tau=8.*Tau;
%第一条航道
duo00=[];
for j00=1:4
ax00=linspace(1,4,4);
ay00=linspace(2,5,4);
duo00=[duo00,ax00(j00)*35+ay00(j00)];
end
duo0=[];
for j0=1:11
    ax0=linspace(6,16,11);
    ay0=4;
    duo0=[duo0,ay0*35+ax0(j0)];
end
duo=[];
for j1=1:7
ax=linspace(6,12,7);
ay=linspace(18,24,7);
duo=[duo,ax(j1)*35+ay(j1)];
end
duo1=[];
for j2=1:12
ax1=24;
ay1=linspace(12,23,12);
duo1=[duo1,ay1(j2)*35+ax1];
end
duo2=[];
for j3=1:12
ax2=linspace(24,35,12);
ay2=linspace(23,34,12);
duo2=[duo2,ay2(j3)*35+ax2(j3)];
end
%第二条航道
buo00=[];
for i00=1:8
bx00=linspace(1,8,8);
by00=linspace(2,9,8);
buo00=[buo00,bx00(i00)*35+by00(i00)];
end
buo1=[];
for i2=1:15
bx1=9;
by1=linspace(9,23,15);
buo1=[buo1,(by1(i2)-1)*35+bx1];
end
buo=[];
for i1=1:6
bx=linspace(22,27,6);
by=linspace(9,14,6);
buo=[buo,bx(i1)*35+by(i1)];
end
buo0=[];
for i0=1:14
    bx0=26;
    by0=linspace(14,27,14);
    buo0=[buo0,bx0*35+by0(i0)];
end
duo2=[];
for j3=1:12
ax2=linspace(24,35,12);
ay2=linspace(23,34,12);
duo2=[duo2,ay2(j3)*35+ax2(j3)];
end
Tau(:,[duo00,duo0,duo,duo1,duo2])=1600;
Tau(:,[buo00,buo1,buo,buo0,duo2])=1600;
K=100;                       	   %迭代次数（指蚂蚁出动多少波）
M=50;                        	   %蚂蚁个数
S=xStart+MM*(MM-yStart) ;                         	   %最短路径的起始点
E=xTarget+MM*(MM-yTarget) ;                           %最短路径的目的点
Alpha=1;                      	   % Alpha 表征信息素重要程度的参数
Beta=7;                       	   % Beta 表征启发式因子重要程度的参数
Rho=0.1 ;                      	   % Rho 信息素蒸发系数
Q=1;                               % Q 信息素增加强度系数
minkl=inf;
mink=0;
minl=0;
D=G2D(G);
N=size(D,1);               %N表示问题的规模（象素个数）
a=1;                     %小方格象素的边长
Ex=a*(mod(E,MM)-0.5);    %终止点横坐标
if Ex==-0.5
    Ex=MM-0.5;
end
Ey=a*(MM+0.5-ceil(E/MM)); %终止点纵坐标
Eta=zeros(N);             %启发式信息，取为至目标点的直线距离的倒数
%以下启发式信息矩阵
for i=1:N
    ix=a*(mod(i,MM)-0.5);
    if ix==-0.5
        ix=MM-0.5;
    end
    iy=a*(MM+0.5-ceil(i/MM));
    if i~=E
        Eta(i)=(1/((ix-Ex)^2+(iy-Ey)^2)^0.5)^2;
    else
        Eta(i)=100;
    end
%     Eta(i) = Eta(i);
end
ROUTES=cell(K,M);       %用细胞结构存储每一代的每一只蚂蚁的爬行路线
PL=zeros(K,M);          %用矩阵存储每一代的每一只蚂蚁的爬行路线长度
%启动K轮蚂蚁觅食活动，每轮派出M只蚂蚁
for k=1:K
    fprintf('第%d代蚂蚁\n',k)
    %k
%     Rho = 0.2*k/K;
    Rho = 0.6*(1-exp(-k/K*1000));
    for m=1:M
        %状态初始化
        W=S;                  %当前节点初始化为起始点
        Path=S;                %爬行路线初始化
        PLkm=0;               %爬行路线长度初始化
        TABUkm=ones(N);       %禁忌表初始化
        TABUkm(S)=0;          %已经在初始点了，因此要排除
        DD=D;                 %邻接矩阵初始化
        %下一步可以前往的节点
        DW=DD(W,:);
        DW1=find(DW);
        for j=1:length(DW1)
            if TABUkm(DW1(j))==0
                DW(DW1(j))=0;
            end
        end
        LJD=find(DW);
        Len_LJD=length(LJD);%可选节点的个数
        %蚂蚁未遇到食物或者陷入死胡同或者觅食停止
        while W~=E&&Len_LJD>=1
            %转轮赌法选择下一步怎么走
            PP=zeros(Len_LJD);
            for i=1:Len_LJD
                PP(i)=(Tau(W,LJD(i))^Alpha)*((Eta(LJD(i)))^Beta);
            end
            sumpp=sum(PP);
            PP=PP/sumpp;%建立概率分布
            Pcum(1)=PP(1);
            for i=2:Len_LJD
                Pcum(i)=Pcum(i-1)+PP(i);
            end
            Select=find(Pcum>=rand);
            to_visit=LJD(Select(1));
            %状态更新和记录
            Path=[Path,to_visit];        %路径增加
            PLkm=PLkm+DD(W,to_visit);    %路径长度增加
            W=to_visit;                  %蚂蚁移到下一个节点
            for kk=1:N
                if TABUkm(kk)==0
                    DD(W,kk)=0;
                    DD(kk,W)=0;
                end
            end
            TABUkm(W)=0;				 %已访问过的节点从禁忌表中删除
            DW=DD(W,:);
            DW1=find(DW);
            for j=1:length(DW1)
                if TABUkm(DW1(j))==0
                    DW(j)=0;
                end
            end
            LJD=find(DW);
            Len_LJD=length(LJD);%可选节点的个数
        end 
        %记下每一代每一只蚂蚁的觅食路线和路线长度
        ROUTES{k,m}=Path;
        if Path(end)==E
            PL(k,m)=PLkm;
            if PLkm<minkl
                mink=k;
                minl=m;
                minkl=PLkm;
                Route_Shortest = Path;
            end
        else
            PL(k,m)=0;
        end
    end
    %更新信息素
    Delta_Tau=zeros(N,N);%更新量初始化
    for m=1:M
        if PL(k,m)
            ROUT=ROUTES{k,m};
            TS=length(ROUT)-1;      %跳数
            PL_km=PL(k,m);
            for s=1:TS
                x=ROUT(s);
                y=ROUT(s+1);
                Delta_Tau(x,y)=Delta_Tau(x,y)+Q/PL_km;
                Delta_Tau(y,x)=Delta_Tau(y,x)+Q/PL_km;
            end
        end
    end
    Tau=(1-Rho).*Tau+Delta_Tau;     %信息素挥发一部分，新增加一部分
%     %计算最大最小信息素    
%     Tau_max = (1/(2*(1-Rho))*1/minkl + 1/minkl)*200;
%     Tau_min = Tau_max/500;
%     Tau(find(Tau > Tau_max)) = Tau_max;
%     Tau(find(Tau < Tau_min)) = Tau_min
    %minkl;
end
% 绘图
    ROUT=Route_Shortest;
    LENROUT=length(ROUT);
    Rx=ROUT;
    Ry=ROUT;
    for ii=1:LENROUT
        Rx(ii)=a*(mod(ROUT(ii),MM)-0.5);
        if Rx(ii)==-0.5
            Rx(ii)=MM-0.5;
        end
        Ry(ii)=a*(MM+0.5-ceil(ROUT(ii)/MM));
    end
    for in=1:LENROUT
        Plan_path(in,1)=Rx(in)+0.5;
        Plan_path(in,2)=Ry(in)+0.5;
    end
disp('全局静态规划时间')
toc
%单画蚁群
plot(xTarget+0.5,yTarget+0.5,'bo');
plot(xStart+0.5,yStart+0.5,'b^');
plot(Plan_path(:,1)+.5,Plan_path(:,2)+.5,'m','linewidth',2); %5%绘线 'b',   

%画迭代次数图
plotif=1;%是否绘图的控制参数
if plotif==1 %绘收敛曲线
    minPL=zeros(K);
    for i=1:K
        PLK=PL(i,:);
        Nonzero=find(PLK);
        PLKPLK=PLK(Nonzero);
        if (isempty(PLKPLK) && i>1)
            minPL_tmp = minPL(i-1);
            minPL(i) = minPL_tmp;
            continue;
        end
        minPL_tmp = min(PLKPLK);
        if (i > 1 && minPL_tmp > minPL(i-1))
            minPL(i) = minPL(i-1);
        else
            minPL(i) = min(PLKPLK);
        end
    end
    figure
    plot(minPL(:,1),'linewidth',1.5);
    hold on
    grid on
    xlabel('迭代次数');
    ylabel('最小路径长度'); %绘爬行图
end
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
Plan_path2=Optimal_path;
figure 
axis([1 MAX_X+1, 1 MAX_Y+1])                %%%  设置x，y轴上下限
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',...
        'xGrid','on','yGrid','on'); 
grid on;       
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
set(gca,'xtick',1:1:MAX_X+1,'ytick',1:1:MAX_Y+1,'GridLineStyle','-',... 
    'xGrid','on','yGrid','on')

aaa=linspace(121.7260,122.0380,4);
xticklabels({[aaa(1,1)],[],[],[],[],[],[],[],[],[],[],[aaa(1,2)],[],[],[],[],[],[],[],[],[],[],[],[aaa(1,3)],[],[],[],[],[],[],[],[],[],[],[],[aaa(1,4)]});
set(gca,'XTickLabelRotation',360);
bbb=linspace(30.1410,29.9130,4);
yticklabels({[bbb(1,1)],[],[],[],[],[],[],[],[],[],[],[bbb(1,2)],[],[],[],[],[],[],[],[],[],[],[],[bbb(1,3)],[],[],[],[],[],[],[],[],[],[],[],[bbb(1,4)]});
xlabel('经度/(°)')
ylabel('纬度/(°)')