clear;clc;
rgb=imread("shange.jpg");
I=rgb2gray(rgb);
% imshow(I)
gmag = imgradient(I);
% imshow(gmag,[])
se = strel('disk',13);
Io = imopen(I,se);
% imshow(Io)
Ie = imerode(I,se);
Iobr = imreconstruct(Ie,I);
% imshow(Iobr)
Ioc = imclose(Io,se);
% imshow(Ioc)
Iobrd = imdilate(Iobr,se);
Iobrcbr = imreconstruct(imcomplement(Iobrd),imcomplement(Iobr));
Iobrcbr = imcomplement(Iobrcbr);
% imshow(Iobrcbr)
% Iobrcbr=double(Iobrcbr);%记得注释
% x=Iobrcbr(:);
% x=sort(x); % 数据排序
% d=diff([x;max(x)+1]); % 通过同一数据为0 找标识
% count = diff(find([1;d])) ; % 找到d里面的非0的位置，
% y =[x(find(d)) count]; % 打印结果
% a=Iobrcbr<179;
% Iobrcbr(a)=2;
% bb=find(179<=Iobrcbr&Iobrcbr<191);
% Iobrcbr(bb)=2;
% c=find(191<=Iobrcbr&Iobrcbr<208);
% Iobrcbr(c)=2;
% d=find(208<=Iobrcbr&Iobrcbr<217);
% Iobrcbr(d)=2;
% e=find(217<=Iobrcbr&Iobrcbr<=232);
% Iobrcbr(e)=1;
% f=find(Iobrcbr==233);
% Iobrcbr(f)=1;
imshow(Iobrcbr)
figure(1)
a=100; 
b=100; 
l=1;    %网格边长
B = imresize(Iobrcbr,[a/l b/l]);%   将数字矩阵转为规定的大小
graph=double(B);
X = size(graph,1);      % 问题的状态空间矩阵的行数
Y = size(graph,2);      % 问题的状态空间矩阵的列数
left_down_lo=122.1406;        % 规划区域左下角经度rgb
left_down_la=29.9796;        %规划区域左下角纬度
right_up_lo=122.1600;         %规划区域右上角经度
right_up_la=29.9933;          %规划区域右上角纬度
jingdu_jiange=(right_up_lo-left_down_lo)/a;
weidu_jiange=(right_up_la-left_down_la)/b;
xx=zeros(20,20);
% for i=1:X
%     for j=1:Y
%         if graph(i,j)==0
%             x1=left_down_lo+jingdu_jiange*j;   y1=left_down_la+weidu_jiange*(Y-i);
%             x2=left_down_lo+jingdu_jiange*j;   y2=left_down_la+weidu_jiange*(Y-i+1);
%             x3=left_down_lo+jingdu_jiange*(j-1); y3=left_down_la+weidu_jiange*(Y-i+1);
%             x4=left_down_lo+jingdu_jiange*(j-1); y4=left_down_la+weidu_jiange*(Y-i);
%             z=fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.1,0.1,0.1]);
%             set(z,{'LineStyle'},{'none'})
%             hold on
%             if 108.563<x3 && x3<108.799 && 21.5509<y3 && y3<21.7398 && graph(i,j)==0
%                 graph(i,j)=3;
%                 x1=left_down_lo+jingdu_jiange*j;   y1=left_down_la+weidu_jiange*(Y-i);
%                 x2=left_down_lo+jingdu_jiange*j;   y2=left_down_la+weidu_jiange*(Y-i+1);
%                 x3=left_down_lo+jingdu_jiange*(j-1); y3=left_down_la+weidu_jiange*(Y-i+1);
%                 x4=left_down_lo+jingdu_jiange*(j-1); y4=left_down_la+weidu_jiange*(Y-i);
%                 v=fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.65,0.25,0.2]);
%                 set(v,{'LineStyle'},{'none'})
%                 hold on
%             end
%         elseif graph(i,j)==1
%             x1=left_down_lo+jingdu_jiange*j;   y1=left_down_la+weidu_jiange*(Y-i);
%             x2=left_down_lo+jingdu_jiange*j;   y2=left_down_la+weidu_jiange*(Y-i+1);
%             x3=left_down_lo+jingdu_jiange*(j-1); y3=left_down_la+weidu_jiange*(Y-i+1);
%             x4=left_down_lo+jingdu_jiange*(j-1); y4=left_down_la+weidu_jiange*(Y-i);
%             x=fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.4 0.15 0.2]);
%             set(x,{'LineStyle'},{'none'})
%             hold on
%         elseif graph(i,j)==2
%             x1=left_down_lo+jingdu_jiange*j;   y1=left_down_la+weidu_jiange*(Y-i);
%             x2=left_down_lo+jingdu_jiange*j;   y2=left_down_la+weidu_jiange*(Y-i+1);
%             x3=left_down_lo+jingdu_jiange*(j-1); y3=left_down_la+weidu_jiange*(Y-i+1);
%             x4=left_down_lo+jingdu_jiange*(j-1); y4=left_down_la+weidu_jiange*(Y-i);
%             c=fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.7,0.2,0.25]);
%             set(c,{'LineStyle'},{'none'})
%             hold on
%         elseif graph(i,j)==3
%             x1=left_down_lo+jingdu_jiange*j;   y1=left_down_la+weidu_jiange*(Y-i);
%             x2=left_down_lo+jingdu_jiange*j;   y2=left_down_la+weidu_jiange*(Y-i+1);
%             x3=left_down_lo+jingdu_jiange*(j-1); y3=left_down_la+weidu_jiange*(Y-i+1);
%             x4=left_down_lo+jingdu_jiange*(j-1); y4=left_down_la+weidu_jiange*(Y-i);
%             v=fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.65,0.25,0.2]);
%             set(v,{'LineStyle'},{'none'})
%             hold on
%             if  109.06<x3 && x3<109.191 && 20.9638<y3 && y3<21.106 && graph(i,j)==3
%                 graph(i,j)=0;
%                 x1=left_down_lo+jingdu_jiange*j;   y1=left_down_la+weidu_jiange*(Y-i);
%                 x2=left_down_lo+jingdu_jiange*j;   y2=left_down_la+weidu_jiange*(Y-i+1);
%                 x3=left_down_lo+jingdu_jiange*(j-1); y3=left_down_la+weidu_jiange*(Y-i+1);
%                 x4=left_down_lo+jingdu_jiange*(j-1); y4=left_down_la+weidu_jiange*(Y-i);
%                 z=fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.1,0.1,0.1]);
%                 set(z,{'LineStyle'},{'none'})
%                 hold on
%             end
%         elseif graph(i,j)==4
%             x1=left_down_lo+jingdu_jiange*j;   y1=left_down_la+weidu_jiange*(Y-i);
%             x2=left_down_lo+jingdu_jiange*j;   y2=left_down_la+weidu_jiange*(Y-i+1);
%             x3=left_down_lo+jingdu_jiange*(j-1); y3=left_down_la+weidu_jiange*(Y-i+1);
%             x4=left_down_lo+jingdu_jiange*(j-1); y4=left_down_la+weidu_jiange*(Y-i);
%             b=fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.93,0.5,0.25]);
%             set(b,{'LineStyle'},{'none'})
%             hold on
%         elseif graph(i,j)==5
%             x1=left_down_lo+jingdu_jiange*j;   y1=left_down_la+weidu_jiange*(Y-i);
%             x2=left_down_lo+jingdu_jiange*j;   y2=left_down_la+weidu_jiange*(Y-i+1);
%             x3=left_down_lo+jingdu_jiange*(j-1); y3=left_down_la+weidu_jiange*(Y-i+1);
%             x4=left_down_lo+jingdu_jiange*(j-1); y4=left_down_la+weidu_jiange*(Y-i);
%             n=fill([x1,x2,x3,x4],[y1,y2,y3,y4],[1,0.83,0.67]);
%             set(n,{'LineStyle'},{'none'})
%             hold on
%             end
%         end
% end
xuyao=find(B>0);
B(xuyao)=1;
xlim([left_down_lo,right_up_lo])
ylim([left_down_la,right_up_la])
set(gca,'ytick',linspace(left_down_la,right_up_la,4))
set(gca,'YTickLabel',linspace(left_down_la,right_up_la,4)); %使得地图的矩阵的行列与正常坐标轴的行列一致
set(gca,'xtick',linspace(left_down_lo,right_up_lo,4))
set(gca,'XTickLabel',linspace(left_down_lo,right_up_lo,4))
xlabel('经度/(°)')
ylabel('纬度/(°)')
hold on

% map=[0.1,0.1,0.1;
%     0.4 0.15 0.2;
%     0.7,0.2,0.25;
%     0.65,0.25,0.2;
%     0.93,0.5,0.25;
%     1,0.83,0.67];
% colormap(map)
% colorbar;
% c=colorbar;
% set(c,'YTick',0:0.2:1); %色标值范围及显示间隔
% set(c,'YTickLabel',{'0','1','2','3','4','5'}) %具体刻度赋值