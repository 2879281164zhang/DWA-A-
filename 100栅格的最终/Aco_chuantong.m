clear
clc
tic
I=imread('222.jpg');   %读入图片
I = rgb2gray(I);     %将图片转为灰度图
a=50; 
b=50; 
l=1;    %网格边长
B = imresize(I,[a/l b/l]);%   将数字矩阵转为规定的大小
G=floor(B/255);
% load matlabk.mat
G=~G;
MM=size(G,1);                  	   % G 地形图为01矩阵，如果为1表示障碍物 
figure 
axis([0,MM,0,MM]) %设置图的横纵坐标，MM为地图矩阵的行数或列数
for i=1:MM 
for j=1:MM 
if G(i,j)==1 %1是黑色代表障碍，0为白色无障碍
x1=j-1;y1=MM-i; 
x2=j;y2=MM-i; 
x3=j;y3=MM-i+1; 
x4=j-1;y4=MM-i+1; 
fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.2,0.2,0.2]); %将1234点所围成的图形进行黑色填充
hold on 
else 
x1=j-1;y1=MM-i; 
x2=j;y2=MM-i; 
x3=j;y3=MM-i+1; 
x4=j-1;y4=MM-i+1; 
fill([x1,x2,x3,x4],[y1,y2,y3,y4],[1,1,1]); %将1234点所围成的图形进行白色填充
hold on 

end 
end 
end 
hold on 




% set(gca,'xtick',1:1:MM,'ytick',1:1:MM,'GridLineStyle','-',... 
%     'xGrid','on','yGrid','on')
% aaa=linspace(120.21,120.32,4);
% xticklabels({[aaa(1,1)],[],[],[],[],[],[],[],[],[],[],[aaa(1,2)],[],[],[],[],[],[],[],[],[],[],[],[aaa(1,3)],[],[],[],[],[],[],[],[],[],[],[aaa(1,4)]});
% set(gca,'XTickLabelRotation',360);
% bbb=linspace(36.09,36,4);
% yticklabels({[bbb(1,1)],[],[],[],[],[],[],[],[],[],[],[bbb(1,2)],[],[],[],[],[],[],[],[],[],[],[],[bbb(1,3)],[],[],[],[],[],[],[],[],[],[],[bbb(1,4)]});
xlabel('经度/(°)')
ylabel('纬度/(°)')