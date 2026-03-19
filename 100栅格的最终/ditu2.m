clear
clc
I=imread('3213.jpg');   %读入图片
imshow(I)
axis on
axis equal
ylim=329;
xlim=395;
y=linspace(1,ylim,3);
x=linspace(1,xlim,3);
set(gca,'Ytick',y)
set(gca,'Yticklabel',linspace(36.110,35.960,3),'Fontsize',12)
set(gca,'Xtick',x)
set(gca,'Xticklabel',linspace(120.132,120.316,3),'Fontsize',12)
xlabel('经度/(°)')
ylabel('纬度/(°)')
box off
