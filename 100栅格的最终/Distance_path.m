function S=Distance_path(Plan_path)
%%%% 求 全局静态规划的 路径长度 
S=0;
 j = size(Plan_path,1) ;
 for i=1:1:(j-1)  
     Dist=sqrt( ( Plan_path(i,1) - Plan_path(i+1,1) )^2 + ( Plan_path(i,2) - Plan_path(i+1,2))^2);
     S=S+Dist;
 end
 