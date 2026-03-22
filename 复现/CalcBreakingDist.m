function stopDist=CalcBreakingDist(vel,model)
% 
global dt;
stopDist=0;
while vel>0
    stopDist=stopDist+vel*dt;% 制动距离的计算
    vel=vel-model(3)*dt;% 
end