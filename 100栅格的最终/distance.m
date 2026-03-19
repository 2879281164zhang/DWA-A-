function dist = distance(x1,y1,x2,y2)

dist=sqrt((x1-x2)^2 + (y1-y2)^2);
%  »¹ÓĞ Âü¹ş¶Ù¾àÀë h(n) = abs(nx - goalx) + abs(ny - goaly)
%       ÇĞ±ÈÑ©·ò¾àÀë h(n) = max[abs(nx - goalx),abs(ny - goaly)