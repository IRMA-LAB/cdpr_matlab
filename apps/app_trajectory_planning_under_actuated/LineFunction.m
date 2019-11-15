function mat = LineFunction(t,p0,p1,s)
 
 l = length(p0);
 for i=1:l
     mat(:,i) = [p0(i)+(p1(i)-p0(i))*t;(p1(i)-p0(i));0];
 end
 
end