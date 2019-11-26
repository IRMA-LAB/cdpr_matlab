function sol = RKSolver(f,t,p0)

n = length(t);
sol.t = t;
dim = length(p0);
sol.y = zeros(dim,n);
sol.f = zeros(1,n);
sol.y(:,1) = p0;
app = f(t(1),p0);
sol.f(1:dim/2,1) = app(dim/2+1:dim,1);
h = t(2)-t(1);

c = [0;1/4;3/8;12/13;1;1/2];
%b = [25/216;0;1408/2565;2197/4104;-1/5;0];
b = [16/135;0;6656/12825;28561/56430;-9/50;2/55];
M = [0 0 0 0 0 0;
    1/4 0 0 0 0 0;
    3/32 9/32 0 0 0 0;
    1932/2197 -7200/2197 7296/2197 0 0 0;
    439/216 -8 3680/513 -845/4104 0 0;
    -8/27 2 -3544/2565 1859/4104 -11/40 0];
K = zeros(dim,6);

for i=2:n
    
    F = zeros(dim,1);
    
    for j = 1:6
        
        s = zeros(dim,1);
        for k = 1:j-1
            
            s = s + M(j,k).*K(:,k);
            
        end
        
        K(:,j) = f(t(i-1)+h*c(j),sol.y(:,i-1)+h.*s);
        
        F = F + b(j).*K(:,j);
        
    end
    
    sol.y(:,i) = sol.y(:,i-1) + h.*F;
    app = f(t(i),sol.y(:,i));
    sol.f(1:dim/2,i) = app(dim/2+1:dim,1);
    
end

end