function sol = HuenSolver(f,t,p0)

n = length(t);
sol.t = t;
dim = length(p0);
sol.y = zeros(dim,n);
sol.f = zeros(1,n);
sol.y(:,1) = p0;
app = f(t(1),p0);
sol.f(1:dim/2,1) = app(dim/2+1:dim,1);
h = t(2)-t(1);

c = [0;2/3];
b = [1/4;3/4];
M = [0 0 ;
    2/3 0];
K = zeros(dim,2);

for i=2:n
    
    F = zeros(dim,1);
    
    for j = 1:2
        
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