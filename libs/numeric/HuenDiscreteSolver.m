function sol = HuenDiscreteSolver(f,t,p0)

n = length(t);
dim = length(p0);

h = t(2)-t(1);

sol.y = zeros(dim,n);
sol.f = zeros(1,n);

sol.t = t;
sol.y(:,1) = p0;
app = f(t(1),p0);
sol.f(1:dim/2,1) = app(dim/2+1:dim,1);


for i=2:n
    
    y_tilde = sol.y(:,i-1) + h.*[sol.y(end/2+1:end,i-1);sol.f(:,i-1)];
    sol.y(:,i) = sol.y(:,i-1) + h./2.*([sol.y(end/2+1:end,i-1);sol.f(:,i-1)]...
        +f(t(i),y_tilde));
    app = f(t(i),sol.y(:,i));
    sol.f(1:dim/2,i) = app(dim/2+1:dim,1);
    
end

end