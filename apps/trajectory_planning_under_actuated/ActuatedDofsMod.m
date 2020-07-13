function dofs = ActuatedDofsMod(sim_data,tau,p0,p1)

    T = sim_data.dt;
    g = Poly656(tau,sim_data.lim,sim_data.cMat);
    geometricDofs = sim_data.geometricFunction(g(1),p0,p1);
    for i=1:length(p0)
       dofs(:,i) = [geometricDofs(1,i);...
           geometricDofs(2,i)*g(2)/T;...
           (geometricDofs(3,i)*g(2)^2+geometricDofs(2,i)*g(3))/(T^2)];
    end
    
end