function dofs = ShapedActuatedDofs(sim_data,tau,p0,p1)

    g = zeros(3,1);
    T = sim_data.dt;
    for j=1:sim_data.shaperL
        tau_d = sim_data.shaper(2,j)./T;
        g = g+sim_data.shaper(1,j).*Poly656(tau-tau_d,sim_data.lim,sim_data.cMat);
    end

    geometricDofs = sim_data.geometricFunction(g(1),p0,p1);
    for i=1:length(p0)
       dofs(:,i) = [geometricDofs(1,i);...
           geometricDofs(2,i)*g(2)/T;...
           (geometricDofs(3,i)*g(2)^2+geometricDofs(2,i)*g(3))/(T^2)];
    end
    
end