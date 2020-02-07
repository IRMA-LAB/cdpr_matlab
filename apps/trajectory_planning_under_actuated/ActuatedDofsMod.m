function dofs = ActuatedDofsMod(sim_data,tau,geoFun,p0,p1,T,ind)

    g = Poly656(tau,sim_data.lims(:,ind),sim_data.cMat);
    geometricDofs = geoFun(g(1),p0,p1);
    for i=1:length(p0)
       dofs(:,i) = [geometricDofs(1,i);...
           geometricDofs(2,i)*g(2)/T;...
           (geometricDofs(3,i)*g(2)^2+geometricDofs(2,i)*g(3))/(T^2)];
    end

end