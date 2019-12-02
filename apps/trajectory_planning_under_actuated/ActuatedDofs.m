function dofs = ActuatedDofs(c,tau,geoFun,p0,p1)

    vt = [tau(1)^4 tau(1)^5 tau(1)^6 tau(1)^7];
    dvt = [tau(1)^3 tau(1)^4 tau(1)^5 tau(1)^6];
    ddvt = [tau(1)^2 tau(1)^3 tau(1)^4 tau(1)^5];

    g(1) = vt*c.c;
    g(2) = dvt*c.cDerivative;
    g(3) = ddvt*c.cDerivative2;

    geometricDofs = geoFun(g(1),p0,p1,c);
    for i=1:length(p0)
       dofs(:,i) = [geometricDofs(1,i);...
           geometricDofs(2,i)*g(2)*tau(2);...
           (geometricDofs(3,i)*g(2)^2+geometricDofs(2,i)*g(3))*tau(2)^2+...
           geometricDofs(2,i)*g(2)*tau(3)];
    end

end