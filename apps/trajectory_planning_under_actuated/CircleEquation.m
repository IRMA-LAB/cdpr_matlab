function mat = CircleEquation(c,r,u,thE,t)

    n = Anti(u(:,1))*u(:,2);
    n = Anti(n)*u(:,1);
    n = n./norm(n);
    for i=1:3
     mat(:,i) = [c(i)+r*u(i,1)*cos(thE*t)+r*n(i)*sin(thE*t);
                 -thE*r*u(i,1)*sin(thE*t)+thE*r*n(i)*cos(thE*t);
                 -thE^2*r*u(i,1)*cos(thE*t)-thE^2*r*n(i)*sin(thE*t)];
    end

end