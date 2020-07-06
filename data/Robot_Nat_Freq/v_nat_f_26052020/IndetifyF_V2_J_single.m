function [vect,Jac] = IndetifyF_V2_J_single(x,v,n_f)

l = length(v);
time = 0;
single_mode_var = length(x)/n_f;
vect = zeros(l,1);
    for k=1:n_f
        mod(k).Ja = zeros(l,1);
        mod(k).Jz = zeros(l,1);
        mod(k).Jf = zeros(l,1);
        mod(k).Jphi = zeros(l,1);
    end
for t=1:l
    if (~isnan(v(t)))
        vect(t) = -v(t);
        for k=1:n_f
            z = x(k*single_mode_var-2);
            f = x(k*single_mode_var-1);
            phi = x(k*single_mode_var);
            A = x(k*single_mode_var-3);
            vect(t) = vect(t)+A.*exp(-2*pi*z*f.*time).*cos(2*pi*f.*time+phi); 
            mod(k).Ja(t,1) = exp(-2*pi*z*f.*time).*cos(2*pi*f.*time+phi);
            mod(k).Jz(t,1) = A.*(-2*pi*f.*time).*exp(-2*pi*z*f.*time).*cos(2*pi*f.*time+phi);
            mod(k).Jf(t,1) = A.*(-2*pi*z.*time.*exp(-2*pi*z*f.*time).*cos(2*pi*f.*time+phi)-...
                (2*pi*time).*exp(-2*pi*z*f.*time).*sin(2*pi*f.*time+phi));
            mod(k).Jphi(t,1) = -A.*exp(-2*pi*z*f.*time).*sin(2*pi*f.*time+phi);
        end
    end
    time = time+0.01;
end

Jac = zeros(l,length(x));
for k=1:n_f
    Jac(:,k*single_mode_var-3) = mod(k).Ja;
    Jac(:,k*single_mode_var-2) = mod(k).Jz;
    Jac(:,k*single_mode_var-1) = mod(k).Jf;
    Jac(:,k*single_mode_var) = mod(k).Jphi;
end

end