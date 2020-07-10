function [vect,Jac] = IndetifyF_V2_J(x,v,n_f,n_points,n_coordinates)

num_var = length(x)/n_f;
l = length(v(1).p(:,1));
time = 0;
single_mode_var = 3+n_points*n_coordinates;
for i=1:n_points
    for j=1:n_coordinates
        res(i).f(:,j) = zeros(l,1);
        for k=1:n_f
            res(i).mod(k).Ja(:,j) = zeros(l,1);
            res(i).mod(k).Jz(:,j) = zeros(l,1);
            res(i).mod(k).Jf(:,j) = zeros(l,1);
            res(i).mod(k).Jphi(:,j) = zeros(l,1);
        end
    end
end
for t=1:l
    for i=1:n_points
        for j=1:n_coordinates
            if (~isnan(v(i).p(t,j)))
                res(i).f(t,j) = -v(i).p(t,j);
                for k=1:n_f
                    z = x(k*single_mode_var-2);
                    f = x(k*single_mode_var-1);
                    phi = x(k*single_mode_var);
                    A = x((k-1)*single_mode_var+(i-1)*n_coordinates+j);
                    res(i).f(t,j) = res(i).f(t,j)+A.*exp(-2*pi*z*f.*time).*cos(2*pi*f.*time+phi); 
                    res(i).mod(k).Ja(t,j) = exp(-2*pi*z*f.*time).*cos(2*pi*f.*time+phi);
                    res(i).mod(k).Jz(t,j) = A.*(-2*pi*f.*time).*exp(-2*pi*z*f.*time).*cos(2*pi*f.*time+phi);
                    res(i).mod(k).Jf(t,j) = A.*(-2*pi*z.*time.*exp(-2*pi*z*f.*time).*cos(2*pi*f.*time+phi)-...
                        (2*pi*time).*exp(-2*pi*z*f.*time).*sin(2*pi*f.*time+phi));
                    res(i).mod(k).Jphi(t,j) = -A.*exp(-2*pi*z*f.*time).*sin(2*pi*f.*time+phi);
                end
            end
        end
    end
    time = time+0.01;
end

vect = [];
Jac = [];
for i=1:n_points
    for j=1:n_coordinates
        vect = [vect;res(i).f(:,j)];
        jac = zeros(l,length(x));
        for k=1:n_f
            jac(:,(k-1)*single_mode_var+(i-1)*n_coordinates+j) = res(i).mod(k).Ja(:,j);
            jac(:,k*single_mode_var-2) = res(i).mod(k).Jz(:,j);
            jac(:,k*single_mode_var-1) = res(i).mod(k).Jf(:,j);
            jac(:,k*single_mode_var) = res(i).mod(k).Jphi(:,j);
        end
        Jac = [Jac;jac];
    end
end

end
