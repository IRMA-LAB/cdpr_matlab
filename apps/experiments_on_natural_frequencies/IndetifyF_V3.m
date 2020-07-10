function vect = IndetifyF_V3(x,v,n_f,n_points,n_coordinates)

num_var = length(x)/n_f;
l = length(v(1).p(:,1));
time = 0;
single_mode_var = 3+n_points*n_coordinates;
for i=1:n_points
    for j=1:n_coordinates
        res(i).f(:,j) = zeros(l,1);
    end
end
for t=1:l
    for i=1:n_points
        for j=1:n_coordinates
            offs = x(single_mode_var*n_f+(i-1)*n_coordinates+j);
            if (~isnan(v(i).p(t,j)))
                res(i).f(t,j) = -v(i).p(t,j)+offs;
                for k=1:n_f
                    z = x(k*single_mode_var-2);
                    f = x(k*single_mode_var-1);
                    phi = x(k*single_mode_var);
                    A = x((k-1)*single_mode_var+(i-1)*n_coordinates+j);
                    res(i).f(t,j) = res(i).f(t,j)+A.*exp(-2*pi*z*f.*time).*cos(2*pi*f.*time+phi); 
                end
            end
        end
    end
    time = time+0.01;
end

vect = [];
for i=1:n_points
    for j=1:n_coordinates
        vect = [vect;res(i).f(:,j)];
    end
end

end
