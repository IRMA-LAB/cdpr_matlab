function ComputePlot_V3(x,v,n_f,n_points,n_coordinates)

num_var = length(x)/n_f;
l = length(v(1).p(:,1));
time = (1:l).*0.01-0.01;
v_an = zeros(l,1);
single_mode_var = 3+n_points*n_coordinates;
for i=1:n_coordinates
    for j=1:n_points
        subplot(n_coordinates,n_points,(i-1)*5+j)
        v_an = zeros(l,1);
        offs = x(single_mode_var*n_f+(j-1)*n_coordinates+i);
        for t = 1:l
            for k=1:n_f
                z = x(k*single_mode_var-2);
                f = x(k*single_mode_var-1);
                phi = x(k*single_mode_var);
                A = x((k-1)*single_mode_var+(j-1)*n_coordinates+i);
                v_an(t) = v_an(t)+ A.*exp(-2*pi*z*f.*time(t)).*cos(2*pi*f.*time(t)+phi); 
            end       
        end
        v_an = v_an+offs;
        plot(time,v(j).p(:,i),time,v_an)
    end
end

end
