function vect = IdentifyF_Vlast(x,v,n_f)

l = length(v);
time = 0;
single_mode_var = 4;
for i=1:1
        val(1:1,:) = zeros(1,l);
end
for t=1:l
    for i=1:1
        val(i,t) = -v(i,t);
                for k=1:n_f
                    z = x(k*single_mode_var-2);
                    f = x(k*single_mode_var-1);
                    phi = x(k*single_mode_var);
                    A = x((k-1)*single_mode_var+i);
                    val(i,t) = val(i,t)+A.*exp(-2*pi*z*f.*time).*cos(2*pi*f*(sqrt(1-z^2)).*time+phi); 
                end
    end
    time = time+0.01;
end

vect = [];
for i=1:1
        vect = [vect;val(i,:)'];
end

end
