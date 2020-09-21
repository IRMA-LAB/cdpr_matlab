function vect = FindAmplPhase(eig_vect,f,var,par)

pos = zeros(length(par)/2,1);
vel = pos;
A = var(1:length(var)/2);
phase = var(length(var)/2+1:end);
for i=1:length(f)
    pos = pos + A(i).*eig_vect(:,i).*cos(phase(i));
    vel = vel + 2*pi*f(i).*A(i).*eig_vect(:,i).*sin(phase(i));
end

vect = [pos-par(1:length(par)/2);
        vel-par(length(par)/2+1:end)];

end