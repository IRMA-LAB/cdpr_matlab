function [dp,dv,da] = ComputeExcitation(cdpr_p,A,eigenvectors,f,phase,T_max)

dp = [];
dv = dp;
da = dp;
tt = linspace(0,T_max,100);
for t=tt
    dpose_f = zeros(cdpr_p.pose_dim-cdpr_p.n_cables,1);
    ddpose_f = zeros(cdpr_p.pose_dim-cdpr_p.n_cables,1);
    dddpose_f = zeros(cdpr_p.pose_dim-cdpr_p.n_cables,1);
    for i=1:length(f)
        dpose_f = dpose_f + A(i).*eigenvectors(:,i).*cos(2*pi*f(i)*t-phase(i));
        ddpose_f = ddpose_f - 2*pi*f(i)*A(i).*eigenvectors(:,i).*sin(2*pi*f(i)*t-phase(i));
        dddpose_f = dddpose_f - (2*pi*f(i))^2* A(i).*eigenvectors(:,i).*cos(2*pi*f(i)*t-phase(i));
    end
    dp = [dp dpose_f];
    dv = [dv ddpose_f];
    da = [da dddpose_f];
end

end