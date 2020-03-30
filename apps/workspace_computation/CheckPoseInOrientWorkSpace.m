function [out] = CheckPoseInOrientWorkSpace(cdpr_p,cdpr_v,tau_lim,out,pose,varargin)

if (~isempty(varargin))
    rec = varargin{1};
end
%options = optimoptions('linprog','Display','none');
cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v = CalcCablesStaticTension(cdpr_v);

if ((~any(isnan(cdpr_v.tension_vector))) && isempty(cdpr_v.tension_vector(cdpr_v.tension_vector>tau_lim(2)))...
        && isempty(cdpr_v.tension_vector(cdpr_v.tension_vector<tau_lim(1))))
    out.counter = out.counter+1;
    out.pose(:,out.counter) = pose;
    out.position(:,out.counter) = out.pose(1:3,out.counter);
    out.ang_par(:,out.counter) = out.pose(4:end,out.counter);
    out.rot_mat(:,:,out.counter) = cdpr_v.platform.rot_mat;
    out.tension_vector(:,out.counter) = cdpr_v.tension_vector;
    
    Kr = cdpr_v.geometric_jacobian(:,4:6);
    Kp = cdpr_v.geometric_jacobian(:,1:3);
    K = [Kp Kr];
    J = MyInv(K);
    Jp = J(1:3,:);
    Jr = J(4:6,:);
    L = [K' -K']';
    E = eye(6);
    Pp = eye(6)-Kp*MyInv(Kp'*Kp)*Kp';
    Pr = eye(6)-Kr*MyInv(Kr'*Kr)*Kr';
    out.manip(1,out.counter) = sqrt(norm(inv(Kr'*Pp*Kr),2));
    out.manip2(1,out.counter) = sqrt(norm(inv(Kp'*Pr*Kp),2));
%     for i=1:6
%     x(:,i) = linprog(-E(:,i),L,ones(12,1));
%     t(i,1) = E(:,i)'* x(:,i);
%     end
%     out.manipP(1,out.counter) = max(t(1:3));
%     out.manipR(1,out.counter) = max(t(4:6));
    out.manipP(1,out.counter) = norm(Jp,Inf);
    out.manipR(1,out.counter) = norm(Jr,Inf);
%     L = [cdpr_v.geometric_jacobian' -cdpr_v.geometric_jacobian']';
%     for ii = 1:3
%         v = zeros(6,1); v(ii+3) = 1;
%         [~,fval(ii)] =  linprog(-v,L,ones(length(L),1),[],[],[],[],options);
%         fval(ii) = -fval(ii);
%     end
%     out.manip3(1,out.counter) = max(fval);
end

end