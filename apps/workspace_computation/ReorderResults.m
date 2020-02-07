function out = ReorderResults(cdpr_p,tau_lim,out)

out.limits = CalcWorkspaceGraphicalLimits(cdpr_p,out);
out.tension_vector_limits = tau_lim;
out.workspace_center = mean(out.pose(1:3,:),2);
out.parametrization = cdpr_p.rotation_parametrization;

A = ceil(out.nat_freq.*10000);
[~,b] = unique(A','stable','rows');
len = length(b);

out.counter = len;
if cdpr_p.rotation_parametrization ~= RotationParametrizations.QUATERNION
    b = [b;length(A)];
    c = [];
    for i=1:length(b)-1
        if (i==length(b)-1)
            idx = linspace(b(i),b(i+1),b(i+1)-b(i)+1);
        else
            idx = linspace(b(i),b(i+1)-1,b(i+1)-b(i));
        end
        [~,min_idx] = min(vecnorm(out.ang_par(:,idx)));
        c = [c;b(i)+min_idx-1];
    end
    out.position = out.position(:,c);
    out.ang_par = out.ang_par(:,c);
    out.rot_mat = out.rot_mat(:,:,c);
    out.nat_freq = out.nat_freq(:,c);
    out.pose = out.pose(:,c);
    out.normal_modes = out.normal_modes(:,:,c);
    out.tension_vector = out.tension_vector(:,c);
else
    
    out.position = out.position(:,b);
    out.ang_par = out.ang_par(:,b);
    out.rot_mat = out.rot_mat(:,:,b);
    out.nat_freq = out.nat_freq(:,b);
    out.pose = out.pose(:,b);
    out.normal_modes = out.normal_modes(:,:,b);
    out.tension_vector = out.tension_vector(:,b);
end

end