function out = CalcWorkspaceUnder33(cdpr_p,cdpr_v,ut,tau_lim,z_lim_inf,varargin)

wp_step = 0.05;
out.counter = 0;

if (~isempty(varargin))
rec = varargin{1};
end

limits = CalcWorkspaceGraphicalLimits(cdpr_p,z_lim_inf);
xyz_mean = (limits(1:3,1)+limits(1:3,2))./2;
dl = (limits(:,2)-limits(:,1))/24*7;
for z = xyz_mean(3,1)-dl(3):wp_step:xyz_mean(3,1)+dl(3)
    for x = xyz_mean(1,1):wp_step:xyz_mean(1,1)+dl(1)
        pose = zeros(cdpr_p.pose_dim,1);
        for y = xyz_mean(2,1):wp_step:xyz_mean(2,1)+dl(2)
            pose(1:3) = [x;y;z];
            [out,pose] = SolveUnderActGeoStatWS(cdpr_p,cdpr_v,ut,tau_lim,out,pose);  
        end
        pose = zeros(cdpr_p.pose_dim,1);
        for y = xyz_mean(2,1)-wp_step:-wp_step:xyz_mean(2,1)-dl(2)
             pose(1:3) = [x;y;z];
            [out,pose] = SolveUnderActGeoStatWS(cdpr_p,cdpr_v,ut,tau_lim,out,pose); 
        end
    end
    for x =  xyz_mean(1,1)-wp_step:-wp_step:xyz_mean(1,1)-dl(1)
       pose = zeros(cdpr_p.pose_dim,1);
        for y= xyz_mean(2,1):wp_step:xyz_mean(2,1)+dl(2)
            pose(1:3) = [x;y;z];
            [out,pose] = SolveUnderActGeoStatWS(cdpr_p,cdpr_v,ut,tau_lim,out,pose);  
        end
        pose = zeros(cdpr_p.pose_dim,1);
        for y = xyz_mean(2,1)-wp_step:-wp_step:xyz_mean(2,1)-dl(2)
             pose(1:3) = [x;y;z];
            [out,pose] = SolveUnderActGeoStatWS(cdpr_p,cdpr_v,ut,tau_lim,out,pose); 
        end
    end
end

out.limits = limits;
out.workspace_center = mean(out.pose(1:3,:),2);
out.parametrization = cdpr_p.rotation_parametrization;

end