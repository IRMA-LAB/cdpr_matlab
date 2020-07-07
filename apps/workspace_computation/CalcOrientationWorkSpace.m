function out = CalcOrientationWorkSpace(cdpr_p,cdpr_v,tau_lim,position,varargin)

ws_angle_step = pi/90;
out.counter = 0;

if (~isempty(varargin))
rec = varargin{1};
end



tors = 0;
for tilt = 0:ws_angle_step:pi/2-pi/18
    for ang = 0:ws_angle_step:2*pi-ws_angle_step
        ang_var = [ang;tilt;tors];
        pose = [position;ang_var];
        [out] = CheckPoseInOrientWorkSpace(cdpr_p,cdpr_v,tau_lim,out,pose);     
    end
end

out.limits = CalcWorkspaceGraphicalLimits(cdpr_p,out);
out.workspace_center = mean(out.limits,2);
out.parametrization = cdpr_p.rotation_parametrization;

end