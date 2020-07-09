function out = CalcOrientWorkspaceCompletelySpatial(cdpr_p,cdpr_v,ut,ws_info,rec)

ws_angle_step = pi/90;
out.counter = 0;

tors = 0;
ang1_step = pi/2/ws_info.mesh_divider;
ang2_step = 2*pi/ws_info.mesh_divider/4;
for tilt = 0:ang1_step:pi/2-ang1_step
    for ang = 0:ang2_step:2*pi-ang2_step
        ang_var = [ang;tilt;tors];
        pose = [ws_info.p;ang_var];
        out = CheckPoseInOrientWorkSpace(cdpr_p,cdpr_v,ws_info,out,pose,rec);     
    end
end

out.limits = CalcWorkspaceGraphicalLimits(cdpr_p,out);
out.workspace_center = mean(out.limits,2);
out.parametrization = cdpr_p.platform.rotation_parametrization;

end