function out = CheckPoseInOrientWorkSpace(cdpr_p,cdpr_v,ws_info,out,pose,rec)
 
cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v = CalcCablesStaticTension(cdpr_v);

if ((~any(isnan(cdpr_v.tension_vector))) && isempty(cdpr_v.tension_vector(cdpr_v.tension_vector>ws_info.tension_limits(2)))...
        && isempty(cdpr_v.tension_vector(cdpr_v.tension_vector<ws_info.tension_limits(1))))
    out.counter = out.counter+1;
    out.pose(:,out.counter) = pose;
    out.position(:,out.counter) = out.pose(1:3,out.counter);
    out.ang_par(:,out.counter) = out.pose(4:end,out.counter);
    out.rot_mat(:,:,out.counter) = cdpr_v.platform.rot_mat;
    out.tension_vector(:,out.counter) = cdpr_v.tension_vector;
end

end