function [pose,out] = Solve_GS_WS(cdpr_p,cdpr_v,ut,pose,tau_lim,ws_info,out,varargin)

ws_lim = ws_info.limits;
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,pose);
[cdpr_v.underactuated_platform.pose_P(cdpr_p.n_cables+1:end),fval] = fsolve(@(v) FunGsNoCheck(cdpr_p,cdpr_v.underactuated_platform.pose_P(1:cdpr_p.n_cables),v),...
    cdpr_v.underactuated_platform.pose_P(cdpr_p.n_cables+1:end),ut.fsolve_options_grad);

if norm(fval)<0.001
    pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);
    if (all(pose(1:3)-ws_lim(1:3,1)>0) && all(ws_lim(1:3,2)-pose(1:3)>0))
      out = Calc_ws_flag_u(cdpr_p,cdpr_v,pose,tau_lim,ws_info.l_error,out,varargin);
    end
end

end