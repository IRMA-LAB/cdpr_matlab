function out = ReorderResults(cdpr_p,tau_lim,out)

out.limits = CalcWorkspaceGraphicalLimits(cdpr_p,out);
out.tension_vector_limits = tau_lim;
out.workspace_center = mean(out.pose(1:3,:),2);
out.parametrization = cdpr_p.platform.rotation_parametrization;

end