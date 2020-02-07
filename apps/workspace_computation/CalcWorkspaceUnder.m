function out = CalcWorkspaceUnder(cdpr_p,cdpr_v,ut,tau_lim,n,z_lim_inf,varargin)

mesh_elements = n;
out.counter = 0;
ws_type = [];

if (~isempty(varargin))
    rec = varargin{1};
    ws_type = varargin{2};
end

lim = DetermineLimits(cdpr_p,z_lim_inf);

switch ws_type
    case WorkspaceType.TRANSLATIONAL
        out = DetermineWsUnderTranslational(cdpr_p,cdpr_v,ut,tau_lim,lim,mesh_elements,out);
        out = ReorderResults(cdpr_p,tau_lim,out);
    case WorkspaceType.ZERO_TORSION
        out = DetermineWsUnderZeroTorsion(cdpr_p,cdpr_v,ut,tau_lim,lim,mesh_elements,out);
        out = ReorderResults(cdpr_p,tau_lim,out);
    case WorkspaceType.MAX_MIN_TENSION
        out = DetermineWsUnderMaxMinT(cdpr_p,cdpr_v,ut,tau_lim,lim,mesh_elements,out);
        out = ReorderResults(cdpr_p,tau_lim,out);
    otherwise  
        warning('Workspace Type not supported or not specified');
end

end