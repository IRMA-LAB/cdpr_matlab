function out = CalcWorkspace(cdpr_p,cdpr_v,ut,out,type,tau_limits,folder,rec,n,varargin)

n_arg_opt = length(varargin);

if (n_arg_opt>0)
      add_wp_info =  varargin{1};
      ws_type = varargin{2};
end

if (type == 1) % translational
  
  if (cdpr_p.n_cables <6) % underactuated
      
      out = CalcWorkspaceUnder(cdpr_p,cdpr_v,ut,tau_limits,n,add_wp_info,rec,ws_type);
      rec = rec.ResetFigureLimits(out.limits,10);
      plot_handle = DisplayAndSaveWorkspace(out,0,folder,rec);
      delete(plot_handle);
      plot_handle = DisplayAndSaveWorkspace(out,1,folder,rec);
      delete(plot_handle);
    
  elseif (cdpr_p.n_cables >6) % overactuated
    
  else % completely actuated
  
  end
  
else  % orientational
  
  if (cdpr_p.n_cables <6) % underactuated

  elseif (cdpr_p.n_cables >6) % overactuated
    
  else % completely actuated
       out = CalcOrientationWorkSpace(cdpr_p,cdpr_v,tau_limits,add_wp_info);
       DisplayAndSaveWorkspace(out,2,folder,rec);
  end
   
end

end