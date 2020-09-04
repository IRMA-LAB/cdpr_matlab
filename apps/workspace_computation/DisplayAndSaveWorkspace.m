function f = DisplayAndSaveWorkspace(cdpr_p,cdpr_v,out,ws_info,folder,rec)

filename = strcat(folder,'/workspace_files/',rec.figure_handle.FileName);
flag_vis = 1;
switch ws_info.display_criteria
  case DisplayCriteria.NONE
    name = strcat(filename,'_no_criteria');
  case DisplayCriteria.POSITION_SENSITIVITY
    for i=1:out.counter
      cdpr_v = UpdateIKZeroOrd(out.position(:,i),out.ang_par(:,i),cdpr_p,cdpr_v);
      Kr = cdpr_v.geometric_jacobian(4:6,:)';
      Kp = cdpr_v.geometric_jacobian(1:3,:)';
      pr =  Kr*MyInv(Kr'*Kr)*Kr';
      Pr = eye(length(pr))-pr;
      out.manipP(1,i) = sqrt(norm(MyInv(Kp'*Pr*Kp),2));
    end
    criteria = out.manipP;
    [~,criteria_ind] = sort(criteria);
    name = strcat(filename,'_pos_sens');    
  case DisplayCriteria.ORIENTATION_SENSITIVITY
    for i=1:out.counter
      cdpr_v = UpdateIKZeroOrd(out.position(:,i),out.ang_par(:,i),cdpr_p,cdpr_v);
      Kr = cdpr_v.geometric_jacobian(4:6,:)';
      Kp = cdpr_v.geometric_jacobian(1:3,:)';
      pp = Kp*MyInv(Kp'*Kp)*Kp';
      Pp = eye(length(pp))-pp;
      out.manipR(1,i) = sqrt(norm(MyInv(Kr'*Pp*Kr),2));
    end
    criteria = out.manipR;
    [~,criteria_ind] = sort(criteria);
    name = strcat(filename,'_orient_sens');   
  case DisplayCriteria.TENSION_SENSITIVITY   
    criteria = out.WS_perf;
    [~,criteria_ind] = sort(criteria);
    name = strcat(filename,'_tension_sens'); 
  case DisplayCriteria.MANIPULABILITY    
  case DisplayCriteria.MAX_TENSION
    criteria = max(out.tension_vector);
    [~,criteria_ind] = sort(criteria);
    name = strcat(filename,'_max_tension');    
  case DisplayCriteria.MIN_TENSION
    criteria = min(out.tension_vector);
    [~,criteria_ind] = sort(criteria);
    name  = strcat(filename,'_min_tension');    
  otherwise
    disp('No workspace display method selected')
    flag_vis = 0;
    
end

if (flag_vis)
  switch ws_info.workspace_type
    case WorkspaceTypes.TRANSLATIONAL
      name  = strcat(name,',eps = [',num2str(out.pose(4,1)),';',num2str(out.pose(5,1)),';',num2str(out.pose(6,1)),']m');
      x = out.pose(1,:);
      y = out.pose(2,:);
      z = out.pose(3,:);
      if (ws_info.display_criteria == DisplayCriteria.NONE)
        f = scatter3(x,y,z,20,'k','filled');
      else
        cmap = jet(length(criteria));
        colorbar;
        colormap(cmap)
        caxis([min(criteria) max(criteria)])
        f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
      end
    case WorkspaceTypes.ORIENTATION
      close all
      name  = strcat(name,',p = [',num2str(out.pose(1,1)),';',num2str(out.pose(2,1)),';',num2str(out.pose(3,1)),']m');
      pax = polaraxes;
      polaraxes(pax);
      th = out.pose(4,:);
      r = out.pose(5,:)*180/pi;
      cmap = parula(length(criteria));
      polarscatter(th(criteria_ind),r(criteria_ind),10,cmap,'filled')
      colorbar;
      colormap(cmap)
      caxis([min(criteria) max(criteria)])
      pax.FontWeight = 'bold';
      pax.RTick = [20 40 60 80];
      pax.Layer = 'top';
      pax.GridAlpha = 1;
      pax.RAxisLocation = 10;
    case WorkspaceTypes.REACHABLE
      x = out.pose(1,:);
      y = out.pose(2,:);
      z = out.pose(3,:);
      if (ws_info.display_criteria == DisplayCriteria.NONE)
        f = scatter3(x,y,z,20,'k','filled');
      else
        cmap = jet(length(criteria));
        colorbar;
        colormap(cmap)
        caxis([min(criteria) max(criteria)])
        f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
      end
    case WorkspaceTypes.TOTAL_ORIENTATION
      name  = strcat(name,',eps_i = [',num2str(ws_info.lim(1,1)),';',num2str(ws_info.lim(2,1)),';',num2str(ws_info.lim(3,1)),']rad, '...
        ,',eps_s = [',num2str(ws_info.lim(1,2)),';',num2str(ws_info.lim(2,2)),';',num2str(ws_info.lim(3,2)),']rad');
      x = out.pose(1,:);
      y = out.pose(2,:);
      z = out.pose(3,:);
      if (ws_info.display_criteria == DisplayCriteria.NONE)
        f = scatter3(x,y,z,20,'k','filled');
      else
        cmap = jet(length(criteria));
        colorbar;
        colormap(cmap)
        caxis([min(criteria) max(criteria)])
        f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
      end
  end
  savefig(strcat(name,'.fig'));
end

ws_data = out;
ws_data = rmfield(ws_data,'pose');
save(strcat(filename,'_WS','.mat'),'ws_data');

end