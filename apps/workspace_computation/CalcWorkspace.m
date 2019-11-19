function out = CalcWorkspace(cdpr_p,cdpr_v,ut,type,tau_limits,folder,rec,varargin)

n_arg_opt = length(varargin);

if (n_arg_opt>0)
      add_wp_info =  varargin{1};
end

if (type == 1) % translational
  
  if (cdpr_p.n_cables <6) % underactuated
      
      out = CalcWorkspaceUnder33(cdpr_p,cdpr_v,ut,tau_limits,add_wp_info);
      rec = rec.ResetFigureLimits(out.limits,10);
      plot_handle = DisplayAndSaveWorkspace(out,0,folder,rec);
      delete(plot_handle);
      plot_handle = DisplayAndSaveWorkspace(out,1,folder,rec);
      delete(plot_handle);
    
  elseif (cdpr_p.n_cables >6) % overactuated
    
  else % completely actuated
  
  end
  
else  % orientational
  angle_step = pi/90;
  counter = 0;
  if (cdpr_p.n_cables <6) % underactuated
  
    tau_max = max(out.tension_vector);
    tau_min = min(out.tension_vector);
    [~,sort_max_ind] = sort(tau_max);
    [~,sort_min_ind] = sort(tau_min);
    x = out.pose(1,:);
    y = out.pose(2,:);
    z = out.pose(3,:);
    cmap = jet(length(tau_max));
    scatter3(rec.axes_handle,x(sort_max_ind),y(sort_max_ind),z(sort_max_ind),10,cmap,'filled')
    scatter3(x(sort_min_ind),y(sort_min_ind),z(sort_min_ind),10,cmap,'filled')

  elseif (cdpr_p.n_cables >6) % overactuated
    
  else % completely actuated
      
    start_idx = 1;
%     figure
%     pax = polaraxes;
%     polaraxes(pax);
%     hold on

    for tors = 0
            %for tors = -pi/3:angle_step:pi/3
%          for tilt = 0:angle_step:pi/2-pi/180*10
for tilt = 0:angle_step:pi/2
            %for tilt = 0:angle_step:pi/3
            for ang = 0:angle_step:2*pi
                ang_var = [ang;tilt;tors];
                cdpr_v = UpdateIKZeroOrd(set_point,ang_var,cdpr_p,cdpr_v);
                cdpr_v = CalcExternalLoadsStateSpace(cdpr_v,cdpr_p,eye(3));
                cdpr_v = CalcCablesTensionStat(cdpr_v);
                check1 = ~any(isnan(cdpr_v.tension_vector)); 
                if (check1)
                    check2 = ~(any(cdpr_v.tension_vector < 0.1) || any(cdpr_v.tension_vector > 10));
                    if (check2) 
                        %check3 = sqrt(det(cdpr_v.geometric_jacobian(:,4:6)'*cdpr_v.geometric_jacobian(:,4:6)));
%                         if (check3 >= 2.5e-4)
                        counter = counter + 1;
                        out.pose_WP(:,counter) = [set_point;ang_var];
                        %out.manip(1,counter) = sqrt(det(cdpr_v.geometric_jacobian(:,4:6)'*cdpr_v.geometric_jacobian(:,4:6)));
                        K = inv(cdpr_v.geometric_jacobian);
                        out.manip(1,counter) = norm(K(4:6,:),Inf);
%                         man1 = out.manip(1,counter);
%                         Kr = cdpr_v.geometric_jacobian(:,4:6);
%                         Kp = cdpr_v.geometric_jacobian(:,1:3);
%                         Pp = eye(6)-Kp*inv(Kp'*Kp)*Kp';
%                         man2 = sqrt(norm(inv(Kr'*Pp*Kr),2));
%                         L = [cdpr_v.geometric_jacobian' -cdpr_v.geometric_jacobian']';
%                         for ii = 1:3
%                             v = zeros(6,1); v(ii+3) = 1;
%                            [x,fval(ii)] =  linprog(-v,L,ones(length(L),1),[],[],[],[],opt);
%                            fval(ii) = -fval(ii);
%                         end
%                         out.manip(1,counter) = max(fval);
                        out.netZ_tension_vector_WP(:,counter) = cdpr_v.tension_vector;
%                         end
                    end
                end
            end
        end
        tMax = max(out.netZ_tension_vector_WP(:,start_idx:counter));
        tMin = min(out.netZ_tension_vector_WP(:,start_idx:counter));
        manMin = min(out.manip(:,start_idx:counter));
        manMax = max(out.manip(:,start_idx:counter));
        %[ss,s_i] = sort(tMax);
        [ss,s_i] = sort(out.manip);
        th = out.pose_WP(4,start_idx:counter);
        r = out.pose_WP(5,start_idx:counter);
        h = out.pose_WP(6,start_idx:counter);

%         cmap = jet(length(th));
%         polarscatter(th(s_i),r(s_i)*180/pi,10,cmap,'filled')
        start_idx = counter;
    end
%     h1=colorbar;
%     colormap(cmap)
%     caxis([manMin manMax])
%     pax.FontWeight = 'bold';
%     pax.RTick = [20 40 60 80];
%     pax.Layer = 'top';
%     pax.GridAlpha = 1;
%     pax.RAxisLocation = 10;
%     hold off
  
  end
   
end

end