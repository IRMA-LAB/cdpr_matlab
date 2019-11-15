function out = CalcWorkspace(cdpr_p,cdpr_v,out,ut,type,set_point,rec)
close all  
opt = optimoptions('linprog');
opt.Display = 'off';
if (type == 1) % translational
  
  cfr_arc_min = 0.02;
  z_step = 0.02;
  r_step = 0.02;
  counter = 0;
  if (cdpr_p.n_cables <6) % underactuated
  gStat_mask = [1;1;1;0;0;0];
  start_idx = 1;
  hold on
  for z = 0.1:-z_step:-1.5
    in_guess = [0;0;0];
    for r = 0:r_step:1.5
      cfr = 2*pi*r;
      steps = ceil(cfr./cfr_arc_min);
      for ang =0:2*pi/steps:2*pi*(1-1./steps)
        
        p = cdpr_p.workspace_center+[r*cos(ang);r*sin(ang);z];
        %tic
        or = fsolve(@(v) CalcWPGeometricStatic(cdpr_p,p,v,gStat_mask),...
        in_guess,ut.fsolve_options_grad);
        %toc
        in_guess = or;
        pose = [p;or];
        cdpr_v = UpdateIKZeroOrd(pose(1:3,1),pose(4:end,1),cdpr_p,cdpr_v);
        cdpr_v = CalcExternalLoadsStateSpace(cdpr_v,cdpr_p,eye(3));
        cdpr_v = CalcCablesTensionStat(cdpr_v);
        check1 = ~any(isnan(cdpr_v.tension_vector./cdpr_p.platform.mass)); 
        if (check1)
          check2 = ~(any(cdpr_v.tension_vector./cdpr_p.platform.mass < 2) || any(cdpr_v.tension_vector./cdpr_p.platform.mass > 10));
          if (check2) 
            counter = counter + 1;
            out.pose_WP(:,counter) = pose;
            out.manip(1,counter) = sqrt(det(cdpr_v.geometric_jacobian'*cdpr_v.geometric_jacobia));
            out.netZ_tension_vector_WP(:,counter) = cdpr_v.tension_vector./cdpr_p.platform.mass;
          end
        end
      end
    end
    tMax = max(out.netZ_tension_vector_WP(:,start_idx:counter));
    %[ss,s_i] = sort(tMax);
    [ss,s_i] = sort(out.manip);
    xx = out.pose_WP(1,start_idx:counter);
    yy = out.pose_WP(2,start_idx:counter);
    zz = out.pose_WP(3,start_idx:counter);
    cmap = jet(length(xx));
    scatter3(xx(s_i),yy(s_i),zz(s_i),10,cmap,'filled')
    start_idx = counter;
  end
  hold off
  
    
  elseif (cdpr_p.n_cables >6) % overactuated
    
  else % completely actuated
  
  end
  
else  % orientational
  angle_step = pi/90;
  counter = 0;
  if (cdpr_p.n_cables <6) % underactuated
  
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