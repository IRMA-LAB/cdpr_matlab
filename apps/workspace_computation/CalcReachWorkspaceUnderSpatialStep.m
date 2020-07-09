function out = CalcReachWorkspaceUnderSpatialStep(cdpr_p,cdpr_v,ut,ws_info,rec,out,pose)

      counter = out.counter;
      p = pose(1:3);
      an = pose(4:end);
      if (cdpr_p.n_cables-3==0)
        if (out.counter == counter)
          cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,p,an);
        else
          cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars...
            (cdpr_p.underactuated_platform,0,out.pose(:,out.counter));
          an(1:end) = cdpr_v.underactuated_platform.pose_P(4:end);
          cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,p,an);
        end
        pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);
        try
          [out,~] = SolveUnderActGeoStatWS(cdpr_p,cdpr_v,ut,ws_info.tension_limits,out,pose);
        catch
          warning('Error in SolveUnderActGeoStatWS');
        end
      elseif (cdpr_p.n_cables-3==1)
        for ang_par = -pi/3:pi/18:pi/3
          if (out.counter == counter)
            cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,[p;ang_par],an(2:end));
          else
            cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars...
              (cdpr_p.underactuated_platform,0,out.pose(:,out.counter));
            an(2:end) = cdpr_v.underactuated_platform.pose_P(5:end);
            cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,[p;ang_par],an(2:end));
          end
          pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);
          try
            [out,~] = SolveUnderActGeoStatWS(cdpr_p,cdpr_v,ut,ws_info.tension_limits,out,pose,rec);
          catch
            warning('Error in SolveUnderActGeoStatWS');
          end
        end
        
      else
        for ang_par1 = -pi/3:pi/18:pi/3
          for ang_par2 = -pi/3:pi/18:pi/3
            if (out.counter == counter)
              cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,[p;ang_par1;ang_par2],an(3:end));
            else
              cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars...
                (cdpr_p.underactuated_platform,0,out.pose(:,out.counter));
              an(3:end) = cdpr_v.underactuated_platform.pose_P(6:end);
              cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.SetVars(0,[p;ang_par1;ang_par2],an(3:end));
            end
            pose = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);
            try
              [out,~] = SolveUnderActGeoStatWS(cdpr_p,cdpr_v,ut,ws_info.tension_limits,out,pose);
            catch
              warning('Error in SolveUnderActGeoStatWS');
            end
          end
        end
      end
      
end