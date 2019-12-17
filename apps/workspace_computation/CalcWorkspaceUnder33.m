function out = CalcWorkspaceUnder33(cdpr_p, cdpr_v, ut, tau_lim, z_lim_inf,...
  varargin)

ws_step = 0.05;
out.counter = 0;

if (~isempty(varargin))
  rec = varargin{1};
end

limits(:, 1) = [-Inf; -Inf; z_lim_inf];
limits(:, 2) = [Inf; Inf; Inf];

for i = 1:cdpr_p.n_cables
  point = cdpr_p.cable(i).pos_OD_glob;
  if i == 1
    limits(:, 2) = point;
    limits(1:2, 1) = point(1:2, 1);
  else
    for j = 1:3
      if (point(j) > limits(j,2))
        limits(j, 2) = point(j);
      elseif (point(j) < limits(j, 1))
        limits(j, 1) = point(j);
      end
    end
  end
end

xyz_mean = (limits(1:3, 1) + limits(1:3, 2)) ./ 2;
dl = (limits(:, 2) - limits(:, 1)) / 20 * 7;
for z = xyz_mean(3,1) - dl(3):ws_step:xyz_mean(3,1) + dl(3)
  for x = xyz_mean(1, 1):ws_step:xyz_mean(1, 1) + dl(1)
    pose = zeros(cdpr_p.pose_dim, 1);
    for y = xyz_mean(2, 1):ws_step:xyz_mean(2, 1)+dl(2)
      pose(1:3) = [x; y; z];
      [out, pose] = SolveUnderActGeoStatWS(cdpr_p, cdpr_v, ut, tau_lim, out,...
        pose);
    end
    pose = zeros(cdpr_p.pose_dim, 1);
    for y = xyz_mean(2, 1) - ws_step:-ws_step:xyz_mean(2, 1) - dl(2)
      pose(1:3) = [x; y; z];
      [out, pose] = SolveUnderActGeoStatWS(cdpr_p, cdpr_v, ut, tau_lim, out,...
        pose);
    end
  end
  for x =  xyz_mean(1, 1)-ws_step:-ws_step:xyz_mean(1, 1) - dl(1)
    pose = zeros(cdpr_p.pose_dim, 1);
    for y= xyz_mean(2, 1):ws_step:xyz_mean(2,1) + dl(2)
      pose(1:3) = [x; y; z];
      [out, pose] = SolveUnderActGeoStatWS(cdpr_p, cdpr_v, ut, tau_lim, out,...
        pose);
    end
    pose = zeros(cdpr_p.pose_dim, 1);
    for y = xyz_mean(2, 1) - ws_step:-ws_step:xyz_mean(2, 1) - dl(2)
      pose(1:3) = [x; y; z];
      [out, pose] = SolveUnderActGeoStatWS(cdpr_p, cdpr_v, ut, tau_lim, out,...
        pose);
    end
  end
end

out.limits = CalcWorkspaceGraphicalLimits(cdpr_p, out);
out.workspace_center = mean(out.pose(1:3, :), 2);
out.parametrization = cdpr_p.rotation_parametrization;
