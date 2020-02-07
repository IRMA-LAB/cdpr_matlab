function lim = DetermineLimits(cdpr_p,z_lim_inf)

limits(:,1) = [-Inf;-Inf;z_lim_inf];
limits(:,2) = [Inf;Inf;Inf];

for i=1:cdpr_p.n_cables
    point = cdpr_p.cable(i).pos_OD_glob;
    if i==1
        limits(:,2) = point;
        limits(1:2,1) = point(1:2,1);
    else
        for j=1:3
            if (point(j)>limits(j,2))
                limits(j,2) = point(j);
            elseif (point(j)<limits(j,1))
                limits(j,1) = point(j);
            end
        end
    end
end

limits2(:,1) = [-Inf;-Inf;-Inf];
limits2(:,2) = [Inf;Inf;Inf];

for i=1:cdpr_p.n_cables
    point = cdpr_p.cable(i).pos_PA_loc;
    if i==1
        limits2(:,2) = point;
        limits2(:,1) = point;
    else
        for j=1:3
            if (point(j)>limits2(j,2))
                limits2(j,2) = point(j);
            elseif (point(j)<limits2(j,1))
                limits2(j,1) = point(j);
            end
        end
    end
end

lim.xyz_mean = (limits(1:3,1)+limits(1:3,2))./2;
lim.dl = (limits(:,2)-limits(:,1))/2;
lim.dl2 = abs(limits2(:,2)-limits2(:,1));

end