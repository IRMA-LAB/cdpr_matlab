function limits = CalcWorkspaceGraphicalLimits(cdpr_p,ws_data)

lim_sup = [Inf;Inf;Inf];
lim_inf = -[Inf;Inf;Inf];

cdpr_v = CdprVar(cdpr_p.n_cables);
for k = 1: length(ws_data)
    cdpr_v = UpdateIKZeroOrd(ws_data.pose(1:3,k),ws_data.pose(4:end,k),cdpr_p,cdpr_v);
    for i=1:cdpr_p.n_cables
        for l = 1:3
            switch l
                case 1
                    point = cdpr_p.cable(i).pos_D_glob;
                case 2
                    point = cdpr_v.cable(i).pos_OA_glob;
                case 3
                    point = cdpr_v.platform.position;
            end
            if (i==1 && l==1)
                lim_sup = point;
                lim_inf = lim_sup;
                
            else
                for j=1:3
                    if (point(j)>lim_sup(j))
                        lim_sup(j) = point(j);
                    elseif (point(j)<lim_inf(j))
                        lim_inf(j) = point(j);
                    end
                end
            end
            
            
        end
    end
end

dlim = lim_sup-lim_inf;
lim_sup = lim_sup+dlim/10;
lim_inf = lim_inf-dlim/10;
limits = [lim_inf lim_sup];