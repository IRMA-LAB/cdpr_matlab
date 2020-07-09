function limits = CalcWorkspaceGraphicalLimits(cdpr_p,ws_data)

lim_sup = mean(ws_data.pose(1:3,:),2);
lim_inf = lim_sup;
cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);
for k = 1:ws_data.counter
    cdpr_v = UpdateIKZeroOrd(ws_data.pose(1:3,k),ws_data.pose(4:end,k),cdpr_p,cdpr_v);
    for i=1:cdpr_p.n_cables
        for l = 1:4
            switch l
                case 1
                    point = cdpr_p.cable(i).pos_OD_glob;
                case 2
                    point = cdpr_v.cable(i).pos_OA_glob;
                case 3
                    point = cdpr_v.platform.position;
                case 4
                    point = RecordType.GetIntersectionInPlane(cdpr_v.platform.position,cdpr_v.cable(i).pos_OA_glob,cdpr_v.platform.rot_mat(:,3));
            end
            
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

dlim = lim_sup-lim_inf;
lim_sup = lim_sup+dlim/10;
lim_inf = lim_inf-dlim/10;
limits = [lim_inf lim_sup];