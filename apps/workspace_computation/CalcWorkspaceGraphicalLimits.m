function limits = CalcWorkspaceGraphicalLimits(cdpr_p,varargin)

lim_sup = [Inf;Inf;Inf];
lim_inf = -[Inf;Inf;Inf];
if (~isempty(varargin))
    lim_inf(3,1) = varargin{1};
end
    
for i=1:cdpr_p.n_cables
    if i==1    
        lim_sup = cdpr_p.cable(i).pos_D_glob;
        lim_inf(1:2,1) = cdpr_p.cable(i).pos_D_glob(1:2,1);
        if lim_inf(3,1)==-Inf
            lim_inf(3,1) = cdpr_p.cable(i).pos_D_glob(3,1);
        end
    else
        for j=1:3
           if (cdpr_p.cable(i).pos_D_glob(j)>lim_sup(i))
                lim_sup(j) = cdpr_p.cable(i).pos_D_glob(j);
            elseif (cdpr_p.cable(i).pos_D_glob(i)<lim_inf(j))
                lim_inf(j) = cdpr_p.cable(i).pos_D_glob(j);
            end 
        end
    end
end

dlim = lim_sup-lim_inf;
lim_sup = lim_sup+dlim/10;
lim_inf = lim_inf-dlim/10;
limits = [lim_inf lim_sup];