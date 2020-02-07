function out = DetermineWsUnderTranslational(cdpr_p,cdpr_v,ut,tau_lim,lim,mesh_elements,out,varargin)

if (~isempty(varargin))
    rec = varargin{1};
end

delta_z_safe = 0.5;
ws_limits = [lim.xyz_mean-lim.dl+lim.dl2 lim.xyz_mean+lim.dl-lim.dl2];
ws_limits(3,2) = ws_limits(3,2)-delta_z_safe;
ws_iterators = zeros(3,mesh_elements);

for i=1:3
    ws_iterators(i,:) = linspace(ws_limits(i,1),ws_limits(i,2),mesh_elements);
end

X = cell(1,3);
for i=1:3
    X{i} = ws_iterators(i,:)';
end
config = Allcomb(X{:})';
dc = zeros(1,cdpr_p.n_cables);
for i = 1:length(config)
    ig_v = zeros(3,1);
    for j=1:cdpr_p.n_cables
        ig_v = ig_v+cdpr_p.cable(j).pos_OD_glob/cdpr_p.n_cables;
        dc(j) = norm(abs(cdpr_p.cable(j).pos_OD_glob(1:2)-config(1:2,i)));
    end
    [~,ind_min] = min(dc);
    ig_v = ig_v-config(1:3,i);
    ig_v = ig_v/norm(ig_v);
    ig = zeros(2,1);
    ig(2) = acos(ig_v(3));
    if (abs(ig(2))>0.001)
        ig(1) = atan2(ig_v(2),ig_v(1));
    else
        ig(1) = 0;
    end
    cp = cdpr_p.cable(ind_min).pos_OD_glob(1:2)-config(1:2,i);
    ca = cdpr_p.cable(ind_min).pos_OD_glob(1:2)-(config(1:2,i)+cdpr_p.cable(ind_min).pos_PA_loc(1:2));
    tors = acos(ca'*cp/(norm(cp)*norm(ca)));
    rz = RotZ(tors);rz(:,3) = [];rz(3,:) = [];
    ca2 = cdpr_p.cable(ind_min).pos_OD_glob(1:2)-(config(1:2,i)+rz*cdpr_p.cable(ind_min).pos_PA_loc(1:2));
    if acos(ca2'*cp/(norm(cp)*norm(ca2)))>tors
        tors = -tors;
    end
    pose = [config(1:3,i);ig;tors];
    try
        [out,~] = SolveUnderActGeoStatWS(cdpr_p,cdpr_v,ut,tau_lim,out,pose);
    catch
        warning('Error in SolveUnderActGeoStatWS');
    end
end

end