function out = DetermineWsUnderZeroTorsion(cdpr_p,cdpr_v,ut,tau_lim,lim,mesh_elements,out,varargin)

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

for i = 1:length(config)
    ig_v = zeros(3,1);
    for j=1:cdpr_p.n_cables
        ig_v = ig_v+cdpr_p.cable(j).pos_OD_glob/cdpr_p.n_cables;
    end
    ig_v = ig_v-config(1:3,i);
    ig_v = ig_v/norm(ig_v);
    ig = zeros(2,1);
    ig(2) = acos(ig_v(3));
    if (abs(ig(2))>0.001)
        ig(1) = atan2(ig_v(2),ig_v(1));
    else
        ig(1) = 0;
    end
    pose = [config(1:3,i);ig;0];
    try
        [out,~] = SolveUnderActGeoStatWS(cdpr_p,cdpr_v,ut,tau_lim,out,pose);
    catch
        warning('Error in SolveUnderActGeoStatWS');
    end
    
end

end