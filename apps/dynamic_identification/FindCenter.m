function [l0,zeta0] = FindCenter(cdpr_p,cdpr_v,ws_par)

for i=1:ws_par.counter
    if(norm(ws_par.workspace_center-ws_par.position(:,i))<0.2)
        zeta0(1:3,1) = ws_par.position(:,i);
        zeta0(4:6,1) = cdpr_p.RotToPar(ws_par.rot_mat(:,:,i));
        cdpr_v = UpdateIKZeroOrd(zeta0(1:3,1),zeta0(4:6,1),cdpr_p,cdpr_v);
        l0 = cdpr_v.cable_vector;
        break
    end
end

end