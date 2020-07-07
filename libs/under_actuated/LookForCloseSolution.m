function guess = LookForCloseSolution(cdpr_p,ws_par,p)

    guess = [];
    for i=1:ws_par.counter
       if(norm(p-ws_par.position(:,i))<0.1) 
           guess = cdpr_p.RotToPar(ws_par.rot_mat(:,:,i));
           break 
       end
    end
end