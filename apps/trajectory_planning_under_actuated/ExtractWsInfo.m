function info = ExtractWsInfo(cdpr_p,ws_par,p)

    for i=1:ws_par.counter
       if(norm(p-ws_par.position(:,i))<0.1) 
           info.pose(1:3,1) = ws_par.position(:,i);
           info.pose(4:6,1) = cdpr_p.RotToPar(ws_par.rot_mat(:,:,i));
           info.nat_period = 1./ws_par.nat_freq(:,i);
           break 
       end
    end
end