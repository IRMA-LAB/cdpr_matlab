function data = GetDestinations(data,ws_par,cdpr_p,record,ut,gFun)

    data.pNumber = 4;
    v_mean = 1;
    
    p(1:3,1)  = ws_par.workspace_center-[0.4;0;0.5];
    data.info(1) = ExtractWsInfo(cdpr_p,ws_par,p(1:3,1));
    data.p(:,1) = data.info(1).pose;
    
    data.p(1:3,2)  = ws_par.workspace_center+[0.6;0;0.-0.5];
    data.info(2) = ExtractWsInfo(cdpr_p,ws_par,data.p(1:3,2));
    data.p(:,2) = data.info(2).pose;

    ISorder = 2;
    [data.shaper(:,:,1),data.shaperL(1)] = ComputeTwoModeShaper(data,1,ISorder);
    data.dt(1) = norm(data.p(1:3,1)-data.p(1:3,2))/v_mean;
    
     data.p(1:3,3)  = ws_par.workspace_center+[0.1;0;0.1];
     data.info(3) = ExtractWsInfo(cdpr_p,ws_par,data.p(1:3,3));
     data.p(:,3) = data.info(3).pose;
     
     ISorder = 2;
     [data.shaper(:,:,2),data.shaperL(2)] = ComputeTwoModeShaper(data,2,ISorder);
     data.dt(2) = norm(data.p(1:3,2)-data.p(1:3,3))/v_mean;
     
     data.info(4) = data.info(1);
     data.p(:,4) = data.info(4).pose;
     
     ISorder = 2;
     [data.shaper(:,:,3),data.shaperL(3)] = ComputeTwoModeShaper(data,3,ISorder);
     data.dt(3) = norm(data.p(1:3,4)-data.p(1:3,3))/v_mean;

  
end