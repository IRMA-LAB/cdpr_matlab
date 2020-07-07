function data = GetDestinations(data,ws_par,cdpr_p,cdpr_v,record,ut,gFun)

data.pNumber = 2;
mode_mask = [1 2 3];

ISorder = 1;

p(1:3,1)  = [0.86;1.18;-0.2];
data.info(1) = ExtractWsInfo(cdpr_p,ws_par,p(1:3,1));
data.p(:,1) = data.info(1).pose;


p(1:3,2)  = [-0.82;1.18;-1.4];
data.info(2) = ExtractWsInfo(cdpr_p,ws_par,p(1:3,2));
data.p(:,2) = data.info(2).pose;

f = ExtractNaturalFrequenciesOnPath(data.p,1,gFun,cdpr_p,cdpr_v,ut,mode_mask);
[data.shaper(:,:,1),data.shaperL(1)] = ComputeMultiModeShaper(f,1,ISorder,ut);
data = ComputeTrajectoryLimsAndT(data,f,1);

% data.p(1:3,3)  = ws_par.workspace_center+[0.1;0;0.1];
% data.info(3) = ExtractWsInfo(cdpr_p,ws_par,data.p(1:3,3));
% data.p(:,3) = data.info(3).pose;
% 
% f = ExtractNaturalFrequenciesOnPath(data.p,2,gFun,cdpr_p,cdpr_v,ut,mode_mask);
% [data.shaper(:,:,2),data.shaperL(2)] = ComputeMultiModeShaper(f,2,ISorder,ut);
% data = ComputeTrajectoryLimsAndT(data,f,2);
% 
% 
% data.info(4) = data.info(1);
% data.p(:,4) = data.info(4).pose;
% f = ExtractNaturalFrequenciesOnPath(data.p,3,gFun,cdpr_p,cdpr_v,ut,mode_mask);
% [data.shaper(:,:,3),data.shaperL(3)] = ComputeMultiModeShaper(f,3,ISorder,ut);
% data = ComputeTrajectoryLimsAndT(data,f,3);

% p(1:3,1)  = ws_par.workspace_center+[0;-0.5;-0.3];
% data.info(1) = ExtractWsInfo(cdpr_p,ws_par,p(1:3,1));
% data.p(:,1) = data.info(1).pose;
% 
% data.p(1:3,2)  = ws_par.workspace_center+[0;0.5;0.3];
% data.info(2) = ExtractWsInfo(cdpr_p,ws_par,data.p(1:3,2));
% data.p(:,2) = data.info(2).pose;
% 
% f = ExtractNaturalFrequenciesOnPath(data.p,1,gFun,cdpr_p,cdpr_v,ut,mode_mask);
% [data.shaper(:,:,1),data.shaperL(1)] = ComputeMultiModeShaper(f,1,ISorder,ut);
% data = ComputeTrajectoryLimsAndT(data,f,1);


  
end