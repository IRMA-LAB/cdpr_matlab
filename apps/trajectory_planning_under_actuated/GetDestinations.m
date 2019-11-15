function data = GetDestinations(data,cdpr_p,record,ut)

    data.pNumber = 4;
    
    data.p(1:3,1)  = cdpr_p.workspace_center-[0.3;0;0.9];
    data.p(4:6,1)  = fsolve(@(v) FunGs(record,cdpr_p,data.p(1:3,1),v),...
    [0;0;0],ut.fsolve_options_grad);

    [vector1,matrix1] = CalcNaturalFrequencies(cdpr_p,data.p(:,1));
    
    data.dt(1) = 2.5; 
    
    data.p(1:3,2)  = cdpr_p.workspace_center+[0.5;0;-0.9];
    data.p(4:6,2)  = fsolve(@(v) FunGs(record,cdpr_p,data.p(1:3,2),v),...
    [0;0;0],ut.fsolve_options_grad);

    [vector2,matrix2] = CalcNaturalFrequencies(cdpr_p,data.p(:,2));
    
    data.dt(2) = 2; 
    
    data.p(1:3,3)  = cdpr_p.workspace_center+[0.1;0;0];
    data.p(4:6,3)  = fsolve(@(v) FunGs(record,cdpr_p,data.p(1:3,3),v),...
    [0;0;0],ut.fsolve_options_grad);
    
    [vector3,matrix3] = CalcNaturalFrequencies(cdpr_p,data.p(:,3));
    data.dt(3) = 2; 
    
     
    data.p(1:3,4)  = data.p(1:3,1);
    data.p(4:6,4)  = data.p(4:6,1);
  
end