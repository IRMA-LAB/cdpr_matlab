function data = GetPosesForStaticCalibration(data,cdpr_p,record,ut)

counter = 0;
r = 0.4;
dr = 0.2;
geometric_static_mask = [1;1;1;0;0;0];
  for k = 1:8
%   for k = 1:1
    center = cdpr_p.workspace_center-[0;0;0.7+(k-1)*0.1];
    counter = counter+1;
    data.p(1:3,counter) = center;
    data.p(4:6,counter) = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_p,record,data.p(1:3,counter),v,geometric_static_mask),...
    [0;0;pi],ut.fsolve_options);
    cdpr_v = CdprVar(cdpr_p.n_cables);
    cdpr_v = UpdateIKZeroOrd(data.p(1:3,counter),data.p(4:6,counter),cdpr_p,cdpr_v);
    data.l(1:3,counter) = [cdpr_v.cable(1).length;cdpr_v.cable(2).length;cdpr_v.cable(3).length];
    for j=1:6
      counter = counter+1;
      if (mod(j,2))
        data.p(1:3,counter) = center+(r+dr).*[cos((j-1)*pi/3);sin((j-1)*pi/3);0];
      else
        data.p(1:3,counter) = center+r.*[cos((j-1)*pi/3);sin((j-1)*pi/3);0];      
      end
      data.p(4:6,counter) = fsolve(@(v) CalcGenericGeometricStatic(...
        cdpr_p,record,data.p(1:3,counter),v,geometric_static_mask),...
        data.p(4:6,counter-1),ut.fsolve_options);
      cdpr_v = CdprVar(cdpr_p.n_cables);
      cdpr_v = UpdateIKZeroOrd(data.p(1:3,counter),data.p(4:6,counter),cdpr_p,cdpr_v);
      data.l(1:3,counter) = [cdpr_v.cable(1).length;cdpr_v.cable(2).length;cdpr_v.cable(3).length];
    end
  end
  for i=1:length(data.l)
    data.dcount(:,i) = (data.l(:,1)-data.l(:,i))/cdpr_p.cable(1, 1).motor_cable_tau;
  end
   
end