function mat = ComputeIdentifMatrix(p,v,a,cdpr_p)

cdpr_variables = CdprVar(cdpr_p.n_cables);
cdpr_variables = UpdateIKZeroOrd(p(1:3),p(4:end),cdpr_p,cdpr_variables);
cdpr_variables = UpdateIKFirstOrd(v(1:3),v(4:end),cdpr_p,cdpr_variables);
cdpr_variables = UpdateIKSecondOrd(a(1:3),a(4:end),cdpr_p,cdpr_variables);
        
At = [(cdpr_variables.platform.acceleration-cdpr_p.platform.gravity_acceleration) (Anti(cdpr_variables.platform.angular_acc)+Anti(cdpr_variables.platform.angular_vel)*Anti(cdpr_variables.platform.angular_vel))*cdpr_variables.platform.rot_mat zeros(3,6)];
Ar = [zeros(3,1) -Anti(cdpr_variables.platform.acceleration-cdpr_p.platform.gravity_acceleration)*cdpr_variables.platform.rot_mat (cdpr_variables.platform.rot_mat*LambdaFun(cdpr_variables.platform.rot_mat'*cdpr_variables.platform.angular_acc)+Anti(cdpr_variables.platform.angular_vel)*cdpr_variables.platform.rot_mat*LambdaFun(cdpr_variables.platform.rot_mat'*cdpr_variables.platform.angular_vel))];
    
Ja = cdpr_variables.geometric_jacobian(:,1:3);
Ju = cdpr_variables.geometric_jacobian(:,4:6);
mat = Ar-Ju'*inv(Ja')*At;
end