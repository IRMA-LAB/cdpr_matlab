function matrix = ComputeIdentificationMatrix(cdpr_p,cdpr_v)


W11 = cdpr_v.platform.acceleration-cdpr_p.platform.gravity_acceleration;
W12 = (Anti(cdpr_v.platform.angular_acc)+Anti(cdpr_v.platform.angular_vel)...
    *Anti(cdpr_v.platform.angular_vel))*cdpr_v.platform.rot_mat;
W13 = zeros(3,6);
W21 = zeros(3,1);
W22 = -Anti(W11)*cdpr_v.platform.rot_mat;
W23 = cdpr_v.platform.rot_mat*OverSigned(cdpr_v.platform.rot_mat'*cdpr_v.platform.angular_acc)...
    +Anti(cdpr_v.platform.angular_vel)*cdpr_v.platform.rot_mat*OverSigned(cdpr_v.platform.rot_mat'*cdpr_v.platform.angular_vel);

matrix = [W11 W12 W13;
          W21 W22 W23];

end