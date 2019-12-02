function [output] = StandardInverseSimulator(cdpr_p,cdpr_v,...
     sim_data,geom_fun,ut)

for i = 1:sim_data.pNumber-1
    
 sim_data.coeff(:,i) = zeros(6,1);
 out = GenerateOutputUnderActuated(i,cdpr_p,cdpr_v,sim_data,geom_fun,ut);
 output(i).t = out.t;
 output(i).platform = out.platform;
 output(i).cables(:) = out.cable(:);
 output(i).coefficients = sim_data.coeff(:,i);
%  for j=1:length(out.tension_vector)
%      cdpr_v = UpdateIKZeroOrd(output(i).platform.pose(1:3,j),output(i).platform.pose(4:6,j),cdpr_p,cdpr_v);
%      output(i).platform.twist(:,j) = [output(i).platform.pose_d(1:3,j);cdpr_v.platform.H_mat*output(i).platform.pose_d(4:6,j)]; 
%      twist_non_act = [cdpr_v.geometric_jacobian(:,4:6)'*inv(cdpr_v.geometric_jacobian(:,1:3)') -eye(3)]';
%      output(i).platform.twist_na(:,j) = zeros(6,1);
%      for k=1:3
%         magnitude(k) = output(i).platform.twist(:,j)'*twist_non_act(:,k)./(twist_non_act(:,k)'*twist_non_act(:,k));
%         output(i).platform.twist_na(:,j) = output(i).platform.twist_na(:,j) + magnitude(k).*twist_non_act(:,k);
%      end
%      output(i).platform.twist_a(:,j) = output(i).platform.twist(:,j)-output(i).platform.twist_na(:,j);
%      output(i).platform.verify(:,j) = cdpr_v.geometric_jacobian*output(i).platform.twist_a(:,j);
%  end
       
end

end