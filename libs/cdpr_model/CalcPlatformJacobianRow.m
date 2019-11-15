function [geometric,analitic] = CalcPlatformJacobianRow(vers_t,pos_PA_glob,H_mat)

  geometric = [vers_t' -vers_t'*Anti(pos_PA_glob)];
  analitic = geometric;
  analitic(1,4:end) = analitic(1,4:end)*H_mat;
  
  
end