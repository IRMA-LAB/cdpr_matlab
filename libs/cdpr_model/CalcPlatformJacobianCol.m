function [geometric,analitic] = CalcPlatformJacobianCol(vers_t,pos_PA_glob,H_mat)

  geometric = [vers_t; Anti(pos_PA_glob)*vers_t];
  analitic = geometric;
  analitic(4:end,1) = H_mat'*analitic(4:end,1);
  
end