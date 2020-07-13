function [geometric,analitic] = CalcPlatformJacobianColDerivatives(vers_t,vers_t_d,pos_PA_glob,H_mat,H_mat_d,omega)

  geometric = [vers_t_d; Anti(pos_PA_glob)*(vers_t_d+Anti(omega)*vers_t)];
  analitic = geometric;
  analitic(4:end,1) = H_mat'*analitic(4:end,1)-H_mat_d'*Anti(pos_PA_glob)*vers_t;
  
  
end