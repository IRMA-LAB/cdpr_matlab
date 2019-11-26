function [geometric,analitic] = CalcPlatformJacobianRowDerivatives(vers_t,vers_t_d,pos_PA_glob,H_mat,H_mat_d,omega)

  geometric = [vers_t_d' -(vers_t_d'+vers_t'*Anti(omega))*Anti(pos_PA_glob)];
  analitic = geometric;
  analitic(1,4:end) = analitic(1,4:end)*H_mat-vers_t'*Anti(pos_PA_glob)*H_mat_d;
  
  
end