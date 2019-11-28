function cable_v = UpdateCableFirstOrd(cable_p,platform_v,cable_v)
%UPDATECABLEFIRSTORD updates each variable connected to the 1th order kinematics problem of the cable. 
%
%   UPDATECABLEFIRSTORD updates each time dependent variable connected to
%   1th order kinematic of the cable and its swivel pulley storing them 
%   in a proper structure.
%
%   CABLE_P is a structure containing static parameters of the cable and 
%   its swivel pulley.
%   PLATFORM_v is a structure containing time dependent variables of the 
%   platform.
%   CABLE_V is a structure containing time dependent variables of the  
%   cable and its swivel pulley.

  cable_v = UpdateVelA(cable_v.pos_PA_glob, platform_v.velocity,...
    platform_v.angular_vel, cable_v);
  cable_v.swivel_ang_vel = CalcSwivelAngleD(cable_v.vers_u, cable_v.vers_w,...
    cable_v.vel_OA_glob, cable_v.pos_DA_glob);
  cable_v = CalcPulleyVersorsD(cable_v);
  cable_v.tan_ang_vel = CalcTangentAngleD(cable_v.vers_n,...
    cable_v.vel_OA_glob, cable_v.pos_BA_glob);
  cable_v = CalcCableVectorsD(cable_p,cable_v);
  cable_v.complete_speed = CalcCableSpeed(cable_v.vers_t, cable_v.vel_OA_glob);
  [cable_v.geometric_jacobian_row_d, cable_v.analitic_jacobian_row_d] = ...
    CalcPlatformJacobianRowDerivatives(cable_v.vers_t, cable_v.vers_t_deriv,...
    cable_v.pos_PA_glob, platform_v.H_mat, platform_v.H_mat_deriv, ...
    platform_v.angular_vel);
