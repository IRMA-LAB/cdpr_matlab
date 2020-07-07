function cable_v = UpdateCableSecondOrd(platform_v, cable_v)
%UPDATECABLESECONDORD updates each variable connected to the 2nd order
% kinematics problem of the cable.
%
%   UPDATECABLESECONDORD updates each time dependent variable connected to
%   2th order kinematic of the cable and its swivel pulley storing them
%   in a proper structure.
%
%   PLATFORM_v is a structure containing time dependent variables of the
%   platform.
%   CABLE_V is a structure containing time dependent variables of the
%   cable and its swivel pulley.

cable_v = UpdateAccA(cable_v.pos_PA_glob, platform_v.acceleration,...
  platform_v.angular_vel, platform_v.angular_acc, cable_v);
cable_v.swivel_ang_acc = CalcSwivelAngleD2(cable_v.vers_u, cable_v.vers_w,...
  cable_v.vers_w_deriv, cable_v.vel_OA_glob, cable_v.acc_OA_glob,...
  cable_v.pos_DA_glob);
cable_v.tan_ang_acc = CalcTangentAngleD2(cable_v.vers_n, cable_v.vers_n_deriv,...
  cable_v.vel_OA_glob, cable_v.acc_OA_glob, cable_v.pos_BA_glob,...
  cable_v.vel_BA_glob, cable_v.tan_ang_vel);
cable_v.complete_acceleration = CalcCableAcceleration(cable_v.vers_t,...
  cable_v.vers_t_deriv, cable_v.vel_OA_glob, cable_v.acc_OA_glob);
