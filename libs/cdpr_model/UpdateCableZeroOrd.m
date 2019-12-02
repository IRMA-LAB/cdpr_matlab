function cable_v = UpdateCableZeroOrd(cable_p, platform_v, cable_v)
%UPDATECABLEZEROORD updates each variable connected to the 0th order kinematics
% problem of the cable.
%
%   UPDATECABLEZEROORD updates each time dependent variable connected to
%   0th order kinematic of the cable and its swivel pulley storing them
%   in a proper structure.
%
%   CABLE_P is a structure containing static parameters of the cable and
%   its swivel pulley.
%   PLATFORM_v is a structure containing time dependent variables of the
%   platform.
%   CABLE_V is a structure containing time dependent variables of the
%   cable and its swivel pulley.

cable_v = UpdatePosA(cable_p.pos_PA_loc, cable_p.pos_OD_glob,...
  platform_v.position, platform_v.rot_mat, cable_v);
cable_v.swivel_ang = CalcSwivelAngle(cable_p.vers_i,...
  cable_p.vers_j, cable_v.pos_DA_glob);
cable_v = CalcPulleyVersors(cable_p.vers_i, cable_p.vers_j, cable_v);
cable_v.tan_ang = CalcTangentAngle(cable_p.vers_k,...
  cable_p.swivel_pulley_r, cable_v.vers_u, cable_v.pos_DA_glob);
cable_v = CalcCableVectors(cable_p.swivel_pulley_r, cable_p.vers_k, cable_v);
cable_v.complete_length = CalcCableLen(cable_p.swivel_pulley_r,...
  cable_v.tan_ang, cable_v.pos_BA_glob);

[cable_v.geometric_jacobian_row, cable_v.analitic_jacobian_row] = ...
  CalcPlatformJacobianRow(cable_v.vers_t, cable_v.pos_PA_glob,...
  platform_v.H_mat);
