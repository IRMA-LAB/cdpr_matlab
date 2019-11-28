classdef CableVar
  %CABLEVAR is a class containing time dependent variables of a cable and its swivel pulley.
  %   Let A be the distal anchor point;
  %       D be the proximal anchor point,i.e. where the cable enters the pulley's groove;
  %       B be the cable exit point from the pulley groove;
  %       C be the geometric center of the swivel pulley;
  %       O be the origin of the global frame;
  %       P be a platform point.
  
  properties
    complete_length % the total cable length between the points A and D.
    swivel_ang % the angle between VERS_U and the position vector (B-C).
    tan_ang % the tangent of SWIVEL_ANG.
    pos_PA_glob % a vector(size[3,1],[m]), containing the components of the position vector (A-P), projected on the global frame.
    pos_OA_glob % a vector(size[3,1],[m]), containing the components of the position vector (A-O), projected on the global frame.
    pos_DA_glob % a vector(size[3,1],[m]), containing the components of the position vector (A-D), projected on the global frame.
    pos_BA_glob % a vector(size[3,1],[m]), containing the components of the position vector (A-B), projected on the global frame.
    vers_u % a vector (size[3,1]), containing the components, projected on the global frame, of the unit vector directed from D to C.
    vers_n % a vector(size[3,1]), containing the components, projected on the global frame, of the versor equidirectional and equiverse to the position vector (B-C).
    vers_w % a vector (size[3,1]), containing the components, projected on the global frame, of the unit vector normal to the pulley plane at the point D.
    vers_t % a vector(size[3,1]), containing the components of the unit vector from B to A, projected on the global frame.
    geometric_jacobian_row %
    analitic_jacobian_row %
    % VERS_U, VERS_W and VERS_N define a right handed coordinate system.
    
    complete_speed % the 1th order time derivative of LENGTH.
    swivel_ang_vel % the 1th order time derivative of SWIVEL_ANG.
    tan_ang_vel % the 1th order time derivative of TAN_ANG.
    vel_OA_glob % a vector(size[3,1] [m/s]), containing the components of the 1th order time derivatives of POS_OA_GLOB.
    vel_BA_glob % a vector(size[3,1] [m/s]), containing the components of the 1th order time derivatives of POS_BA_GLOB.
    vers_u_deriv % a vector(size[3,1]), containing the components of the 1th order time derivatives of VERS_U.
    vers_w_deriv % a vector(size[3,1]), containing the components of the 1th order time derivatives of VERS_W.
    vers_n_deriv % a vector(size[3,1]), containing the components of the 1th order time derivatives of VERS_N.
    vers_t_deriv % a vector(size[3,1]), containing the components of the 1th order time derivatives of VERS_RHO.
    geometric_jacobian_row_d %
    analitic_jacobian_row_d %
    
    complete_acceleration % the 2nd order time derivative of LENGTH.
    swivel_ang_acc % the 2nd order time derivative of SWIVEL_ANG.
    tan_ang_acc % the 2nd order time derivative of TAN_ANG.
    acc_OA_glob % a vector(size[3,1] [m/s]), containing the components of the 2nd order time derivatives of POS_OA_GLOB.
  end
  
  methods
    function obj = UpdateKinematic(obj, l, s, t)
      %UPDATE updates cable transmission key variables variables.
      %
      %   L is the total cable length between the points A and D.
      %   S is the angle between VERS_U and the position vector (B-C).
      %   T is is the tangent of S.
      obj.length = l;
      obj.swivel_ang = s;
      obj.tan_ang = t;
    end
  end
end
