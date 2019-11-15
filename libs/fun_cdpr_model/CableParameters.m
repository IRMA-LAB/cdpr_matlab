classdef CableParameters
%CABLEPARAMETERS is a class containing the parameters of each cable.  
%
%   CABLEPARAMETERS links the geometrical parameter values of each cable 
%   and its corresponding swivel pulley from an input structure to the
%   homonymous object's properties. 
%     
  properties
    pos_A_loc;%is a vector (size[3,1], [m]), containing the components of the distal anchor point A, projected in the local frame.
    pos_D_glob;%is a vector (size[3,1], [m]), containing the components of the proximal anchor point D, projected in the fixed frame.
    vers_i;%
    vers_j;%
    vers_k;%
    rot_mat;%
    swivel_pulley_r;% is the swivel pulley radius.
    swivel_pulley_encoder_res;% is the swivel pulley encoder resolution. 
    motor_cable_tau;% is the transmission ratio [m/counts]
    l0;% is the initial cable length
    motor_encoder_res;% is the motor encoder resolution.
  end
  methods
    function obj = CableParameters(cable_parameters_struct)
%   CABLEPARAMETERS instantiates an object of CableParameters type.
%   CABLE_PARAMETERS_STRUCT is a structure containing the cable
%   parameters, arranged in different fields. 
      obj.pos_A_loc = cable_parameters_struct.pos_A_loc;
      obj.pos_D_glob = cable_parameters_struct.pos_D_glob;
      obj.vers_i = cable_parameters_struct.vers_i;
      obj.vers_j = cable_parameters_struct.vers_j;
      obj.vers_k = cable_parameters_struct.vers_k;
      obj.swivel_pulley_r =  cable_parameters_struct.swivel_pulley_r;
      obj.swivel_pulley_encoder_res = cable_parameters_struct.swivel_pulley_encoder_res;
      obj.motor_cable_tau = cable_parameters_struct.motor_cable_tau;
      obj.l0 = cable_parameters_struct.l0;
      obj.motor_encoder_res = cable_parameters_struct.motor_encoder_res;
      obj.rot_mat = [obj.vers_i obj.vers_j obj.vers_k];
    end
  end
end