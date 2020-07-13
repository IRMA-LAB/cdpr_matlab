classdef CableParameters
%CABLEPARAMETERS is a class containing the parameters of each cable.  
%
%   CABLEPARAMETERS links the geometrical parameter values of each cable 
%   and its corresponding swivel pulley from an input structure to the
%   homonymous object's properties. 
%     
  properties
    pos_PA_loc;%is a vector (size[3,1], [m]), containing the components of the distal anchor point A, projected in the local frame.
    pos_OD_glob;%is a vector (size[3,1], [m]), containing the components of the proximal anchor point D, projected in the fixed frame.
    vers_i;%
    vers_j;%
    vers_k;%
    rot_mat;%
    swivel_pulley_r;% is the swivel pulley radius.
    swivel_pulley_transmission_ratio;% is the swivel pulley-encoder transmission ratio. 
    l0;% is the initial cable length
    motor_transmission_ratio;% is the motor-cable transmission ratio.
    id
  end
  methods
    function obj = CableParameters(cable_parameters_struct)
%   CABLEPARAMETERS instantiates an object of CableParameters type.
%   CABLE_PARAMETERS_STRUCT is a structure containing the cable
%   parameters, arranged in different fields. 
      obj.pos_PA_loc = cable_parameters_struct.winch.pos_PA_loc;
      obj.pos_OD_glob = cable_parameters_struct.pulley.pos_OD_glob;
      obj.vers_i = cable_parameters_struct.pulley.vers_i;
      obj.vers_j = cable_parameters_struct.pulley.vers_j;
      obj.vers_k = cable_parameters_struct.pulley.vers_k;
      obj.vers_k = obj.vers_k./norm(obj.vers_k);
      obj.vers_i = cross(obj.vers_j,obj.vers_k);
      obj.vers_i = obj.vers_i./norm(obj.vers_i);
      obj.vers_j = cross(obj.vers_k,obj.vers_i);
      obj.vers_j = obj.vers_j./norm(obj.vers_j);
      obj.id = cable_parameters_struct.id;
      obj.swivel_pulley_r =  cable_parameters_struct.pulley.radius;
      obj.swivel_pulley_transmission_ratio = cable_parameters_struct.pulley.transmission_ratio;
      obj.l0 = cable_parameters_struct.winch.l0;
      obj.motor_transmission_ratio = cable_parameters_struct.winch.transmission_ratio;
      obj.rot_mat = [obj.vers_i obj.vers_j obj.vers_k];
    end
  end
end