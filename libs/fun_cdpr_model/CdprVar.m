classdef CdprVar
%CDPRVAR is a class containing time dependent variables of the cdpr.

  properties
    platform = PlatformVar;% is an object containing time dependent variables of the platform.
    cable = CableVar;% is an object containing time dependent variables of a cable and its swivel pulley.
    underactuated_platform = UnderActuatedVar;
    
    geometric_jacobian;%
    analitic_jacobian;%
    
    geometric_jacobian_d;%
    analitic_jacobian_d;%
    
    tension_vector;%
    cable_vector;
    cable_speed;
    
  end
  methods
    function obj = CdprVar(n)
        % CDPRVAR instantiates an object of CableParameters type.
        % CDPRVAR defines the object CABLE containing an object for each
        % cable that stores time dependent variables of the cable and its
        % swivel pulley.
      for i=1:n
        obj.cable(i,1) = CableVar;
      end
    end
    function obj = UpdateJacobiansD(obj)
      for i=1:length(obj.cable)
        obj.geometric_jacobian_d(i,:) = obj.cable(i).geometric_jacobian_row_d;
        obj.analitic_jacobian_d(i,:) = obj.cable(i).analitic_jacobian_row_d;
        obj.cable_speed(i,1) = obj.cable(i).complete_speed;
      end  
    end
    function obj = UpdateJacobians(obj)
      for i=1:length(obj.cable)
        obj.geometric_jacobian(i,:) = obj.cable(i).geometric_jacobian_row;
        obj.analitic_jacobian(i,:) = obj.cable(i).analitic_jacobian_row;
        obj.cable_vector(i,1) = obj.cable(i).complete_length;
      end  
    end
  end
end