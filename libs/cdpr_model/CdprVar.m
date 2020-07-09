classdef CdprVar
%CDPRVAR is a class containing time dependent variables of the cdpr.

  properties
    platform = PlatformVar;% is an object containing time dependent variables of the platform.
    cable = CableVar;% is an object containing time dependent variables of a cable and its swivel pulley.
    underactuated_platform = UnderActuatedVar;
    overactuated_platform = OverActuatedVar;
    geometric_jacobian;%
    analitic_jacobian;%
    
    geometric_jacobian_d;%
    analitic_jacobian_d;%
    
    D_mat;
    D_mat_d;
    
    tension_vector;%
    cable_vector;
    cable_speed;
    
  end
  methods
    function obj = CdprVar(n,m)
        % CDPRVAR instantiates an object of CableParameters type.
        % CDPRVAR defines the object CABLE containing an object for each
        % cable that stores time dependent variables of the cable and its
        % swivel pulley.
      for i=1:n
        obj.cable(i,1) = CableVar;
      end
      obj.platform = PlatformVar(m);
    end
    function obj = UpdateJacobiansD(obj)
      for i=1:length(obj.cable)
        obj.geometric_jacobian_d(:,i) = obj.cable(i).geometric_jacobian_col_d;
        obj.analitic_jacobian_d(:,i) = obj.cable(i).analitic_jacobian_col_d;
        obj.cable_speed(i,1) = obj.cable(i).complete_speed;
      end  
    end
    function obj = UpdateJacobians(obj)
      for i=1:length(obj.cable)
        obj.geometric_jacobian(:,i) = obj.cable(i).geometric_jacobian_col;
        obj.analitic_jacobian(:,i) = obj.cable(i).analitic_jacobian_col;
        obj.cable_vector(i,1) = obj.cable(i).complete_length;
      end  
    end
    function obj = UpdateDMatrix(obj)
        %UPDATEMASSMATRIX updates the state space mass matrix of the platform. 
        %
        %   PAR is a structure containing the inertial properties of the 
        %   platform.
        obj.D_mat = obj.platform.D_mat;
    end
    function obj = UpdateDMatrix_d(obj)
        %UPDATEMASSMATRIX updates the state space mass matrix of the platform. 
        %
        %   PAR is a structure containing the inertial properties of the 
        %   platform.
        obj.D_mat_d = obj.platform.D_mat_d;
    end
  end
end