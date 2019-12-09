classdef CdprParameter
  %CDPRPARAMETER is a class containing static parameters of the cdpr.
  %   CDPRPARAMETER decodes data from .json format to matlab objects.
  %   It needs a .json configuration file in order to define the set
  %   of static parameters as object's properties.
  %
  properties
    platform  % object containing the parameters of the platform.
    cable     % object containing the parameters of each cable and its corresponding swivel pulley.
    underactuated_platform % TBD
    rotation_parametrization % a string containing the name of the adopted method for rotation parameterization.
    n_cables  % the number of all cables.
    pose_dim  % the number of parameters used for platform pose parameterization.
    workspace_center  % a vector (size[3,1], [m]) containing the components of the workspace center, projected in the fixed frame.
  end

  methods
    function obj = CdprParameter(name)
      %   CDPRPARAMETER instantiates an object of CdprParameters type.
      %   LOCATION is a string containing the path where the file NAME is
      %   located.
      %   NAME is a string containing the name of a .json config file.
      json.startup;
      p =  json.read(name);
      workspace_center = zeros(3,1);
      i = 0;
      while (i<length(p.actuator))
        i = i + 1;
        if (p.actuator(i).active == 0)
          p.actuator(i) = [];
          i = i - 1;
        end
      end

      obj.n_cables = length(p.actuator);
      obj.rotation_parametrization = p.rotation_parametrization;
      if (obj.rotation_parametrization ==  RotationParametrizations.QUATERNION)
        obj.pose_dim = 7;
      else
        obj.pose_dim = 6;
      end
      obj.platform = PlatformParameters(p.platform);
      for i = 1:obj.n_cables
        cable(i, 1) = CableParameters(p.actuator(i));
        workspace_center = workspace_center + cable(i, 1).pos_OD_glob;
      end
      if (obj.n_cables < 6)
        obj.underactuated_platform = UnderActuatedPar(obj.n_cables,obj.pose_dim);
      end
      obj.workspace_center = workspace_center ./ 3;
      obj.workspace_center(3) = 0;
      obj.cable = cable;
    end

    function par = RotToPar(obj, mat)
      switch (obj.rotation_parametrization)
        case RotationParametrizations.EULER_ZYZ
          par = Rot2ZYZ(mat,zeros(3, 1));
        case RotationParametrizations.TAIT_BRYAN
          par = Rot2Tayt(mat,zeros(3, 1));
        case RotationParametrizations.RPY
          par = Rot2RPY(mat,zeros(3, 1));
        case RotationParametrizations.TILT_TORSION
          par = Rot2TT(mat,zeros(3, 1));
        case RotationParametrizations.QUATERNION
          par = Rot2Quat(mat);
      end
    end
  end
end
