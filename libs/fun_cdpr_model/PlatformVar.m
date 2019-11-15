classdef PlatformVar
%PLATFORMVAR is a class containing time dependent variables of the platform.  
%   
%   PLATFORMVAR updates inertial and 0th,1th and 2nd order kinematic time 
%   dependent variables and stores them as object's properties.
%     
  properties
      
    position;% is a vector (size[3,1], [m]), containing the components of the position vector of the platform point P, projected on the global frame.     
    rot_mat;%is the rotation matrix (size[3,3]).
    pos_PG_glob;%is a vector(size[3,1],[m]), containing the components of the position vector (G-P), projected on the global frame.
    pos_OG_glob;%is a vector(size[3,1],[m]), containing the components of the position vector (G-O), projected on the global frame.
    
    velocity;%is a vector (size[3,1], [m/s]), containing the components of the velocity vector of the platform point P, projected on the global frame.
    angular_vel;%is a vector (size[3,1], [rad/s]), containing the components of angular velocity, projected on the global frame.
    vel_OG_glob;%is a vector (size[3,1], [m/s]), containing the components of the velocity vector of the platform center of mass G, projected on the global frame.
   
    acceleration;%is a vector (size[3,1], [m/s^2]), containing the components of the acceleretion vector of the platform point P, projected on the global frame.
    angular_acc;%is a vector (size[3,1], [rad/s^2]), containing the components of angular acceleration, projected on the global frame.
    acc_OG_glob;%is a vector (size[3,1], [m/s^2]), containing the components of the acceleration vector of the platform point G, projected on the global frame.
    
    inertia_matrix_global;%is the inertia matrix(size[3,3] [kg m^2]), expressed in the global frame.    
    mass_matrix_global;%is the mass matrix(size[6,6]), expressed in the global frame.   
    ext_load;%
    dyn_load;%
    total_load;%
    
    orientation;%is a vector (size[3,1], [rad]), containing the angles of rotation used to parameterize the platform orientation.
    quaternion;%is a vector (size[4,1]), containing the components of the quaternion used to parameterize platform orientation.
    pose;%is a vector(size[6,1]), containing the the properties POSITION and ORIENTATION, used to parameterize platform pose.
    pose_q;%is a vector(size[7,1]), containing the the properties POSITION and QUATERNION, used to parameterize platform pose.
    
    orientation_deriv;%is a vector (size[3,1], [rad/s]), containing the 1th order time derivatives of angles of rotation.
    quaternion_deriv;%is a vector (size[3,1]), containing the 1th order time derivatives of QUATERNION.
    pose_d;
    pose_q_d;
    H_mat;%is a matrix (size[3,3]) that transforms the 1th order derivatives of rotation angles to angular velocity.
    
    orientation_deriv_2;%is a vector (size[3,1], [rad/s^2]), containing the 2nd order time derivatives of angles of rotation.
    quaternion_deriv_2;%is a vector (size[3,1]), containing the 2nd order time derivatives of QUATERNION.
    pose_d_2;
    pose_q_d_2;
    H_mat_deriv;%is a matrix (size[3,3]), obtained by time differentiation of the matrix H_MAT.
    
    mass_matrix_global_ss;%is the mass matrix(size[6,6]), expressed in the global frame. 
    ext_load_ss;%
    dyn_load_ss;%
    total_load_ss;%
    

  end
  methods
    function obj = UpdatePose(obj,pos,orient,ang_type)
        %UPDATEPOSE updates 0th order kinematic variables.
        %   UPDATEPOSE updates time dependent variables according to the  
        %   method used for rotation parameterization.
        %
        %   POS is a vector (size[3,1],[m]), containing the components of
        %   the position vector of the platform point P, projected in the 
        %   global frame.
        %   ORIENT is is a vector (size[3,1], [rad]), containing the angles 
        %   of rotation used to parameterize the platform orientation.
        %   ANG_TYPE is a string containing the name of the method used for
        %   rotation parameterization.
        
      obj.position = pos;
      switch (ang_type)
        case RotationParametrizations.EULER_ZYZ
          obj.rot_mat = RotZYZ(orient);
          obj.quaternion = Rot2Quat(obj.rot_mat);
          obj.orientation = orient;
          obj.pose = [pos;orient];
          obj.pose_q = [pos;obj.quaternion];
          obj.H_mat = HtfZYZ(obj.orientation); 
        case RotationParametrizations.TAYT_BRYAN
          obj.rot_mat = RotXYZ(orient);
          obj.orientation = orient;
          obj.pose = [pos;orient];
          obj.quaternion = Rot2Quat(obj.rot_mat);
          obj.pose_q = [pos;obj.quaternion];
          obj.H_mat = HtfTaytBryan(obj.orientation);
        case RotationParametrizations.RPY
          obj.rot_mat = RotRPY(orient);
          obj.orientation = orient;
          obj.pose = [pos;orient];
          obj.quaternion = Rot2Quat(obj.rot_mat);
          obj.pose_q = [pos;obj.quaternion];
          obj.H_mat = HtfRPY(obj.orientation);
        case RotationParametrizations.TILT_TORSION
          obj.rot_mat = RotTiltTorsion(orient);
          obj.orientation = orient;
          obj.pose = [pos;orient];
          obj.quaternion = Rot2Quat(obj.rot_mat);
          obj.pose_q = [pos;obj.quaternion];
          obj.H_mat = HtfTiltTorsion(obj.orientation);
        case RotationParametrizations.QUATERNION
          obj.rot_mat = Quat2Rot(orient);
          obj.quaternion = orient;
          obj.pose_q = [pos;orient];
          %obj.orient = ??;   
          %obj.pose = ??;
          obj.H_mat = HtfQuaternion(obj.quaternion);
      end
    end
    function obj = UpdateVelocity(obj,vel,orient_d,ang_type) 
        %UPDATEVELOCITY updates 1th order kinematic variables.
        %   UPDATEVELOCITY updates time dependent variables according to the  
        %   method used for rotation parameterization.
        %
        %   VEL is a vector (size[3,1],[m/s]), containing the components of
        %   the velocity vector of the platform point P, projected in the 
        %   global frame.
        %   ORIENT_D is a vector (size[3,1], [rad/s]), containing the 1th 
        %   order time derivatives of angles of rotation used to parameterize 
        %   the platform orientation.
        %   ANG_TYPE is a string containing the name of the method used for
        %   rotation parameterization.
      obj.velocity = vel;
      switch (ang_type)
        case RotationParametrizations.EULER_ZYZ
          obj.orientation_deriv = orient_d;
          %obj.quaternion_deriv = ??;
          obj.angular_vel = obj.H_mat*orient_d;
          obj.pose_d = [obj.velocity;obj.orientation_deriv];
          obj.H_mat_deriv = DHtfZYZ(obj.orientation,obj.orientation_deriv);   
        case RotationParametrizations.TAYT_BRYAN
          obj.orientation_deriv = orient_d;
          %obj.quaternion_deriv = ??;
          obj.angular_vel = obj.H_mat*orient_d;
          obj.pose_d = [obj.velocity;obj.orientation_deriv];
          obj.H_mat_deriv = DHtfTaytBryan(obj.orientation,obj.orientation_deriv);
        case RotationParametrizations.RPY
          obj.orientation_deriv = orient_d;
          %obj.quaternion_deriv = ??;
          obj.angular_vel = obj.H_mat*orient_d;
          obj.pose_d = [obj.velocity;obj.orientation_deriv];
          obj.H_mat_deriv = DHtfRPY(obj.orientation,obj.orientation_deriv);
        case RotationParametrizations.TILT_TORSION
          obj.orientation_deriv = orient_d;
          %obj.quaternion_deriv = ??;
          obj.angular_vel = obj.H_mat*orient_d;
          obj.pose_d = [obj.velocity;obj.orientation_deriv];
          obj.H_mat_deriv = DHtfTiltTorsion(obj.orientation,obj.orientation_deriv);
        case RotationParametrizations.QUATERNION
          %obj.orientation_deriv = orient_d;
          obj.quaternion_deriv = orient_d;
          obj.angular_vel = obj.H_mat*orient_d;
          obj.pose_q_d = [obj.velocity;obj.orientation_deriv];
          obj.H_mat_deriv = DHtfQuaternion(obj.quaternion,obj.quaternion_deriv);
      end
    end
    function obj = UpdateAcceleration(obj,acc,orient_d_2,ang_type)
        %UPDATEACCELERATION updates 2nd order kinematic variables.
        %   UPDATEACCELERATION  updates time dependent variables according  
        %   to the method used for rotation parameterization.
        %
        %   ACC is a vector (size[3,1],[m/s^2]), containing the components 
        %   of the acceleration vector of the platform point P, projected 
        %   in the global frame.
        %   ORIENT_D_2 is a vector (size[3,1], [rad/s^2]), containing the 
        %   2nd order time derivatives of angles of rotation used to 
        %   parameterize the platform orientation.
        %   ANG_TYPE is a string containing the name of the method used for
        %   rotation parameterization.
      obj.acceleration = acc;
      switch (ang_type)
        case RotationParametrizations.EULER_ZYZ
          obj.orientation_deriv_2 = orient_d_2;
          %obj.quaternion_deriv = ??;
          obj.angular_acc = obj.H_mat*orient_d_2 +...
            obj.H_mat_deriv*obj.orientation_deriv;
          obj.pose_d_2 = [obj.acceleration;obj.orientation_deriv_2];
        case RotationParametrizations.TAYT_BRYAN
          obj.orientation_deriv_2 = orient_d_2;
          %obj.quaternion_deriv = ??;
          obj.angular_acc = obj.H_mat*orient_d_2 +...
            obj.H_mat_deriv*obj.orientation_deriv;
          obj.pose_d_2 = [obj.acceleration;obj.orientation_deriv_2];
        case RotationParametrizations.RPY
          obj.orientation_deriv_2 = orient_d_2;
          %obj.quaternion_deriv = ??;
          obj.angular_acc = obj.H_mat*orient_d_2 +...
            obj.H_mat_deriv*obj.orientation_deriv;
          obj.pose_d_2 = [obj.acceleration;obj.orientation_deriv_2];
        case RotationParametrizations.TILT_TORSION
          obj.orientation_deriv_2 = orient_d_2;
          %obj.quaternion_deriv = ??;
          obj.angular_acc = obj.H_mat*orient_d_2 +...
            obj.H_mat_deriv*obj.orientation_deriv;
          obj.pose_d_2 = [obj.acceleration;obj.orientation_deriv_2];
        case RotationParametrizations.QUATERNION
          %obj.orientation_deriv_2 = ??;
          obj.quaternion_deriv_2 = orient_d_2;
          obj.angular_acc = obj.H_mat*orient_d_2 +...
            obj.H_mat_deriv*obj.orientation_deriv;
          obj.pose_q_d_2 = [obj.acceleration;obj.quaternion_deriv_2];
      end
      
    end
  function obj = UpdateMassMatrix(obj,par)
        %UPDATEMASSMATRIX updates the mass matrix of the platform. 
        %
        %   PAR is a structure containing the inertial properties of the 
        %   platform.
    anti_com = Anti(obj.pos_PG_glob);
    obj.mass_matrix_global(1:3,1:3) = eye(3).*par.platform.mass;
    obj.inertia_matrix_global = obj.rot_mat*par.platform.inertia_mat_G_loc*obj.rot_mat'-...
      par.platform.mass.*anti_com*anti_com;
    obj.mass_matrix_global(1:3,4:6) = -par.platform.mass.*anti_com;
    obj.mass_matrix_global(4:6,1:3) = par.platform.mass.*anti_com;
    obj.mass_matrix_global(4:6,4:6) = obj.inertia_matrix_global;
  end
  function obj = UpdateMassMatrixStateSpace(obj,par)
        %UPDATEMASSMATRIX updates the state space mass matrix of the platform. 
        %
        %   PAR is a structure containing the inertial properties of the 
        %   platform.
    obj = obj.UpdateMassMatrix(par);
    obj.mass_matrix_global_ss(1:3,1:3) = obj.mass_matrix_global(1:3,1:3);
    obj.mass_matrix_global_ss(1:3,4:6) = obj.mass_matrix_global(1:3,4:6)*obj.H_mat;
    obj.mass_matrix_global_ss(4:6,1:3) = obj.H_mat'*obj.mass_matrix_global(4:6,1:3);
    obj.mass_matrix_global_ss(4:6,4:6) = obj.H_mat'*obj.mass_matrix_global(4:6,4:6)*obj.H_mat;
  end
  end
end