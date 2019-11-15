function platform_v = UpdatePlatformVelocity(velocity,orientation_deriv,rot_par,platform_v)
%UPDATEPLATFORMVELOCITY updates the platform velocity.
%   UPDATEPLATFORMVELOCITY updates the time dependent variables associated
%   to the 1th order time derivative of the platform pose and stores them 
%   in a proper structure. 
%
%   VELOCITY is a vector (size[3,1],[m]), containing the components of the
%   1th order time derivative of the position vector (P-O), projected on
%   the global frame.
%   ORIENTATION_DERIV can be a vector(size[3,1],[rad/s]), containing the
%   1th order time derivative of angles of rotation or a vector
%   (size[4,1]),containing the components of the 1th order time derivative
%   of the quaternion, used to parameterize the platform orientation.
%   ROT_PAR is a string containing the name of the method used to
%   parameterize the platform orientation.
%   PLATFORM_V is a structure containing the time dependent platform 
%   variables.

platform_v = platform_v.UpdateVelocity(velocity,orientation_deriv,rot_par);
platform_v.vel_OG_glob = platform_v.velocity + cross(platform_v.angular_vel,platform_v.pos_PG_glob);

end