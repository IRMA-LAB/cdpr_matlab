function platform_v = UpdatePlatformAcceleration(acceleration,orientation_deriv_2,rot_par,platform_v)
%UPDATEPLATFORMACCELERATION updates the platform acceleration.
%   UPDATEPLATFORMACCELERATION updates the time dependent variables associated
%   to the 2nd order time derivative of the platform pose and stores them 
%   in a proper structure. 
%
%   ACCELERATION is a vector (size[3,1],[m]), containing the components of 
%   the 2nd order time derivative of the position vector (P-O), projected 
%   on the global frame.
%   ORIENTATION_DERIV_2 can be a vector(size[3,1],[rad/s]), containing the
%   2nd order time derivative of angles of rotation or a vector
%   (size[4,1]),containing the components of the 2nd order time derivative
%   of the quaternion, used to parameterize the platform orientation.
%   ROT_PAR is a string containing the name of the method used to
%   parameterize the platform orientation.
%   PLATFORM_V is a structure containing the time dependent platform 
%   variables.
platform_v = platform_v.UpdateAcceleration(acceleration,orientation_deriv_2,rot_par);
platform_v.acc_OG_glob = platform_v.acceleration + cross(platform_v.angular_acc,platform_v.pos_PG_glob)+...
  cross(platform_v.angular_vel,cross(platform_v.angular_vel,platform_v.pos_PG_glob));

end