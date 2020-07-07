function cable_v = UpdateAccA(pos_PA_glob,acceleration,angular_velocity,...
                      angular_acceleration,cable_v)
%UPDATEACCA updates the acceleration vector of the point A.
%
%   UPDATEACCA updates the acceleration vector of the distal anchor point A, 
%   projecting the components on the global frame.
%
%   POS_PA_GLOB is a vector (size[3,1], [m/s]), containing the components 
%   of the position vector (A-P), projected on the global frame.
%   ACCELERATION is a vector (size[3,1],[m/s]), containing the components of
%   the 2th order time derivative of the position vector (P-O), projected 
%   on the global frame.
%   ANGULAR_VELOCITY is a vector (size[3,1], [rad/s]), containing the
%   components of the angular velocity, projected on the global frame.
%   ANGULAR_ACCELERATION is a vector (size[3,1], [rad/s]), containing the
%   components of the angular acceleration, projected on the global frame.
%   CABLE_V is  a structure containing time dependent cable variables.
%
%   ACC_OA_GLOB is a vector(size[3,1],[m/s]), containing the components of
%   the 2nd order time derivative of the position vector (A-O), projected
%   on the global frame.

cable_v.acc_OA_glob = acceleration + cross(angular_acceleration,pos_PA_glob)...
  +cross(angular_velocity,cross(angular_velocity,pos_PA_glob));

end