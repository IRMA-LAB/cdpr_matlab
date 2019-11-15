function cable_v = UpdateVelA(pos_PA_glob,velocity,angular_velocity,cable_v)
%UPDATEVELA updates the velocity vector of the point A.
%
%   UPDATEVELA updates the velocity vector of the distal anchor point A, 
%   projecting the components on the global frame.
%
%   POS_PA_GLOB is a vector (size[3,1], [m/s]), containing the components 
%   of the position vector (A-P), projected on the global frame.
%   VELOCITY is a vector (size[3,1],[m/s]), containing the components of
%   the 1th order time derivative of the position vector (P-O), projected 
%   on the global frame.
%   ANGULAR_VELOCITY is a vector (size[3,1], [rad/s]), containing the
%   components of the angular velocity, projected on the global frame.
%   CABLE_V is  a structure containing time dependent cable variables.
%
%   VEL_OA_GLOB is a vector(size[3,1],[m/s]), containing the components of
%   the 1th order time derivative of the position vector (A-O), projected
%   on the global frame.

cable_v.vel_OA_glob = velocity + cross(angular_velocity,pos_PA_glob);

end