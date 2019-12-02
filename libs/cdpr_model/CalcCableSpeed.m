function complete_speed = CalcCableSpeed(vers_t, vel_OA_glob)
%CALCCABLESPEED computes the cable speed.
%
%   CALCCABLESPEED computes the 1th order time derivative of the cable
%   length.
%
%   VERS_RHO is a vector (size[3,1]), containing the components of the
%   unit vector equidirectional and equiverse to the position vector
%   (A-B). The aforementioned components are projected on the global frame.
%   VEL_OA_GLOB is a vector (size[3,1],[m/s]), containing the components
%   of the velocity vector of the distal anchor point A, projected on the
%   global frame.

complete_speed = dot(vers_t, vel_OA_glob);
