function platform_v = UpdatePlatformPose(position,orientation,rot_par,pos_G_loc,platform_v)
%UPDATEPLATFORMPOSE updates the platform pose.
%
%   UPDATEPLATFORMPOSE updates the time dependent variables associated to 
%   the platform pose and stores them in a proper structure. 
%   
%   POSITION is a vector(size[3,1],[m]), containing the components of the
%   position vector (P-O), projected on the global frame.
%   ORIENTATION can be a vector(size[3,1],[rad]), containing the angles of
%   rotation or a vector (size[4,1]), containing the components of the
%   quaternion, used to parameterize the platform orientation.
%   ROT_PAR is a string containing the name of the method used to
%   parameterize the platform orientation.
%   POS_G_LOC is a vector(size[3,1],[m]), containing the components of the
%   position vector (G-P), projected on the local frame.
%   PLATFORM_V is a structure containing the time dependent platform 
%   variables.

platform_v = platform_v.UpdatePose(position,orientation,rot_par);
platform_v.pos_PG_glob = platform_v.rot_mat*pos_G_loc;
platform_v.pos_OG_glob = platform_v.position + platform_v.pos_PG_glob;

end