function s = GetInfoCdpr(cdpr_v,t,j,s)

s.t(1,j) = t;
s.tension_vector(:,j) = cdpr_v.tension_vector;
s.platform.pose(:,j) = cdpr_v.platform.pose;
s.platform.pose_d(:,j) = cdpr_v.platform.pose_d;
s.platform.pose_d_2(:,j) = cdpr_v.platform.pose_d_2;

for i = 1:length(cdpr_v.cable)
    s.cable(i).complete_length(:,j) = cdpr_v.cable(i).complete_length;
    s.cable(i).complete_speed(:,j) = cdpr_v.cable(i).complete_speed;
    s.cable(i).complete_acceleration(:,j) = cdpr_v.cable(i).complete_acceleration;
    s.cable(i).swivel_ang(:,j) = cdpr_v.cable(i).swivel_ang;
    s.cable(i).swivel_ang_vel(:,j) = cdpr_v.cable(i).swivel_ang_vel;
    s.cable(i).swivel_ang_acc(:,j) = cdpr_v.cable(i).swivel_ang_acc;
    s.cable(i).tan_ang(:,j) = cdpr_v.cable(i).tan_ang;
    s.cable(i).tan_ang_vel(:,j) = cdpr_v.cable(i).tan_ang_vel;
    s.cable(i).tan_ang_acc(:,j) = cdpr_v.cable(i).tan_ang_acc;
end


end