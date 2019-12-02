clear all
clc

n_data = 3;
motors_id = [0,4,6];
traj_type = 0;
relative = 0;
for i=1:n_data
    traj = load(strcat('InverseRTR',int2str(i),'.mat'));
    fid = fopen(strcat('pvtRTR',int2str(i),'.txt'), 'w');
    fprintf(fid,'%d %d %d %d %d\n', [traj_type, relative, motors_id]);
    fprintf(fid,'%d %d %d %d\n', [traj.t', traj.cables(1).complete_length', traj.cables(2).complete_length', traj.cables(3).complete_length'].');
    fclose(fid);
end

for i=1:n_data
    traj = load(strcat('InverseSTD',int2str(i),'.mat'));
    fid = fopen(strcat('pvtSTD',int2str(i),'.txt'), 'w');
    fprintf(fid,'%d %d %d %d %d\n', [traj_type, relative, motors_id]);
    fprintf(fid,'%d %d %d %d\n', [traj.t', traj.cables(1).complete_length', traj.cables(2).complete_length', traj.cables(3).complete_length'].');
    fclose(fid);
end