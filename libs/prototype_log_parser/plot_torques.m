clc; clear; close all;

%% Parse file
filepath = '/tmp/cable-robot-logs/data.log';
data = parseCableRobotLogFile(filepath);

%% Pack according to actuator ID
[sorted_id, sorting_idx] = sort(data.actuator_status.values.id);

actuators_id = unique(sorted_id);
num_actuators = length(actuators_id);
torques_mat = zeros(num_actuators, length(sorted_id)/num_actuators);
pulley_enc_mat = torques_mat;
pulley_angle_mat = torques_mat;
sorted_ts = torques_mat;
for i = 1:num_actuators
    idx = sorting_idx(sorted_id == actuators_id(i));
    torques_mat(i, :) = data.actuator_status.values.motor_torque(idx);
    pulley_enc_mat(i, :) = data.actuator_status.values.aux_position(idx);
    pulley_angle_mat(i, :) = data.actuator_status.values.pulley_angle(idx);
    sorted_ts(i, :) = data.actuator_status.timestamp(idx);
end
    
%% Plot torques
figure('units','normalized','outerposition',[0 0 1 1])
for i = 1:num_actuators
    subplot(3,1,i)
    plot(sorted_ts(i,:), torques_mat(i,:))
    grid on
    title(sprintf('Actuator #%d torques', actuators_id(i)))
    xlabel('[sec]')
    xlim([sorted_ts(i,1), sorted_ts(i,end)])
end
%% Plot pulley encoder values
figure()
for i = 1:num_actuators
    subplot(3,1,i)
    plot(sorted_ts(i,:), pulley_enc_mat(i,:))
    grid on
    title(sprintf('Actuator #%d encoder values', actuators_id(i)))
    xlabel('[sec]')
    xlim([sorted_ts(i,1), sorted_ts(i,end)])
end
%% Plot pulley angles
figure()
for i = 1:num_actuators
    subplot(3,1,i)
    plot(sorted_ts(i,:), pulley_angle_mat(i,:))
    grid on
    title(sprintf('Actuator #%d pulley angles', actuators_id(i)))
    xlabel('[sec]')
    xlim([sorted_ts(i,1), sorted_ts(i,end)])
end