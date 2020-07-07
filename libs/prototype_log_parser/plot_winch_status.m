clc; clear; close all;

%% Parse file
filepath = '/tmp/cable-robot-logs/data.log';
data = parseCableRobotLogFile(filepath);

%% Pack according to actuator ID
[sorted_id, sorting_idx] = sort(data.winch_status.values.id);

actuators_id = unique(sorted_id);
num_actuators = length(actuators_id);
MSG_NUM = floor(length(sorted_id)/num_actuators);
torques_mat = zeros(num_actuators, MSG_NUM);
pulley_enc_mat = torques_mat;
sorted_ts = torques_mat;
for i = 1:num_actuators
    idx = sorting_idx(sorted_id == actuators_id(i));
    torques_mat(i, :) = data.winch_status.values.motor_torque(idx(1:MSG_NUM));
    pulley_enc_mat(i, :) = data.winch_status.values.aux_position(idx(1:MSG_NUM));
    sorted_ts(i, :) = data.winch_status.timestamp(idx(1:MSG_NUM));
end

%% Plot pulley encoder values
figure('units','normalized','outerposition',[0 0 1 1])
for i = 1:num_actuators
    subplot(num_actuators,1,i)
    plot(sorted_ts(i,:), pulley_enc_mat(i,:))
    hold on
    plot([TOUCH_TIME TOUCH_TIME], ylim, '-.')
    plot([settling_time settling_time], ylim, '-.')
    hold off
    grid on
    title(sprintf('Actuator #%d encoder values', actuators_id(i)))
    xlabel('[sec]')
    xlim([sorted_ts(i,1), sorted_ts(i,end)])
end

%% Plot torques
figure('units','normalized','outerposition',[0 0 1 1])
for i = 1:num_actuators
    subplot(num_actuators,1,i)
    plot(sorted_ts(i,:), torques_mat(i,:))
    grid on
    title(sprintf('Actuator #%d torques', actuators_id(i)))
    xlabel('[sec]')
    xlim([sorted_ts(i,1), sorted_ts(i,end)])
end