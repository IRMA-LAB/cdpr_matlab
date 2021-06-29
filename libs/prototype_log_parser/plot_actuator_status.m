clc; clear; close all;

%% Parse file
filepath = '/tmp/laser-engraver-logs/data.log';
data = parseCableRobotLogFile(filepath);
plot_flags = [0 0 0 0 1 1 1];

%% Pack according to actuator ID
[sorted_id, sorting_idx] = sort(data.actuator_status.values.id);

actuators_id = unique(sorted_id);
num_actuators = length(actuators_id);
torques_mat = zeros(num_actuators, length(sorted_id)/num_actuators);
motor_pos_mat = torques_mat;
motor_vel_mat = torques_mat;
pulley_enc_mat = torques_mat;
pulley_angle_mat = torques_mat;
cable_len_mat = torques_mat;
cable_tension_mat = torques_mat;
sorted_ts = torques_mat;
for i = 1:num_actuators
    idx = sorting_idx(sorted_id == actuators_id(i));
    torques_mat(i, :) = data.actuator_status.values.motor_torque(idx);
    motor_pos_mat(i, :) = data.actuator_status.values.motor_position(idx);
    motor_vel_mat(i, :) = data.actuator_status.values.motor_speed(idx);
    pulley_enc_mat(i, :) = data.actuator_status.values.aux_position(idx);
    pulley_angle_mat(i, :) = data.actuator_status.values.pulley_angle(idx);
    cable_len_mat(i, :) = data.actuator_status.values.cable_length(idx);
    cable_tension_mat(i, :) = data.actuator_status.values.cable_tension(idx);
    sorted_ts(i, :) = data.actuator_status.timestamp(idx);
end

%% Plot torques
if plot_flags(1)
%     figure('units','normalized','outerposition',[0 0 1 1])
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), torques_mat(i,:))
        grid on
        title(sprintf('Actuator #%d torques', actuators_id(i)))
        xlabel('[sec]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end
%% Plot positions
if plot_flags(2)
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), motor_pos_mat(i,:))
        grid on
        title(sprintf('Actuator #%d position', actuators_id(i)))
        xlabel('[sec]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end
%% Plot velocities
if plot_flags(2)
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), motor_vel_mat(i,:))
        grid on
        title(sprintf('Actuator #%d velocity', actuators_id(i)))
        xlabel('[sec]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end
%% Plot pulley encoder values
if plot_flags(4)
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), pulley_enc_mat(i,:))
        grid on
        title(sprintf('Actuator #%d encoder values', actuators_id(i)))
        xlabel('[sec]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end
%% Plot pulley angles
if plot_flags(5)
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), pulley_angle_mat(i,:))
        grid on
        title(sprintf('Actuator #%d pulley angles', actuators_id(i)))
        xlabel('[sec]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end
%% Plot cable lenghts
if plot_flags(6)
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), cable_len_mat(i,:))
        grid on
        title(sprintf('Actuator #%d cable lenght', actuators_id(i)))
        xlabel('[sec]')
        ylabel('[m]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end
%% Plot cable tensions
if plot_flags(7)
    figure()
    for i = 1:num_actuators
        subplot(4,1,i)
        plot(sorted_ts(i,:), cable_tension_mat(i,:))
        grid on
        title(sprintf('Actuator #%d cable tension', actuators_id(i)))
        xlabel('[sec]')
        ylabel('[N]')
        xlim([sorted_ts(i,1), sorted_ts(i,end)])
    end
end