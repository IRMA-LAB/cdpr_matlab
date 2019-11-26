classdef WinchStatus < handle
    properties
        id
        op_mode
        motor_position
        motor_speed
        motor_torque
        cable_length
        aux_position
    end
    
    methods
        function set(obj, winch_status_packed)
            obj.id = winch_status_packed(1);
            obj.op_mode = winch_status_packed(2);
            obj.motor_position = winch_status_packed(3);
            obj.motor_speed = winch_status_packed(4);
            obj.motor_torque = winch_status_packed(5);
            obj.cable_length = winch_status_packed(6);
            obj.aux_position = winch_status_packed(7);
        end
        
        function append(obj, winch_status_packed)
            obj.id(end + 1) = winch_status_packed(1);
            obj.op_mode(end + 1) = winch_status_packed(2);
            obj.motor_position(end + 1) = winch_status_packed(3);
            obj.motor_speed(end + 1) = winch_status_packed(4);
            obj.motor_torque(end + 1) = winch_status_packed(5);
            obj.cable_length(end + 1) = winch_status_packed(6);
            obj.aux_position(end + 1) = winch_status_packed(7);
        end        
        
        function clear(obj)
            obj.id = [];
            obj.op_mode = [];
            obj.motor_position = [];
            obj.motor_speed = [];
            obj.motor_torque = [];
            obj.cable_length = [];
            obj.aux_position = [];
        end
    end
end
