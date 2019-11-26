function [delta_l,delta_sw] = Reparse(s,par)

    l = zeros(length(s.id)/par.n_cables,1);
    an = l;
    motor_indices = unique(s.id);
    for i=1:par.n_cables
        id = find(s.id == motor_indices(i));
        for j = 1:length(id)
           l(j,i) =  s.cable_length(id(j));
           an(j,i) =  s.pulley_angle(id(j));
        end
    end
    delta_l = l;
    delta_sw = an;
    
end