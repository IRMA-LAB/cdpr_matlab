function vect = CableLengthFromOscillation(x,cdpr_parameters,cdpr_variables,p)

    vect = [];
    data_l = length(p.val);
    for j=1:data_l
        pos = p.val(1:3,j);
        ang = p.val(4:6,j);
        cdpr_variables = UpdateIKZeroOrd(pos,...
            ang,cdpr_parameters,cdpr_variables);
        for i=1:cdpr_parameters.n_cables
            vect = [vect;x(i)-cdpr_variables.cable(i).complete_length];
        end
    end

end