function [cdpr_v,constraint] = UnderactuatedStaticConstraint(cdpr_v)


constraint = cdpr_v.underactuated_platform.geometric_orthogonal'*cdpr_v.platform.ext_load;
cdpr_v = CalcCablesStaticTension(cdpr_v);

for i=1:length(cdpr_v.cable)
    if (isnan(cdpr_v.tension_vector(i)))
        constraint=NaN(length(constraint,1));
        break
    end
end

end