function matrix = CalcJacobianSw(cdpr_v)

n = length(cdpr_v.cable); m = length(cdpr_v.analitic_jacobian);
matrix = zeros(n,m);
for i=1:n
    matrix(i,:) = [cdpr_v.cable(i).vers_w' ...
        -cdpr_v.cable(i).vers_w'*Anti(cdpr_v.cable(i).pos_PA_glob)*cdpr_v.platform.H_mat]./...
        cdpr_v.cable(i).vers_u'*cdpr_v.cable(i).pos_DA_glob;
end

end