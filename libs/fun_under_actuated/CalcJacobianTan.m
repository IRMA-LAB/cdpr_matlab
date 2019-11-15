function matrix = CalcJacobianTan(cdpr_v)

n = length(cdpr_v.cable); m = length(cdpr_v.analitic_jacobian);
matrix = zeros(n,m);
for i=1:n
    matrix(i,:) = [cdpr_v.cable(i).vers_n' ...
        -cdpr_v.cable(i).vers_n'*Anti(cdpr_v.cable(i).pos_PA_glob)*cdpr_v.platform.H_mat]./...
        norm(cdpr_v.cable(i).pos_BA_glob);
end

end