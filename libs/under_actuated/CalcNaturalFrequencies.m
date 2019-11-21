function [vector,matrix] = CalcNaturalFrequencies(cdpr_p,pose)

cdpr_v = CdprVar(cdpr_p.n_cables);

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians(cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);
cdpr_v.platform = cdpr_v.platform.UpdateMassMatrix(cdpr_p);
cdpr_v = CalcCablesStaticTension(cdpr_v);

K_matrix = CalcStiffnessMatUnder(cdpr_v);
M_matrix = CalcMassMatUnder(cdpr_v);
D_matrix = zeros(length(M_matrix));

for i=1:length(D_matrix)
    D_matrix(:,i) = linsolve(M_matrix,K_matrix(:,i)); 
end
[eigenvectors,eigenvalues_mat] = eig(D_matrix);

matrix = eigenvectors;
vector = sqrt(diag(eigenvalues_mat))./(2.*pi);

end