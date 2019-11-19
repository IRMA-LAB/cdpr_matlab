function [vector,matrix] = CalcNaturalFrequencies(cdpr_p,pose)

cdpr_v = CdprVar(cdpr_p.n_cables);

cdpr_v = UpdateIKZeroOrd(pose(1:3),pose(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians(cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateAnaliticJacobians(cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian);
cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
cdpr_v = CalcCablesStaticTension(cdpr_v);

K_matrix = CalcStiffnessMat(cdpr_v,cdpr_p);
M_matrix = cdpr_v.underactuated_platform.geometric_orthogonal'*cdpr_v.platform.mass_matrix_global*cdpr_v.underactuated_platform.geometric_orthogonal;
[~,col] = size(K_matrix);
row = length(M_matrix);
D_matrix = zeros(row,col);

for i=1:col
    D_matrix(:,i) = linsolve(M_matrix,K_matrix(:,i)); 
end
[eigenvectors,eigenvalues_mat] = eig(D_matrix);

matrix = eigenvectors;
vector = sqrt(diag(eigenvalues_mat))./(2.*pi);

end