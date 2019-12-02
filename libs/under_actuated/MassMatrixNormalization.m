function eigenvectors = MassMatrixNormalization(eigenvectors,matrix)

for i=1:length(eigenvectors)
   scalar =  eigenvectors(:,i)'*matrix*eigenvectors(:,i);
   eigenvectors(:,i) = eigenvectors(:,i)./sqrt(scalar);
end

end