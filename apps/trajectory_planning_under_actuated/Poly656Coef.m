function matC = Poly656Coef(lims,K)

a = lims(1); b = lims(2);
matT0 = Poly6MatT(0);
matTa = Poly6MatT(a);
matTb = Poly6MatT(1-b);
matT1 = Poly6MatT(1);
BC0 = zeros(3,1); BC1 = eye(3,1);

matT = zeros(14);
vectT = zeros(14,1);

matT(1:3,1:5) = matT0(1:3,1:5);
matT(4:7,1:5) = matTa(1:4,1:5);
matT(4:7,6:9) = -matTa(1:4,1:4);
matT(8:11,6:9) = matTb(1:4,1:4);
matT(8:11,10:14) = -matTb(1:4,1:5);
matT(12:14,10:14) = matT1(1:3,1:5);

vectT(1:3) = BC0-matT0(1:3,6:7)*K(:,1);
vectT(4:7) = matTa(1:4,5:6)*K(:,2)-matTa(1:4,6:7)*K(:,1);
vectT(8:11) = matTb(1:4,6:7)*K(:,3)-matTb(1:4,5:6)*K(:,2);
vectT(12:14) = BC1-matT1(1:3,6:7)*K(:,3);

c = linsolve(matT,vectT);
matC = zeros(7,3);
matC(:,1) = [c(1:5);K(:,1)];
matC(:,2) = [c(6:9);K(:,2);0];
matC(:,3) = [c(10:14);K(:,3)];
end