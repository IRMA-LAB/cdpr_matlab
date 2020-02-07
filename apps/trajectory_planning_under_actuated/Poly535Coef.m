function matC = Poly535Coef(lims)

a = lims(1); b = lims(2);
matT0 = Poly6MatT(0);
matTa = Poly6MatT(a);
matTb = Poly6MatT(1-b);
matT1 = Poly6MatT(1);
BC0 = zeros(4,1); BC1 = eye(4,1);

matT = zeros(16);
vectT = zeros(16,1);

matT(1:4,1:6) = matT0(1:4,1:6);
matT(5:8,1:6) = matTa(1:4,1:6);
matT(5:8,7:10) = -matTa(1:4,1:4);
matT(9:12,7:10) = matTb(1:4,1:4);
matT(9:12,11:16) = -matTb(1:4,1:6);
matT(13:16,11:16) = matT1(1:4,1:6);

vectT(1:4) = BC0;
vectT(5:8) = BC0;
vectT(9:12) = BC0;
vectT(13:16) = BC1;

c = linsolve(matT,vectT);
matC = zeros(7,3);
matC(:,1) = [c(1:6);0];
matC(:,2) = [c(7:10);0;0;0];
matC(:,3) = [c(11:16);0];
end