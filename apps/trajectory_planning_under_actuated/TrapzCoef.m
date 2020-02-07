function matC = TrapzCoef(lims)

a = lims(1); b = lims(2);
matT0 = Poly6MatT(0);
matTa = Poly6MatT(a);
matTb = Poly6MatT(1-b);
matT1 = Poly6MatT(1);
BC0 = zeros(2,1); BC1 = eye(2,1);

matT = zeros(8);
vectT = zeros(8,1);

matT(1:2,1:3) = matT0(1:2,1:3);
matT(3:4,1:3) = matTa(1:2,1:3);
matT(3:4,4:5) = -matTa(1:2,1:2);
matT(5:6,4:5) = matTb(1:2,1:2);
matT(5:6,6:8) = -matTb(1:2,1:3);
matT(7:8,6:8) = matT1(1:2,1:3);

vectT(1:2) = BC0;
vectT(3:4) = BC0;
vectT(5:6) = BC0;
vectT(7:8) = BC1;

c = linsolve(matT,vectT);
matC = zeros(7,3);
matC(:,1) = [c(1:3);0;0;0;0];
matC(:,2) = [c(4:5);0;0;0;0;0];
matC(:,3) = [c(6:8);0;0;0;0];
end