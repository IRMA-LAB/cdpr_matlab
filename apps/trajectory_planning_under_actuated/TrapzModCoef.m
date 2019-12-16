function matC = TrapzModCoef(lims)

a = lims(1); b = lims(2);
matT0 = Poly6MatT(0);
matTa = Poly6MatT(a);
matTb = Poly6MatT(1-b);
matT1 = Poly6MatT(1);
BC0 = zeros(3,1); BC1 = eye(3,1);

matT = zeros(8);
vectT = zeros(8,1);

matT(1:3,1:5) = matT0(1:3,1:5);
matT(4:6,1:5) = matTa(1:3,1:5);
matT(4:6,6:7) = -matTa(1:3,1:2);
matT(7:9,6:7) = matTb(1:3,1:2);
matT(7:9,8:12) = -matTb(1:3,1:5);
matT(10:12,8:12) = matT1(1:3,1:5);

vectT(1:3) = BC0;
vectT(4:6) = BC0;
vectT(7:9) = BC0;
vectT(10:12) = BC1;

c = linsolve(matT,vectT);
matC = zeros(7,3);
matC(:,1) = [c(1:5);0;0];
matC(:,2) = [c(6:7);0;0;0;0;0];
matC(:,3) = [c(8:12);0;0];
end