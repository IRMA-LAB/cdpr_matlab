function val = Poly535(t,lims,matC)

a = lims(1); b = lims(2);
matT = Poly5MatT(t);

if (t<0)
    matT = Poly5MatT(0);
elseif(t>1)
    matT = Poly5MatT(1);
end

if(t<=a)
    val(1,1) = matT(1,:)*matC(:,1);
    val(2,1) = matT(2,:)*matC(:,1);
    val(3,1) = matT(3,:)*matC(:,1);
elseif(t<1-b)
    val(1,1) = matT(1,:)*matC(:,2);
    val(2,1) = matT(2,:)*matC(:,2);
    val(3,1) = matT(3,:)*matC(:,2);
else
    val(1,1) = matT(1,:)*matC(:,3);
    val(2,1) = matT(2,:)*matC(:,3);
    val(3,1) = matT(3,:)*matC(:,3);
end

end