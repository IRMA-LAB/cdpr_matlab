function mat = Anti(v)

mat = zeros(3);
mat(1,2) = -v(3);
mat(1,3) = v(2);
mat(2,1) = v(3);
mat(2,3) = -v(1);
mat(3,1) = -v(2);
mat(3,2) = v(1);

end