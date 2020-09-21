function mat =  OverSigned(vect)

mat = zeros(3,6);

mat(1,1) = vect(1);
mat(2,2) = vect(2);
mat(3,3) = vect(3);

mat(1,4) = vect(2);
mat(1,5) = vect(3);

mat(2,4) = vect(1);
mat(2,6) = vect(3);

mat(3,5) = vect(1);
mat(3,6) = vect(2);

end