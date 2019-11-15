function mat = Anti(v)
%ANTI computes the skew-symmetric matrix related to the array v.
%   The matrix is defined as follows in order to perform cross-products
%   in matrix form.
%
%   V (size[3,1]) is the array of coordinates.
%
%   MAT (size[3,3]) is the resulting skew-symmetric matrix.


mat = [0 -v(3) v(2);
    v(3) 0 -v(1);
    -v(2) v(1) 0];

end