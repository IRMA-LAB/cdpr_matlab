function matrix = CalcMatrixT(cable_v)

matrix = zeros(3);
matrix = matrix + cable_v.vers_w*cable_v.vers_w'.*...
    (sin(cable_v.tan_ang)./(cable_v.vers_u'*cable_v.pos_DA_glob));
matrix = matrix + cable_v.vers_n*cable_v.vers_n'./...
    norm(cable_v.pos_BA_glob);

end