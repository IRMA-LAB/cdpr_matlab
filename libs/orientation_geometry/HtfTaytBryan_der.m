function mat = HtfTaytBryan_der(angle,var_n)

c1 = cos(angle(1)); s1 = sin(angle(1));
c2 = cos(angle(2)); s2 = sin(angle(2));
mat = zeros(3);

switch var_n
    case 1
        mat(2,2) = -s1;
        mat(2,3) = -c1*c2;
        mat(3,2) = c1;
        mat(3,3) = -s1*c2;
    case 2
        mat(1,3) = c2;
        mat(2,3) = +s1*s2;
        mat(3,3) = -c1*s2;
    case 3
end

end