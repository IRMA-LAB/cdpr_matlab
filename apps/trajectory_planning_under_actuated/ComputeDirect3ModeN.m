function [S,index] = ComputeDirect3ModeN(f,n)

    T = 1/f;
    alfa = T/(2*(1+n));
    B = 1/(8*sin(pi/(1+n))^2);
    S = [B 0.5-B 0.5-B B;
         -3*alfa -alfa alfa 3*alfa];
    S(2,:) = S(2,:)-S(2,1);
    index = 4;
end