function [S,index] = ComputeTwoHumpsEIShaper(f,V)

    T = 1/f;
    X = nthroot(V^2*(sqrt(1-V^2)+1), 3);
    A = (3*X^2+2*X+3*V^2)/(16*X);
    S = [A 0.5-A 0.5-A A;
         0 0.5*T T 1.5*T];
    index = 4;
end