function [S,index] = ComputeThreeHumpsEIShaper(f,V)

    T = 1/f;
    A = (1+3*V+2*sqrt(2*(V^2+V)))/16;
    B = (1-V)/4;
    S = [A B 1-2*(A+B) B A;
         0 0.5*T T 1.5*T 2*T];
    index = 5;
end