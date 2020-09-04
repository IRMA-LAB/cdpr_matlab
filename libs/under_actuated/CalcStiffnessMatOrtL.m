function matrix = CalcStiffnessMatOrtL(ort,par,K)

matrix = ort'*K*par;

end