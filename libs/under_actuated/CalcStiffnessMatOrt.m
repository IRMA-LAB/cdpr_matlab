function matrix = CalcStiffnessMatOrt(ort,K)

matrix = ort'*K*ort;

end