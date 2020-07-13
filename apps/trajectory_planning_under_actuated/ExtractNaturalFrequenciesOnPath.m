function f = ExtractNaturalFrequenciesOnPath(ps,pf,gFun,par,var,ut)

ff = [];
n = 100;
for j=0:n
    v = gFun(j/n,ps(1:3),pf(1:3));
    pose = [v(1,:)'; (ps(4:end)+pf(4:end))/2];
    freq = CalcNatFreqAssignedPos(par,var,ut,pose);
    ff = [ff freq];
end
f_tot = reshape(ff,1,[]);
f = [min(f_tot);mean(f_tot);max(f_tot)];

end