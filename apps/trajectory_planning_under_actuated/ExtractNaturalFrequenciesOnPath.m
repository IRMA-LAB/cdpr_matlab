function f = ExtractNaturalFrequenciesOnPath(p,i,gFun,par,var,ut,mode_mask)

ff = [];
n = 100;
for j=0:n
    v = gFun(j/n,p(1:3,i),p(1:3,i+1));
    pose = [v(1,:)'; (p(4:end,i)+p(4:end,i+1))/2];
    freq = CalcNatFreqAssignedPos(par,var,ut,pose);
    ff = [ff freq(mode_mask)];
end
f_tot = reshape(ff,1,[]);
f = [min(f_tot);mean(f_tot);max(f_tot)];

end