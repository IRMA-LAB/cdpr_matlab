clear all
close all
clc

addpath('../../libs/export_utilities')

fr(1) = 2;
V = 0.05;
X = nthroot(V^2*(sqrt(1-V^2)+1), 3);
nn = pi/(asin(sqrt(2*X/(3*X^2+2*X+3*V^2))))-1;
fr(2) = (1+nn)/2*fr(1);
fr(3) = nn*fr(1);
fr(4) = (3*nn-1)/2*fr(1);
% fr(3) = fr(1)+5*(fr(2)-fr(1));
%fr(2) = fr(1)+1/3*(fr(3)-fr(1));
shaper_order = 1;
ut = UtilitiesType;
[S_2h,index_2h] = ComputeTwoHumpsEIShaper(fr(2),V);
[S_3h,index_3h] = ComputeThreeHumpsEIShaper(mean(fr),V);
[S_dir_an,index_an] = ComputeDirect3ModeN(fr(1),nn);
% guess = S_2h;

[S_conv,index_conv] = ComputeThreeModeShaper(fr,shaper_order);
z = zeros(1,length(fr));
n_pulse = 1+length(fr)*shaper_order;
%[guess,~] = ZeroVibrationInputShaping(n_pulse-1,sum(1./f)/2*shaper_order/(n_pulse));
t_start = 1/(8*max(fr)*shaper_order);
t_end = 8/(min(fr)*shaper_order);
trials = 200;
i = 0;
for t = t_start:(t_end-t_start)/trials:t_end
      [guess,~] = ZeroVibrationInputShapingNImpulse(n_pulse,t);
    [s1,f_val] = fsolve(@(x) MultiModeVibrationNullFunction(x,fr,z,shaper_order),[guess(1,:)';guess(2,2:end)'],ut.fsolve_options_grad);
    if (norm(f_val)<0.001)
        S = [s1(1:(length(s1)+1)/2)';0 s1((length(s1)+1)/2+1:end)'];
        S(2,:) = S(2,:)-min(S(2,:));
        [S(2,:),idx] = sort(S(2,:));
        S(1,:) = S(1,idx);
        index = length(S);
        if (~any(S(1,:)<0) && (floor(S(1,1)*10000)==floor(S(1,end)*10000)))
            i=i+1;
            shaper(:,:,i) =S;
        end
    end
end
[~,~,n] = size(shaper);
for i=1:n
    mat(:,i) =  floor(shaper(:,1,i).*10000);
end
[~,c,~] = unique(mat','rows');
shaper_min = shaper(:,:,c);
[~,idx] = min(shaper_min(2,end,:));
S_dir = shaper_min(:,:,idx);
index_dir = index;
S_dir_2 = S_dir;
S_dir_2(2,:) = S_dir(2,:)-S_dir(2,end)/2;
S = S_dir;
A = S_dir_2(1,3);
B = S_dir_2(1,4);
alfa = (S_dir_2(2,3)-S_dir_2(2,2))/2;
beta = (S_dir_2(2,4)-S_dir_2(2,1))/2;
ABRatio = A/B;
coss(2,1) = cos(2*pi*fr(1)*alfa);
coss(1,1) = cos(2*pi*fr(1)*beta);
coss(2,2) = cos(2*pi*fr(2)*alfa);
coss(1,2) = cos(2*pi*fr(2)*beta);
coss(2,3) = cos(2*pi*fr(3)*alfa);
coss(1,3) = cos(2*pi*fr(3)*beta);

j=0;
k=0;
for ff=0.1*min(fr):0.001:2*max(fr)
j=j+1;
f(j,1) = ff;
v(j,1) = VibrtionResidual(S_dir,index,2*pi*ff,0);
% %v(j,2) = VibrtionResidual(S_dir_an,index_an,2*pi*ff,0);
 v(j,2) = VibrtionResidual(S_3h,index_3h,2*pi*ff,0);
% if (f(j,1)>=fr(1)*0.9 && f(j,1)<=fr(4)*1.1)
%     k=k+1;
%     f_int(k,1) = f(j,1);
%     v_int(k,1) = v(j,1);
%     v_int(k,2) = v(j,2);
% end
 %v(j,2) = VibrtionResidual3modeD(S_dir_2,index,2*pi*ff,0);
end

plot(f,v(:,1),f,v(:,2),f,0.05*ones(length(f)))