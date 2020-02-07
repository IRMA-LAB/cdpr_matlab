
clear all
close all
clc
addpath('../../config')
addpath('../../data/workspace_files')
addpath('../../libs/cdpr_model')
addpath('../../libs/export_utilities')
addpath('../../libs/numeric')
addpath('../../libs/orientation_geometry')
addpath('../../libs/under_actuated')
folder = '../../data';

[cdpr_parameters, cdpr_variables, ws_parameters, cdpr_outputs,record,utilities] = ...
    LoadConfigAndInit("Grab_prototype_33","DynamicPlanning");
close all
f = [0.5;2];
z = zeros(1,length(f));
n_pulse = 7;
%[guess,~] = ZeroVibrationInputShaping(n_pulse-1,sum(1./f)/2*shaper_order/(n_pulse));
tStart = 1./max(f);
while 1
    [guess,~] = ZeroVibrationInputShapingNImpulse(n_pulse,tStart);
    [s1,fval] = fsolve(@(x) StopBandVibrationNullFunction(x,f,z,n_pulse),[guess(1,:)';guess(2,2:end)'],utilities.fsolve_options_grad);
    if (norm(fval)<0.001)
        S = [s1(1:(length(s1)+1)/2)';0 s1((length(s1)+1)/2+1:end)'];
        if(~any(S(1,:)<0))
        S(2,:) = S(2,:)-min(S(2,:));
        [S(2,:),idx] = sort(S(2,:));
        S(1,:) = S(1,idx);
        k = 0;
for w = 0:0.001:max(f)*1.1
k = k+1;
ff(k) = w;
v(k) = VibrtionResidual(S,length(S),2*pi*w,0);
end
plot(ff,v)
        end
       % break;
       tStart = tStart+0.1;
    else
        tStart = tStart+0.1;
    end
end


k = 0;
for w = 0:0.001:max(f)*1.1
k = k+1;
ff(k) = w;
v(k) = VibrtionResidual(S,length(S),2*pi*w,0);
end
plot(ff,v)
