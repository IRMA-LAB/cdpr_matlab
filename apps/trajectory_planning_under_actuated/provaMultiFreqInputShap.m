clear all
close all
clc
fmincon_options = optimoptions('fmincon','Algorithm','interior-point',...
            'FunctionTolerance',1e-8,'MaxFunctionEvaluation',1000000,...
            'MaxIterations',1000000,'OptimalityTolerance',1e-2,'StepTolerance',1e-8,'UseParallel',true,'ConstraintTolerance',1e-8);
fsolve_options = optimoptions('fsolve','Algorithm',...
            'levenberg-marquardt','FunctionTolerance',1e-8,'MaxFunctionEvaluation',...
            1000000,'MaxIterations',1000000,'OptimalityTolerance',1e-1,...
            'StepTolerance',1e-8,'UseParallel',true);       
f = [0.4;4;5]; z = zeros(1,length(f)); order = 2;
n_pulse = 1+length(f)*order;
%[guess,~] = ZeroVibrationInputShapingNImpulse(n_pulse,1/(f(5)));
[guess,~] = ZeroVibrationInputShaping(n_pulse-1,1/(f(2)));
s1 = fsolve(@(x) MultiModeVibrationNullFunction(x,f,z,order),[guess(1,:)';guess(2,2:end)'],fsolve_options);
%s2 = fmincon(@(x)0,[guess(1,:)';guess(2,2:end)'],[],[],[],[],[],[],@(x)MultiModeVibrationNullFunctionPositive(x,f,z,order),fmincon_options);
shaper = [s1(1:(length(s1)+1)/2)';0 s1((length(s1)+1)/2+1:end)'];
shaperL = length(shaper);
[shaper1,shaperL1] = ZeroVibrationInputShaping(order,1/f(1));
[shaper2,shaperL2] = ZeroVibrationInputShaping(order,1/f(2));
[shaper3,shaperL3] = ZeroVibrationInputShaping(order,1/f(3));
k=0;
for freq=0:0.01:10
k = k+1;
ff(k,1) = freq;
v(k,1) = VibrtionResidual(shaper1,shaperL1,2*pi*freq,0);
v(k,2) = VibrtionResidual(shaper2,shaperL2,2*pi*freq,0);
v(k,3) = VibrtionResidual(shaper3,shaperL3,2*pi*freq,0);
v(k,4) = VibrtionResidual(shaper,shaperL,2*pi*freq,0);
end
plot(ff,v(:,1),ff,v(:,2),ff,v(:,3),ff,v(:,4))
