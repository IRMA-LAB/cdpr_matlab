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



shaper_order = 4;
shaper_freq1 = 0.4;
k=3;
shaper_freq2= (1+2*k)*shaper_freq1;
shaper_period3= 0.1;
[shaper1,shaperL1] = ZeroVibrationInputShaping(shaper_order,1/shaper_freq1);
[shaper2,shaperL2] = ZeroVibrationInputShaping(shaper_order,1/shaper_freq2);
[shaper3,shaperL3] = ZeroVibrationInputShaping(shaper_order-1,1/shaper_freq1);
k=0;
for f=0:0.01:shaper_freq2*3
    k = k+1;
    ff(k,1) = f;
    v(k,1) = VibrtionResidual(shaper1,shaperL1,2*pi*f,0);
    v(k,2) = VibrtionResidual(shaper2,shaperL2,2*pi*f,0);
    v(k,3) = VibrtionResidual(shaper3,shaperL3,2*pi*f,0);
 
end
plot(ff,v(:,1),ff,v(:,2),ff,v(:,3))