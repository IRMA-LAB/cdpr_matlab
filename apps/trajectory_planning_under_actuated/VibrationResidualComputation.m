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

shaper_order = 2;
shaper_period1 = 2;
shaper_period2= 0.5;
shaper_period3= 0.1;
[shaper1,shaperL1] = ZeroVibrationInputShaping(shaper_order,shaper_period1);
[shaper2,shaperL2] = ZeroVibrationInputShaping(shaper_order,shaper_period2);
[shaper3,shaperL3] = ZeroVibrationInputShaping(shaper_order,shaper_period3);
k=0;
for ww=0:0.1:20
    k = k+1;
    w(k,1) = ww/(2*pi);
    v(k,1) = VibrtionResidual(shaper1,shaperL1,ww,0);
    v(k,2) = VibrtionResidual(shaper2,shaperL2,ww,0);
    v(k,3) = VibrtionResidual(shaper3,shaperL3,ww,0);
 
end
plot(w,v(:,1),w,v(:,1).*v(:,2),w,v(:,1).*v(:,2).*(v(:,3)))