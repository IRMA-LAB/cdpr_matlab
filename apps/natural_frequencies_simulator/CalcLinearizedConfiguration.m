function [p,dp,ddp] = CalcLinearizedConfiguration(data,t)


p_mat = data.normal_modes*diag(data.amplitude);
dp_mat = -data.normal_modes.*(2.*pi)*diag(data.amplitude)*diag(data.nat_freq);
ddp_mat = -data.normal_modes.*(4.*(pi^2))*diag(data.amplitude)*diag(data.nat_freq)*diag(data.nat_freq);

p_v = cos(2.*pi.*data.nat_freq.*t-data.phase);
dp_v = sin(2.*pi.*data.nat_freq.*t-data.phase);
ddp_v = cos(2.*pi.*data.nat_freq.*t-data.phase);

% p = 180/pi.*(p_mat*p_v+data.ang_par);
% dp = 180/pi.*dp_mat*dp_v;
% ddp = 180/pi.*ddp_mat*ddp_v;

% p = (p_mat*p_v+data.ang_par);
p = (p_mat*p_v);
dp = dp_mat*dp_v;
ddp = ddp_mat*ddp_v;

end