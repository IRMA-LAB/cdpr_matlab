function [p,dp,ddp] = CalcLinearizedConfiguration(data,t)


p_mat = data.normal_modes.*data.amplitude';
dp_mat = -data.normal_modes.*(2.*pi.*(data.nat_freq.*data.amplitude)');
ddp_mat = -data.normal_modes.*(4.*(pi^2).*(data.nat_freq.*data.nat_freq.*data.amplitude)');

p_v = cos(2.*pi.*data.nat_freq.*t-data.phase);
dp_v = sin(2.*pi.*data.nat_freq.*t-data.phase);
ddp_v = cos(2.*pi.*data.nat_freq.*t-data.phase);

p = 180/pi.*(p_mat*p_v+data.ang_par);
dp = 180/pi.*dp_mat*dp_v;
ddp = 180/pi.*ddp_mat*ddp_v;

end