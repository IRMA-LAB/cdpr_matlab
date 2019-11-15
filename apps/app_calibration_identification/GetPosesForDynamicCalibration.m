function data = GetPosesForDynamicCalibration(data,dir)

in = load(strcat(dir,'dl'));
data.dl = in.dl;
in = load(strcat(dir,'ds'));
data.ds = in.ds;
for i=0:length(data.dl)-1
   c = strcat('pose',int2str(i));
   in = load(strcat(dir,'pose',int2str(i)));
   data.(c) = in.pose;
   c = strcat('pose_d',int2str(i));
   in = load(strcat(dir,'pose_d',int2str(i)));
   data.(c) = in.pose_d;
   c = strcat('pose_dd',int2str(i));
   in = load(strcat(dir,'pose_dd',int2str(i)));
   data.(c) = in.pose_dd;
end

end