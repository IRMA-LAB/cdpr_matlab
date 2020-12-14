clear all
close all
clc

addpath('../../libs/orientation_geometry')

dest_dir = 'ParsedDataPose';
org_file_name = 'v_nat_f_';
dest_file_name = 'v_pose_nat_f_';
dest_file_name_pose = 'v_pose_nat_f_';
cd ./v_nat_f_26052020
load marker_pos_rel_to_ref


options = optimoptions('lsqnonlin','MaxIterations',10000,'MaxFunctionEvaluations',10000,'OptimalityTolerance',1e-8,'FunctionTolerance',1e-8,'UseParallel',true);
options = optimoptions2('lsqnonlin','MaxIterations',10000,'MaxFunctionEvaluations',10000,'OptimalityTolerance',1e-8,'FunctionTolerance',1e-8,'UseParallel',true);
d_filt = designfilt('lowpassfir', ...
    'FilterOrder', 50,'PassBandFrequency', 4, 'StopBandFrequency',10,...
    'DesignMethod','equiripple','SampleRate',100);

[~,~,~] = mkdir(dest_dir);

dt = 0.01;
n_points=5;
n_coordinates = 3;
lines_to_skip = 5;
start_col = 3;
a=dir('*.txt');
l = length(a);

for i=24:l
    
   file_name_act = [org_file_name num2str(i) '.txt'] ;
   cond = 1;
   additional_lines = 800;
   data = readmatrix(file_name_act,'NumHeaderLines',lines_to_skip+additional_lines);
   pointer = 0;
   for j=1:n_points
       for k=1:n_coordinates
        t = (1:length(data(:,start_col+pointer))).*0.01-0.01;
        points(j).p(:,k) = data(:,start_col+pointer);
        pointer = pointer+1;
        clear t_val
        clear val
       end
   end
   for j=1:n_points
       if(~isnan(mean(points(j).p(1,:))'))
           p0 = (points(j).p(1,:))';
           break
       end
   end    
   p0 = [p0;0;0;0];
   time = [];
   tt = 0;
   idx = 0;
   pose = [];
   mean(points(1).p(:,:),1)
   for j = 1:length(points(1).p(:,1))
       points_val = [];
       for k=1:n_points
          points_val = [points_val, (points(k).p(j,:))'];
       end
       try
            p0 = lsqnonlin(@(x)ReconstructPoseFun(points_val,marker_rec,x),p0,[-3000;-3000;-3000;-2*pi;-2*pi;-2*pi],[3000;3000;3000;2*pi;2*pi;2*pi],options);
            idx = idx+1;
            pose = [pose, p0];
            time = [time, tt];
       end
       tt = tt+0.01;
   end
   
   for j=1:6
      s = spline(time,pose(j,:)); 
      for tt = 1:length(t)
         pose_s(j,tt) = ppval(s,t(tt)); 
      end
      pose_f(j,:) = mean(pose_s(j,:))+filtfilt(d_filt,pose_s(j,:)-mean(pose_s(j,:)));
      pose_f_d(j,:) = diff(pose_f(j,:))/0.01;   % first derivative
      pose_f_dd(j,:) = diff(pose_f_d(j,:))/0.01; 
   end
   
   dimm = length(pose_f_dd);
   pose_f(:,dimm+1:end) = [];
   pose_f_d(:,dimm+1:end) = [];
   
   pose_print = [pose_f; pose_f_d; pose_f_dd]; 
   save([dest_dir '/' dest_file_name num2str(i) '.mat'], 'pose_print');
   clear points
   clear pose_f
   clear pose_f_d
   clear pose_f_dd
   clear t
   clear pose_print
   clear pose
   clear time
   
end