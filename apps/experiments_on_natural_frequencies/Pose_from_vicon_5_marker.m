clear all
close all
clc

addpath('../../libs/orientation_geometry')

dest_dir = 'EE_pose_vicon';
org_dir = 'exp_06102020';
org_file_name = 'v_data_';
dest_file_name = 'vicon_pose_';
dest_file_name2 = 'vicon_markers_';

load Marker_Data/marker_pos_rel_to_ref

options = optimoptions('lsqnonlin','MaxIterations',10000,'MaxFunctionEvaluations',10000,'OptimalityTolerance',1e-8,'FunctionTolerance',1e-8,'UseParallel',true);
options2 = optimoptions('lsqnonlin','MaxIterations',10000,'MaxFunctionEvaluations',10000,'OptimalityTolerance',1e-8,'FunctionTolerance',1e-8,'UseParallel',true,'SpecifyObjectiveGradient',true,'Display','none');

d_filt = designfilt('lowpassfir', ...
    'FilterOrder', 50,'PassBandFrequency', 4, 'StopBandFrequency',10,...
    'DesignMethod','equiripple','SampleRate',100);

[~,~,~] = mkdir(dest_dir);

dt = 0.01;
n_points=5;
n_coordinates = 3;
header_lengths = 5;
garbage_length = 0;
lines_to_skip = header_lengths+garbage_length;
start_col = 3;

cd(org_dir);

a=dir('*.txt');
l = length(a);
cd ..


for i=1:l
   
   % Read File 
   file_name_act = [org_dir '/' org_file_name num2str(i) '.txt'] ;
   cond = 1;
   data = readmatrix(file_name_act,'NumHeaderLines',lines_to_skip);
   
   [rows,cols] = size(data);
   if cols==17
       data = data(800:800+1102,:);
   pointer = 0;
   
   % Extract marker data from matrix into structure
   for j=1:n_points
       for k=1:n_coordinates
        t = (1:length(data(:,start_col+pointer))).*0.01-0.01;
        points(j).p(:,k) = data(:,start_col+pointer);
        pointer = pointer+1;
       end
   end
   
   % Get the first good configuration where all the markers are well seen
   
   for j=1:length(data)
      isnan_f = 0;
      for jj=1:n_points
       if(isnan(mean(points(jj).p(j,:))'))
           isnan_f = 1;
           break
       end 
      end
      if(isnan_f==0)
         p0 = (points(1).p(j,:))'; 
         break
      end
   end
   if (isnan_f ==0)
%    for j=1:n_points
%        if(~isnan(mean(points(j).p(1,:))'))
%            p0 = (points(j).p(1,:))';
%            break
%        end
%    end
   p0 = [p0;0;0;0];
   time = [];
   
   % Reconstruct platform pose from 5 markers: if markers are not seen,
   % they are not included in reconstruction
   tt = 0;
   idx = 0;
   pose = [];
   for j = 1:length(points(1).p(:,1))
       points_val = [];
       for k=1:n_points
          points_val = [points_val, (points(k).p(j,:))'];
       end
       try
            p0 = lsqnonlin(@(x)ReconstructPoseFun(points_val,marker_rec,x),...
                p0,[-3000;-3000;-3000;-2*pi;-2*pi;-2*pi],[3000;3000;3000;2*pi;2*pi;2*pi],options2);
         
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
   
   for j=1:(length(pose_f(1,:))-2)
          R = RotXYZ(pose_f(4:6,j));
            m(1).p(:,j) = pose_f(1:3,j);
        for k=2:5
            m(k).p(:,j) = pose_f(1:3,j)+R*marker_rec(k-1).p;
        end 
   end
   
   dimm = length(pose_f_dd);
   pose_f(:,dimm+1:end) = [];
   pose_f_d(:,dimm+1:end) = [];
   t = t(1:1101);
   
   pose_f(:,1:50) = []; 
   pose_f(:,end-49:end) = []; 
   pose_f_d(:,1:50) = []; 
   pose_f_d(:,end-49:end) = []; 
   pose_f_dd(:,1:50) = []; 
   pose_f_dd(:,end-49:end) = [];
   t(:,1:50) = []; 
   t(:,end-49:end) = [];
    for k=1:5
          m(k).p(:,1:50) = [];
          m(k).p(:,end-49:end) = [];
          m(k).p = m(k).p./1000;
    end 
    pose_f(1:3,:) = pose_f(1:3,:)./1000;
    pose_f_d(1:3,:) = pose_f_d(1:3,:)./1000;
    pose_f_dd(1:3,:) = pose_f_dd(1:3,:)./1000;
   
   clear pose
   pose = [pose_f; pose_f_d; pose_f_dd; t]; 
   save([dest_dir '/' dest_file_name num2str(i) '.mat'], 'pose');
   save([dest_dir '/' dest_file_name2 num2str(i) '.mat'], 'm');
   clear m
   clear points
   clear pose_f
   clear pose_f_d
   clear pose_f_dd
   clear t
   clear pose
   clear time
   clear p0
   clear data
   end
   end
   
end