clear all
close all
clc

dest_dir = 'ParsedDataPose';
org_file_name = 'v_nat_f_';
dest_file_name = 'v_points_nat_f_';
dest_file_name_pose = 'v_pose_nat_f_';
cd ./v_nat_f_26052020

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

for i=1:l
    
   file_name_act = [org_file_name num2str(i) '.txt'] ;
   cond = 1;
   additional_lines = 800;
   data = readmatrix(file_name_act,'NumHeaderLines',lines_to_skip+additional_lines);
   pointer = 0;
   for j=1:n_points
       for k=1:n_coordinates
        t = (1:length(data(:,start_col+pointer))).*0.01-0.01;
        %points(j).p(:,k) = data(end/3:end,start_col+pointer);
        points(j).p(:,k) = data(:,start_col+pointer);
        jj = 0;
        for ii=1:length(t)
            if(~isnan(points(j).p(ii,k)))
                jj = jj+1;
                val(jj) =  points(j).p(ii,k);
                t_val(jj) = t(ii);
            end
        end
        
        try
        s = spline(t_val,val);
        for ii=1:length(t)
            points(j).p(ii,k) = ppval(s,t(ii));
        end
        p_mean = mean(points(j).p(:,k));
        points_filt(j).p(:,k) = p_mean+filtfilt(d_filt,points(j).p(:,k)-p_mean);
        %ppp = p_mean+filtfilt(d_filt,points(j).p(:,k)-p_mean);
        %ppp(1:100) = []; ppp(end-100:end) = [];
        %points_filt(j).p(:,k) = ppp
        plot(t,points(j).p(:,k),t,points_filt(j).p(:,k))
        end
        PlotFreq(points_filt(j).p(:,k));
        pointer = pointer+1;
        clear t_val
        clear val
       end
   end
   clear p_mean
   ref_p = (points_filt(1).p)';
   p_mean = (points_filt(2).p/4)';
   for jj=3:5
      p_mean = p_mean+  (points_filt(jj).p./4)';
   end
   p_mean_y = (points_filt(2).p/2)'+(points_filt(3).p/2)'-p_mean;
   for jj=1:length(ref_p)
      n_ref_n = p_mean(:,jj)-ref_p(:,jj);
      n_ref_n = n_ref_n./(norm(n_ref_n));
      ref_n(:,jj) = n_ref_n;
      A = [];
      for ii=1:4
          A(ii,:) = points_filt(ii+1).p(jj,:);
      end
      nn = linsolve(A,-ones(4,1));
      nn_v = nn./norm(nn);
      ref_x_n = cross(p_mean_y(:,jj),nn_v);
      ref_x = ref_x_n./norm(ref_x_n);
      ref_y = cross(nn_v,ref_x);
      R = [ref_x,ref_y,nn_v];
      for ii=1:4
         marker(ii).p(:,jj) =  R'*((points_filt(ii+1).p(jj,:))'-ref_p(:,jj));
      end 
   end
   for ii=1:4
         marker_rec(ii).p = mean(marker(ii).p,2);
      end
   save([dest_dir '/' dest_file_name num2str(i) '.mat'], 'points');
   clear points
   clear points_filt
   
   
end