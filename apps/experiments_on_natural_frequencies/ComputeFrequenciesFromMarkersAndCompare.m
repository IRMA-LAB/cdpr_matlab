clear all
close all
clc

dest_dir = 'EE_pose_vicon';
dest_fig_name = 'New_Experiment_';
org_dir = 'EE_pose_vicon';
marker_file_name = 'vicon_markers_';
freq_file_name = 'vicon_data_f_';
% dest_file_name = 'vicon_pose_';
% dest_file_name2 = 'vicon_markers_';
l = zeros(108,4);
freq = l;
for i=1:108
   try
       load(strcat(org_dir,'/',freq_file_name,num2str(i),'.mat'));
       pose(i,:) = res.pose';
       l(i,1:length(res.l)) = res.l';
       freq(i,1:length(res.f)) = res.f';
       
   end
end

for i=1:108
    try
        load(strcat(org_dir,'/',marker_file_name,num2str(i),'.mat'));
        load(strcat(org_dir,'/',freq_file_name,num2str(i),'.mat'));
        
        pks = [];
        locs = [];
        f = figure();
        a = gca;
        hold on
        grid on
        lgd = legend();
        lgd.Interpreter = 'latex';
        
        for j=1:5
            switch j
                case 1
                    col_line = 'b';
                    name = 'marker n. 1';
                case 2
                    col_line = 'r';
                    name = 'marker n. 2';
                case 3
                    col_line = 'm';
                    name = 'marker n. 3';
                case 4
                    col_line = 'k';
                    name = 'marker n. 4';
                case 5
                    col_line = 'g';
                    name = 'marker n. 5';
            end
            for k=1:3
                switch k
                    case 1
                        col_line_d = [col_line '-'];
                        name_d = [name ' coord. x'];
                    case 2
                        col_line_d = [col_line '--'];
                        name_d = [name ' coord. y'];
                    case 3
                        col_line_d = [col_line ':'];
                        name_d = [name ' coord. z'];
                end
                [pp,ll] = PlotFreq(m(j).p(k,:)',col_line_d,name_d);
                pks = [pks pp];
                locs = [locs ll'];
            end
        end
        y_f = a.YLim;
        for j=1:length(res.f)
            x_f = [res.f(j) res.f(j)];
            plot(x_f,y_f,'o-k')
            
            lims = [res.f(j)-7*res.f(j)/100 res.f(j)+7*res.f(j)/100];
            f_locs = find(locs < lims(2) & locs > lims(1));
            [A,idx] = sort(pks(f_locs),'descend');
            f_star = locs(f_locs);
            f_star = f_star(idx);
            if length(A)>15
            A(16:end) = [];
            f_star(16:end) = [];
        end
        f_mean = 0;
        std = 0;
        tot = sum(A);
        for idx=1:length(A)
           f_mean = f_mean+A(idx)*f_star(idx)/tot; 
        end
        for idx=1:length(A)
           std = std+A(idx)*(f_star(idx)-f_mean)^2/tot; 
        end
        std = sqrt(std);
        f_exp(i,j) = f_mean;
        f_std(i,j) = std;
        end
        savefig(strcat(dest_dir,'/',dest_fig_name,num2str(i),'.fig'));
        close(f)
    end
end

xlswrite(strcat(dest_dir,'/','f_exp_mean.xlsx'),f_exp);
xlswrite(strcat(dest_dir,'/','f_exp_std.xlsx'),f_std);

