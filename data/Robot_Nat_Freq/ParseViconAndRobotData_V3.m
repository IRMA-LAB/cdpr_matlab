clear all
close all
clc

options = optimoptions('lsqnonlin','MaxIterations',1000,'MaxFunctionEvaluations',10000,'OptimalityTolerance',1e-8,'FunctionTolerance',1e-8,'StepTolerance',1e-7);
ms = MultiStart('UseParallel',true,'Display','iter');

dest_dir = 'ParsedData';
org_file_name = 'v_nat_f_';
dest_file_name = 'v_points_nat_f_';

cd ./v_nat_f_26052020

[~,~,~] = mkdir(dest_dir);
dt = 0.01;
n_points=5;
n_coordinates = 3;
lines_to_skip = 5;
start_col = 3;

a=dir('*.txt');
l = length(a);

for i=37:l
   p0 = []; 
   file_name_act = [org_file_name num2str(i) '.txt'] ;
   cond = 1;
   additional_lines = 0;
   while cond
       data = readmatrix(file_name_act,'NumHeaderLines',lines_to_skip+additional_lines);
       [row,col] = size(data);
       if col~=1
           cond = 0;
       else
           ind = 1;
           while (~isnan(data(ind)))
               ind = ind+1;
           end
           additional_lines = additional_lines+ind-1;
       end
   end
   pointer = 0;
   for j=1:n_points
       for k=1:n_coordinates
        points(j).p(:,k) = data(end/2:end,start_col+pointer);
        sum = 0;
        idx = 0;
        for ii=1:length(points(j).p(:,k))
            if(~isnan(points(j).p(ii,k)))
                sum = sum+points(j).p(ii,k);
                idx = idx+1;
            end
        end
        sum = sum/idx;
        p0= [p0;sum];
        data_analysis(j).p(:,k)=points(j).p(:,k);
        pointer = pointer+1;
       end
   end
   save([dest_dir '/' dest_file_name num2str(i) '.mat'], 'points');
   
   %PlotFreq(data_analysis)
   if (i<37)
        n_f = 2;
   elseif(i<49)
        n_f = 3;
   else
       n_f = 4;
   end
    v0_single = [ones(n_coordinates.*n_points,1);0.1;1;0];
    lb_single =[-30.*ones(n_coordinates.*n_points,1);0;0;0;];
    ub_single = [30.*ones(n_coordinates.*n_points,1);1;10;2*pi;];
    lb = []; ub = []; v0 = [];
    for ii=1:n_f
       lb = [lb ;lb_single]; 
       ub = [ub ;ub_single];
       v0 = [v0 ;v0_single.*ii];
    end
    v0 = [v0;p0];
    lb = [lb; p0-abs(p0)];
    ub = [ub; p0+abs(p0)];
    problem = createOptimProblem('lsqnonlin',...
    'objective',@(x)IndetifyF_V3(x,data_analysis,n_f,n_points,n_coordinates),'lb',lb,'ub',ub,...
    'x0',v0,'options',options);   
    [x,ff] = run(ms,problem,15);
    
    ComputePlot_V3(x,data_analysis,n_f,n_points,n_coordinates);
    clear points
    clear data_analysis
   
end

cd ../