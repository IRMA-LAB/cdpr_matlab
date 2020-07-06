clear all
close all
clc

options = optimoptions('lsqnonlin','MaxIterations',10000,'MaxFunctionEvaluations',10000,'OptimalityTolerance',1e-8,'FunctionTolerance',1e-8);
ms = MultiStart('UseParallel',true,'Display','iter');

dest_dir = 'ParsedData';
org_file_name = 'v_nat_f_';
dest_file_name = 'v_points_nat_f_';

cd ./v_nat_f_26052020

[~,~,~] = mkdir(dest_dir);
dt = 0.01;
n_points=5;
lines_to_skip = 5;
start_col = 3;

a=dir('*.txt');
l = length(a);

for i=40:l
    
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
       for k=1:3
        points(j).p(:,k) = data(end/3:end,start_col+pointer);
        sum = 0;
        idx = 0;
        for ii=1:length(points(j).p(:,k))
            if(~isnan(points(j).p(ii,k)))
                sum = sum+points(j).p(ii,k);
                idx = idx+1;
            end
        end
        sum = sum/idx;
        data_analysis(j).p(:,k)=points(j).p(:,k)-sum;
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
    v0_single = [1;0.1;1;0];
    lb_single =[-30;0;0;0];
    ub_single = [30;1;10;2*pi];
    lb = []; ub = []; v0 = [];
    for ii=1:n_f
       lb = [lb ;lb_single]; 
       ub = [ub ;ub_single];
       v0 = [v0 ;v0_single.*ii];
    end
    problem = createOptimProblem('lsqnonlin',...
    'objective',@(x)IndetifyF(x,data_analysis(1).p(:,3),n_f),'lb',lb,'ub',ub,...
    'x0',v0,'options',options);
    
    [x,ff] = run(ms,problem,50)
   %v = lsqnonlin(@(x)IndetifyF(x,data_analysis,2),v0,lb,ub,options)
    ttt = (1:length(data_analysis(1).p(:,3))).*0.01-0.01;
    f = ComputePlot(x,data_analysis(1).p(:,3),n_f);
    plot(ttt,f,ttt,data_analysis(1).p(:,3))
  clear points
  clear data_analysis
   
end

cd ../