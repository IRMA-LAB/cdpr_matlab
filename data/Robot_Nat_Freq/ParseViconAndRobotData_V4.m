clear all
close all
clc

ms = MultiStart('UseParallel',true,'Display','iter');

dest_dir = 'ParsedData';
org_file_name = 'v_nat_f_';
dest_file_name = 'v_points_nat_f_';
dest_file_name_f = 'v_nat_f_';
dest_file_name_idx = 'v_idx_nat_f_';
dest_file_name_fig = 'v_points_comp_nat_f_';
cd ./v_nat_f_26052020

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
        data_analysis(j).p(:,k)=points(j).p(:,k)-sum;
        pointer = pointer+1;
       end
   end
   save([dest_dir '/' dest_file_name num2str(i) '.mat'], 'points');
   
   %PlotFreq(data_analysis)
   if (i<37)
        options = optimoptions('lsqnonlin','SpecifyObjectiveGradient',true,'MaxIterations',200,'MaxFunctionEvaluations',200,'OptimalityTolerance',1e-8,'FunctionTolerance',1e-8);
        n_f = 2;
   elseif(i<49)
        options = optimoptions('lsqnonlin','SpecifyObjectiveGradient',true,'MaxIterations',500,'MaxFunctionEvaluations',500,'OptimalityTolerance',1e-8,'FunctionTolerance',1e-8);
        n_f = 3;
   else
        options = optimoptions('lsqnonlin','SpecifyObjectiveGradient',true,'MaxIterations',1000,'MaxFunctionEvaluations',1000,'OptimalityTolerance',1e-8,'FunctionTolerance',1e-8);
        n_f = 4;
   end
    sol0 = [];
    f_val0 = [];
    v0_single_coord = [1;0.2;1;0];
    lb_single_coord =[-30;0;0;0];
    ub_single_coord = [30;1;6;2*pi];
    for jj = 1:n_coordinates 
        for ii = 1:n_points
            lb = []; ub = []; v0 = [];
            for kk=1:n_f
                lb = [lb ;lb_single_coord]; 
                ub = [ub ;ub_single_coord];
                v0 = [v0 ;v0_single_coord(1)/kk;v0_single_coord(2)*kk;v0_single_coord(3)*kk;v0_single_coord(4)];
            end
            problem = createOptimProblem('lsqnonlin',...
            'objective',@(x)IndetifyF_V2_J_single(x,data_analysis(ii).p(:,jj),n_f),'lb',lb,'ub',ub,...
            'x0',v0,'options',options);   
            [x,f_val,exitflag,output,solutions] = run(ms,problem,20);
            freq = [];
            for kk=1:n_f
                freq = [freq;x((kk-1)*4+3)];
            end
            [~,idx] = sort(freq);
            x_n = x;
            for kk=1:n_f
                x_n((kk-1)*4+1:(kk-1)*4+4) = x((idx(kk)-1)*4+1:(idx(kk)-1)*4+4);
            end
            sol0 = [sol0 x_n];
            f_val0 = [f_val0 f_val];
        end
    end
    [f_val_sorted,idx] = sort(f_val0);
    sol_sorted = sol0(:,idx);
    freq_sorted = [];
    [row col] = size(sol_sorted);
    for j=1:col
        for kk=1:n_f
                fff(kk,1) =  sol_sorted((kk-1)*4+3,j);
        end
        freq_sorted = [freq_sorted fff];
    end
    
    save([dest_dir '/' dest_file_name_idx num2str(i) '.mat'], 'idx');    
    save([dest_dir '/' dest_file_name_f num2str(i) '.mat'], 'freq_sorted');
    figg = ComputePlot_V2_single(sol0,data_analysis,n_f,n_points,n_coordinates);
    savefig(figg,[dest_dir '/' dest_file_name_fig num2str(i) '.fig']);

    clear points
    clear data_analysis
    clear figg
    close all
   
end

cd ../