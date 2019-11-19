function f = DisplayAndSaveWorkspace(out,mode,folder,rec)

switch mode
    case 0 %max tension
        criteria = max(out.tension_vector);
        filename = strcat(folder,'/workspace_files/',rec.figure_handle.Name,'_max_tension');
    case 1 %min tension
        criteria = min(out.tension_vector);
        filename = strcat(folder,'/workspace_files/',rec.figure_handle.Name,'_min_tension');
end
[~,criteria_ind] = sort(criteria);
x = out.pose(1,:);
y = out.pose(2,:);
z = out.pose(3,:);
cmap = jet(length(criteria));
f = scatter3(x(criteria_ind),y(criteria_ind),z(criteria_ind),10,cmap,'filled');
savefig(strcat(filename,'.fig'));
ws_data = out;
ws_data = rmfield(ws_data,'pose');
save(strcat(filename,'.mat'),'ws_data');
end