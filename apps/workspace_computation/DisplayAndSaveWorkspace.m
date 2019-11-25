function f = DisplayAndSaveWorkspace(out,mode,folder,rec)

switch mode
    case 0 %max tension
        criteria = max(out.tension_vector);
        filename = strcat(folder,'/workspace_files/',rec.figure_handle.Name,'_max_tension');
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
    case 1 %min tension
        criteria = min(out.tension_vector);
        filename = strcat(folder,'/workspace_files/',rec.figure_handle.Name,'_min_tension');
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
    case 2 %orientation sensitivity
        name = rec.figure_handle.Name;
        f = figure;
        f.Name = strcat(name,',z=',num2str(out.pose(3,1)));
        pax = polaraxes;
        polaraxes(pax);
        criteria = out.manip;
        [~,criteria_ind] = sort(criteria);
        th = out.pose(4,:);
        r = out.pose(5,:)*180/pi;
        cmap = jet(length(criteria));
        polarscatter(th(criteria_ind),r(criteria_ind),10,cmap,'filled')
        h1=colorbar;
        colormap(cmap)
        caxis([min(criteria) max(criteria)])
        pax.FontWeight = 'bold';
        pax.RTick = [20 40 60 80];
        pax.Layer = 'top';
        pax.GridAlpha = 1;
        pax.RAxisLocation = 10;
        filename = strcat(folder,'/workspace_files/',f.Name,'_manip');
        savefig(strcat(filename,'.fig'));
        ws_data = out;
        ws_data = rmfield(ws_data,'pose');
        save(strcat(filename,'.mat'),'ws_data');
        close(f);
end

end