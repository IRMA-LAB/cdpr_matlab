function DataLoggerStruct(s,folder,name,video_flag,par,var,rec,ut)

for i=1:length(s)
    ss = s(i);
    filename_res = strcat(folder,'/planning_results/',name,num2str(i),'.mat');
    filename_vid = strcat(folder,'/videos/',name,num2str(i),'.avi');
    save(filename_res,'-struct','ss');
    MakeVideo(ss,filename_vid,video_flag,par,var,rec,ut);
end

end