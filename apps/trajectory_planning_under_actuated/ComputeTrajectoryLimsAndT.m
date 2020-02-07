function data = ComputeTrajectoryLimsAndT(data,f,i)

%v = fmincon(@(x)TrapzAccRespOptFun(x,f(2),2),1/3);
% if(median(f)/min(f)<2)
    data.lims(:,i) = (min(f)/(median(f)+min(f))).*[1;1];
    data.dt(i) = (median(f)+min(f))/(median(f)*min(f));
    data.lims1(:,i) = (min(f)/(median(f)+min(f))).*[1;1];
    data.dt1(i) = (median(f)+min(f))/(median(f)*min(f))/2;
% else
%     data.lims(:,i) = (min(f)/(max(f)+min(f))).*[1;1];
%     data.dt(i) = (max(f)+min(f))/(max(f)*min(f));
% end
% a=[];T=[];
% for i=1:1
% for c = 1:5
%         for b=1:5
%             v = fmincon(@(x)TrapzAccRespOptFun(x,f),[0.25;1/(2*min(f))],[],[],[],[],[0;0],[0.5;1/min(f)]);
%             if ( ~any(v<0.001))
%                 v = sort(v);
%                 a = [a;v(1)/sum(v)]; T = [T;sum(v)/pi];
%             end
%         end
% end
% end
% [t,ind] = min(T);
% lim = a(ind);
% v = lsqnonlin(@(x)TrapzAccRespOptFun(x,[f(1);f(2)]),[0.25;mean(f)],[0.2;min(f)],[0.5;10*max(f)]);
% a=v(1).*[1;1];
% T=1/v(2);
% data.lims(:,i) = a.*[1;1];
% data.dt(i) = T;
end