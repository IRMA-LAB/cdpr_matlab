classdef RecordType
    properties
        n_cables;
        figure_handle;
        axes_handle;
        lines;
        frame_vector;
    end
    methods
        function obj = RecordType(par,title,name)
            obj.n_cables = par.n_cables;
            obj = obj.SetFigureParameter(par,title,name);
            for i=1:obj.n_cables
                obj.lines.pulley(i) = animatedline('Color','r','LineWidth',2);
                obj.lines.cable(i) = animatedline('Color','k','LineWidth',2);
                obj.lines.platformPoint(i) = animatedline('Color','b','LineWidth',1);
                obj.lines.platformEdge(i) = animatedline('Color','b','LineWidth',1);
                obj.lines.platformPlane(i) = animatedline('Color','b','LineWidth',1);
            end
        end
        function obj = SetFigureParameter(obj,par,title,name)
            obj.figure_handle = figure('Name',title,'FileName',name,'NumberTitle','off','Position',[0 0 700 700]);
            for i=1:par.n_cables
                if (norm(par.cable(i,1).pos_D_glob)>norm(par.cable(i,1).pos_A_loc))
                    limits(:,i) = par.cable(i,1).pos_D_glob;
                else
                    limits(:,i) = par.cable(i,1).pos_A_loc;
                end
            end
            hold on
            max_x = max(limits(1,:)); min_x = min(limits(1,:)); incr_x = 0.2*(max_x - min_x);
            lim_x = [((min_x-incr_x)*10) ((max_x+incr_x)*10)]./10; l_x = (lim_x(2)-lim_x(1)); tickSpace_x = l_x/10;
            max_y = max(limits(2,:)); min_y = min(limits(2,:)); incr_y = 0.2*(max_y - min_y);
            lim_y = [((min_y-incr_y)*10) ((max_y+incr_y)*10)]./10; l_y = (lim_y(2)-lim_y(1)); tickSpace_y = l_y/10;
            max_z = max(limits(3,:)); min_z = min(limits(3,:)); middle_z = (max_z - min_z)/2;
            max_l = max([lim_x lim_y]); lim_z = [((middle_z-max_l)*15) (abs(max_z)*15)]./10; l_z = (lim_z(2)-lim_z(1));
            tickSpace_z = l_z/10;
            
            obj.axes_handle = gca; obj.axes_handle.Box = 'Off'; obj.axes_handle.LineWidth = 2;
            obj.axes_handle.XAxisLocation = 'origin'; obj.axes_handle.XLim = lim_x;
            obj.axes_handle.XTick = lim_x(1):tickSpace_x:lim_x(2); obj.axes_handle.XGrid = 'On';
            obj.axes_handle.YAxisLocation = 'origin'; obj.axes_handle.ZLim = lim_z;
            obj.axes_handle.ZTick = lim_z(1):tickSpace_z:lim_z(2);   obj.axes_handle.YGrid = 'On';
            obj.axes_handle.YLim = lim_y; obj.axes_handle.YTick = lim_y(1):tickSpace_y:lim_y(2);
            obj.axes_handle.ZGrid = 'On'; obj.axes_handle.GridLineStyle = '--';
            
            obj.axes_handle.XLabel.String = 'x'; obj.axes_handle.XLabel.FontSize = 16;
            obj.axes_handle.XLabel.FontWeight = 'bold';
            obj.axes_handle.YLabel.String = 'y'; obj.axes_handle.YLabel.FontSize = 16;
            obj.axes_handle.YLabel.Rotation = 0; obj.axes_handle.YLabel.FontWeight = 'bold';
            obj.axes_handle.ZLabel.String = 'z'; obj.axes_handle.ZLabel.FontSize = 16;
            obj.axes_handle.ZLabel.Rotation = 0; obj.axes_handle.ZLabel.FontWeight = 'bold';
            
            obj.axes_handle.DataAspectRatioMode = 'manual';
            obj.axes_handle.DataAspectRatio= [1;1;1];
            
            obj.axes_handle.CameraPosition = [-2.837916634960155,-22.911510361523757,13.494156116891816];
            %obj.axes_handle.CameraTarget = [-0.913362548387552,0.616539044482742,-0.747766910811659];
            %obj.axes_handle.CameraViewAngle = 9.3176;
        end
        function obj = ResetFigureLimits(obj,limits,spacing)
            
            obj.axes_handle.XLim = limits(1,:);
            obj.axes_handle.YLim = limits(2,:);
            obj.axes_handle.ZLim = limits(3,:);
            obj.axes_handle.XTick = limits(1,1):(limits(1,2)-limits(1,1))/spacing:limits(1,2); 
            obj.axes_handle.YTick = limits(2,1):(limits(2,2)-limits(2,1))/spacing:limits(2,2); 
            obj.axes_handle.ZTick = limits(3,1):(limits(3,2)-limits(3,1))/spacing:limits(3,2);   
            
            obj.axes_handle.DataAspectRatioMode = 'manual';
            obj.axes_handle.DataAspectRatio= [1;1;1];
            
            obj.axes_handle.CameraPosition = [-2.837916634960155,-22.911510361523757,13.494156116891816];
        
        end
        function frame = SetFrame(obj,cdpr_v,cdpr_p)
            for i=1:obj.n_cables
                clearpoints(obj.lines.pulley(i));
                clearpoints(obj.lines.cable(i));
                clearpoints(obj.lines.platformPoint(i));
                clearpoints(obj.lines.platformEdge(i));
                clearpoints(obj.lines.platformPlane(i));
                k = cdpr_v.platform.rot_mat(:,3);
                r = cdpr_v.platform.position;
                a = cdpr_v.cable(i).pos_OA_glob;
                p(:,i) = RecordType.GetIntersectionInPlane(r,a,k);
                for j = 0:0.05:2*pi+0.05
                    vers_n_rot = cdpr_v.cable(i).vers_u*cos(j) +...
                        cdpr_p.cable(i).vers_k*sin(j);
                    r = cdpr_p.cable(i).pos_D_glob+cdpr_p.cable(i).swivel_pulley_r*...
                        (cdpr_v.cable(i).vers_u+vers_n_rot);
                    addpoints(obj.lines.pulley(i),[r(1)],[r(2)],[r(3)]);
                end
                addpoints(obj.lines.platformEdge(i),...
                    [cdpr_v.cable(i).pos_OA_glob(1) cdpr_v.platform.position(1)],...
                    [cdpr_v.cable(i).pos_OA_glob(2) cdpr_v.platform.position(2)],...
                    [cdpr_v.cable(i).pos_OA_glob(3) cdpr_v.platform.position(3)]);
                addpoints(obj.lines.platformPoint(i),...
                    [p(1,i) cdpr_v.platform.position(1)],...
                    [p(2,i) cdpr_v.platform.position(2)],...
                    [p(3,i) cdpr_v.platform.position(3)]);
                addpoints(obj.lines.platformPlane(i),...
                    [cdpr_v.cable(i).pos_OA_glob(1) p(1,i)],...
                    [cdpr_v.cable(i).pos_OA_glob(2) p(2,i)],...
                    [cdpr_v.cable(i).pos_OA_glob(3) p(3,i)]);
                r = cdpr_p.cable(i).pos_D_glob+cdpr_p.cable(i).swivel_pulley_r*...
                    (cdpr_v.cable(i).vers_u+cdpr_v.cable(i).vers_n);
                addpoints(obj.lines.cable(i),...
                    [cdpr_v.cable(i).pos_OA_glob(1) r(1,1)],...
                    [cdpr_v.cable(i).pos_OA_glob(2) r(2,1)],...
                    [cdpr_v.cable(i).pos_OA_glob(3) r(3,1)]);
                frame = getframe(obj.figure_handle);
            end
        end
    end
    methods (Static)
        function p = GetIntersectionInPlane(r,a,k)
            A = zeros(3);
            v = zeros(3,1);
            A(3,:) = k';
            v(3) =  k'*a;
            [~,index] = max(abs(k));
            switch index
                case 1
                    A(1:2,:) = [k(2) -k(1) 0;
                        k(3) 0 -k(1)];
                    v(1:2,1) = [-k(1)*r(2)+k(2)*r(1);-k(1)*r(3)+k(3)*r(1)];
                case 2
                    A(1:2,:) = [k(2) -k(1) 0;
                        0 k(3) -k(2)];
                    v(1:2,1) = [-k(1)*r(2)+k(2)*r(1);-k(2)*r(3)+k(3)*r(2)];
                case 3
                    A(1:2,:) = [k(3) 0 -k(1) ;
                        0 k(3) -k(2)];
                    v(1:2,1) = [-k(1)*r(3)+k(3)*r(1);-k(2)*r(3)+k(3)*r(2)];
            end
            p = linsolve(A,v);
        end
    end
end