classdef UnderActuatedPar
    %PLATFORMPARAMETERS is a class containing static parameters of the platform.
    %
    %   PLATFORMPARAMETERS links the parameter values of the platform from an
    %   input structure to the homonymous object's properties.
    %   Parameters consist of inertial properties of the platform and the
    %   wrench components acting on it. The wrench is evaluated wrt the center
    %   of mass G.
    %
    properties
        
        actuated_mask;
        unactuated_mask;
        n_cables;
        pose_dim;
        
    end
    methods
        function obj = UnderActuatedPar(n,n_p)
            obj.n_cables = n;
            obj.pose_dim = n_p;
            obj.actuated_mask = zeros(n,1);
            obj.unactuated_mask = zeros(n_p-n,1);
            for i=1:n
                obj.actuated_mask(i) = i;
            end
            for i=n+1:n_p
                obj.unactuated_mask(i-n) = i;
            end
        end
        
        function obj = SetMask(obj,mask)
            % CDPRVAR instantiates an object of CableParameters type.
            % CDPRVAR defines the object CABLE containing an object for each
            % cable that stores time dependent variables of the cable and its
            % swivel pulley.
            act_count = 0;
            unact_count = 0;
            for i=1:obj.pose_dim
                if (mask(i)==1)
                    act_count = act_count+1;
                    obj.actuated_mask(act_count) = i;
                else
                    unact_count = unact_count+1;
                    obj.unactuated_mask(unact_count) = i;
                end
            end
        end
        
    end
end