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
        
        permutation_matrix;
        n_cables;
        pose_dim;
        
    end
    methods
        function obj = UnderActuatedPar(n,n_p,mask)
            obj.n_cables = n;
            obj.pose_dim = n_p;
            obj.permutation_matrix = zeros(6);
            for i=1:6
               obj.permutation_matrix(i,(mask(i))) = 1; 
            end
        end
        
        function obj = SetMask(obj,mask)
            % CDPRVAR instantiates an object of CableParameters type.
            % CDPRVAR defines the object CABLE containing an object for each
            % cable that stores time dependent variables of the cable and its
            % swivel pulley.
            obj.permutation_matrix = zeros(6);
            for i=1:6
               obj.permutation_matrix(i,(mask(i))) = 1; 
            end
        end
        
    end
end