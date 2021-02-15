classdef OverActuatedPar
    %PLATFORMPARAMETERS is a class containing static parameters of the platform.
    %
    %   PLATFORMPARAMETERS links the parameter values of the platform from an
    %   input structure to the homonymous object's properties.
    %   Parameters consist of inertial properties of the platform and the
    %   wrench components acting on it. The wrench is evaluated wrt the center
    %   of mass G.
    %
    properties
        
        n_cables;
        pose_dim;
        
    end
    methods
        function obj = OverActuatedPar(n,n_p)
            obj.n_cables = n;
            obj.pose_dim = n_p;
        end
        
    end
end