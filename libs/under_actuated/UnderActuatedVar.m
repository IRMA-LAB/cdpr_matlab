classdef UnderActuatedVar
%ANDERACTUATEDVAR is a class containing time dependent variables of the regarding under-actuated cdprs.  
%   
%   ANDERACTUATEDVAR updates ... 
%   
%     
  properties
    
    pose_P;
    pose_P_d;
    pose_P_dd;
    
    free_twist;
    controlled_twist;
%     actuated;
%     unactuated;
%     
%     geometric_jacobian_a;%
%     geometric_jacobian_u;%
%     geometric_orthogonal;
%     geometric_parallel;
%     geometric_tau;
%     analitic_jacobian_a;%
%     analitic_jacobian_u;%
    analitic_jacobian_P;

    geometric_orthogonal;
    geometric_parallel;
    analitic_orthogonal;
    analitic_parallel;
    
    analitic_orthogonal_P;
    analitic_parallel_P;
    
    geometric_orthogonal_d;
    geometric_parallel_d;
    analitic_orthogonal_d;
    analitic_parallel_d;
    
    analitic_orthogonal_P_d;
    analitic_parallel_P_d;
    
    mass_matrix_q;
    C_matrix_q;
    total_load_q;

  end
  methods
      function obj = UpdateJacobians(obj,par,Ja,D)
          obj.analitic_jacobian_P = par.permutation_matrix*Ja;
          obj.analitic_orthogonal_P = [-linsolve(obj.analitic_jacobian_P(1:par.n_cables,:)',obj.analitic_jacobian_P(par.n_cables+1:end,:)');eye(par.pose_dim-par.n_cables)];
          obj.analitic_orthogonal = par.permutation_matrix'*obj.analitic_orthogonal_P;
          obj.geometric_orthogonal = D*obj.analitic_orthogonal;
          obj.analitic_parallel_P = [inv(obj.analitic_jacobian_P(1:par.n_cables,:)');zeros(par.pose_dim-par.n_cables,par.n_cables)];
          obj.analitic_parallel = par.permutation_matrix'*obj.analitic_parallel_P;
          obj.geometric_parallel = D*obj.analitic_parallel;
      end
      function obj = UpdateTwists(obj,par,pose_d,l_d)
          obj.ExtractVars(par,1,pose_d);
          obj.free_twist = obj.geometric_orthogonal*obj.pose_P_d(par.n_cables:end);
          obj.controlled_twist = obj.geometric_parallel*l_d;
      end
      function obj = UpdateJacobiansD(obj,par,Ja_D,D,D_d)
          J_P_d = par.permutation_matrix*Ja_D;
          obj.analitic_orthogonal_P_d = [-linsolve(obj.analitic_jacobian_P(1:par.n_cables,:)',...
              J_P_d(1:par.n_cables,:)'*obj.analitic_orthogonal_P(1:par.n_cables,:)...
              +J_P_d(par.n_cables+1:end,:)');zeros(par.pose_dim-par.n_cables)];
          obj.analitic_orthogonal_d = par.permutation_matrix'*obj.analitic_orthogonal_P_d;
          obj.geometric_orthogonal_d = D*obj.analitic_orthogonal_d+D_d*obj.analitic_orthogonal;
          
          obj.analitic_parallel_P_d = [-linsolve(obj.analitic_jacobian_P(1:par.n_cables,:)',...
              J_P_d(1:par.n_cables,:)'*obj.analitic_parallel_P(1:par.n_cables,:));zeros(par.pose_dim-par.n_cables,par.n_cables)];
          obj.analitic_parallel_d = par.permutation_matrix'*obj.analitic_parallel_P_d;
          obj.geometric_parallel_d = D*obj.analitic_parallel_d+D_d*obj.analitic_parallel;
      end

    
    function obj = UpdateDynamics(obj,par,Ja,Ja_d,D,D_d,M,C,f)
     
        obj.UpdateJacobians(par,Ja,D);
        obj.UpdateJacobiansD(par,Ja_d,D_d);
        Chi = [obj.geometric_parallel obj.geometric_orthogonal];
        obj.mass_matrix_q = Chi'*M*Chi;
        obj.C_matrix_q = Chi'*(M*[obj.geometric_parallel_d obj.geometric_orthogonal_d]+C*Chi);
        obj.total_load_q = -Chi'*f;
        
    end
    
    function obj = UpdateStatics(obj,par,Ja,D,f)
     
        obj = obj.UpdateJacobians(par,Ja,D);
        Chi = [obj.geometric_parallel obj.geometric_orthogonal];
        obj.total_load_q = -Chi'*f;
    
    end
    
    function obj = UpdateMassMatrix(obj,M)
     
        Chi = [obj.geometric_parallel obj.geometric_orthogonal];
        obj.mass_matrix_q = Chi'*M*Chi;
    
    end
    
    function obj = UpdateCMatrix(obj,M,C)
     
        Chi = [obj.geometric_parallel obj.geometric_orthogonal];
        obj.C_matrix_q = Chi'*(M*[obj.geometric_parallel_d obj.geometric_orthogonal_d]+C*Chi);
    
    end
    
    function obj = UpdateLoad(obj,f)
     
        Chi = [obj.geometric_parallel obj.geometric_orthogonal];
        obj.total_load_q = -Chi'*f;
    
    end
    
    function [tau,constr] = CalcStaticTension(obj,par)
     
        tau = -obj.total_load_q(1:par.n_cables);
        constr = -obj.total_load_q(par.n_cables+1:end);
    
    end
    
    function [tau,constr] = CalcDynamicTension(obj,par,l_d,l_dd)
     
        v = obj.mass_matrix_q*[l_dd;obj.pose_P_dd(par.n_cables:end)]...
            +obj.C_matrix_q*[l_d;obj.pose_P_d(par.n_cables:end)]+obj.total_load_q;
        
        tau = -v(1:par.n_cables);
        constr = v(par.n_cables:end);
    
    end
    
    function obj = ExtractVars(obj,par,order,var)
        
        switch order
            case 0     
                obj.pose_P = par.permutation_matrix*var;
            case 1
                obj.pose_P_d = par.permutation_matrix*var;
            case 2
                obj.pose_P_dd = par.permutation_matrix*var;
        end

    end
    
    function obj = SetVars(obj,order,var_act,var_un_act)
           
        switch order
            case 0     
                obj.pose_P = [var_act;var_un_act];
            case 1
                obj.pose_P_d = [var_act;var_un_act];
            case 2
                obj.pose_P_dd = [var_act;var_un_act];
        end

    end
    
    function obj = SetVarsAct(obj,order,par,var_act)
           
        switch order
            case 0     
                obj.pose_P(1:par.n_cables,1) = var_act;
            case 1
                obj.pose_P_d(1:par.n_cables,1) = var_act;
            case 2
                obj.pose_P_dd(1:par.n_cables,1) = var_act;
        end

    end
    function obj = SetVarsUnAct(obj,order,par,var_unact)
           
        switch order
            case 0     
                obj.pose_P(par.n_cables+1:end,1) = var_unact;
            case 1
                obj.pose_P_d(par.n_cables+1:ebd,1) = var_unact;
            case 2
                obj.pose_P_dd(par.n_cables+1:end,1) = var_unact;
        end

    end
    
    function var = RecomposeVars(obj,order,par)
        
        
        switch order
            case 0
                var = par.permutation_matrix'*obj.pose_P;
            case 1
                var = par.permutation_matrix'*obj.pose_P_d;
            case 2
                var = par.permutation_matrix'*obj.pose_P_dd;
        end

    end
    
  end
end