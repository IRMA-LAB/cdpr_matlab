classdef UnderActuatedVar
  %ANDERACTUATEDVAR is a class containing time dependent variables of the regarding under-actuated cdprs.
  %
  %   ANDERACTUATEDVAR updates ...
  %
  %
  properties
    actuated
    unactuated
    
    geometric_jacobian_a
    geometric_jacobian_u
    geometric_orthogonal
    analitic_jacobian_a
    analitic_jacobian_u
    analitic_orthogonal
    
    actuated_deriv
    unactuated_deriv
    
    actuated_deriv_2
    unactuated_deriv_2
    
    mass_matrix_global_a
    mass_matrix_global_u
    
    mass_matrix_global_ss_a
    mass_matrix_global_ss_u
    
    total_load_a
    total_load_u
    
    total_load_ss_a
    total_load_ss_u
    
    external_load_a
    external_load_u
    
    external_load_ss_a
    external_load_ss_u
    
    Gamma_mat
  end
  
  methods
    
    function obj = UpdateGeometricJacobians(obj, par, Jg)
      obj.geometric_jacobian_a = Jg(:, par.actuated_mask);
      obj.geometric_jacobian_u = Jg(:, par.unactuated_mask);
      [n, m] = size(Jg);
      
      app1 = zeros(n, m-n);
      app2 = eye(m - n);
      for i = 1:m-n
        app1(:, i) = linsolve(-obj.geometric_jacobian_a, ...
          obj.geometric_jacobian_u(:,i));
      end
      obj.geometric_orthogonal(par.actuated_mask,:) = app1;
      obj.geometric_orthogonal(par.unactuated_mask,:) = app2;
    end
    
    function obj = UpdateAnaliticJacobians(obj, par, Ja)
      obj.analitic_jacobian_a = Ja(:, par.actuated_mask);
      obj.analitic_jacobian_u = Ja(:, par.unactuated_mask);
      [n,m] = size(Ja);
      
      app1 = zeros(n, m-n);
      app2 = eye(m - n);
      for i = 1:m-n
        app1(:,i) = linsolve(-obj.analitic_jacobian_a,...
          obj.analitic_jacobian_u(:,i));
      end
      obj.analitic_orthogonal(par.actuated_mask,:) = app1;
      obj.analitic_orthogonal(par.unactuated_mask,:) = app2;
    end
    
    function obj = UpdateDynamics(obj, par, J, M, l)      
      obj = UpdateGeometricJacobians(obj, par, J);
      
      obj.mass_matrix_global_a = M(:, par.actuated_mask);
      obj.mass_matrix_global_u = M(:, par.unactuated_mask);
      
      obj.total_load_a = l(par.actuated_mask);%
      obj.total_load_u = l(par.unactuated_mask);%      
    end
    
    function obj = UpdateStatics(obj, par, J, l)      
      obj = UpdateGeometricJacobians(obj, par, J);
      
      obj.external_load_a = l(par.actuated_mask);%
      obj.external_load_u = l(par.unactuated_mask);%      
    end
    
    function obj = UpdateDynamicsStateSpace(obj, par, J, M, l)      
      obj = UpdateAnaliticJacobians(obj, par, J);
      
      obj.mass_matrix_global_ss_a = M(:, par.actuated_mask);
      obj.mass_matrix_global_ss_u = M(:, par.unactuated_mask);
      
      obj.total_load_ss_a = l(par.actuated_mask);%
      obj.total_load_ss_u = l(par.unactuated_mask);%
    end
    
    function obj = UpdateStaticsStateSpace(obj, par, J, l)      
      obj = UpdateAnaliticJacobians(obj, par, J);
      
      obj.external_load_ss_a = l(par.actuated_mask);%
      obj.external_load_ss_u = l(par.unactuated_mask);%      
    end
    
    function obj = ExtractVars(obj, par, order, var)      
      switch order
        case 0
          obj.actuated = var(par.actuated_mask);
          obj.unactuated = var(par.unactuated_mask);
        case 1
          obj.actuated_deriv = var(par.actuated_mask);
          obj.unactuated_deriv = var(par.unactuated_mask);
        case 2
          obj.actuated_deriv_2 = var(par.actuated_mask);
          obj.unactuated_deriv_2 = var(par.unactuated_mask);
      end      
    end
    
    function obj = SetVars(obj, order, var_act, var_un_act)
      switch order
        case 0
          obj.actuated = var_act;
          obj.unactuated = var_un_act;
        case 1
          obj.actuated_deriv = var_act;
          obj.unactuated_deriv = var_un_act;
        case 2
          obj.actuated_deriv_2 = var_act;
          obj.unactuated_deriv_2 = var_un_act;
      end      
    end
    
    function var = RecomposeVars(obj, order, par)      
      var = zeros(length([obj.actuated; obj.unactuated]), 1);
      switch order
        case 0
          var(par.actuated_mask) = obj.actuated;
          var(par.unactuated_mask) = obj.unactuated;
        case 1
          var(par.actuated_mask) = obj.actuated_deriv;
          var(par.unactuated_mask) = obj.unactuated_deriv;
        case 2
          var(par.actuated_mask) = obj.actuated_deriv_2;
          var(par.unactuated_mask) = obj.unactuated_deriv_2;
      end      
    end
    
    function obj = UpdateReducedTransformationMatrix(obj, mat)      
      dims = size(obj.analitic_orthogonal);
      obj.Gamma_mat = mat(dims(2) + 1:dims(1), :) * obj.analitic_orthogonal;      
    end
  end
end
