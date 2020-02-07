function [output] = RestToRestCoefficientsMinimumTime(cdpr_p,cdpr_v,...
    sim_data,geom_fun,ut,record)

for i = 1:sim_data.pNumber-1
    
    sim_data.coeff(:,i)  = [zeros(2*(cdpr_p.pose_dim-cdpr_p.n_cables),1)];
    sim_data.dt(i) = sim_data.dt(i)+sim_data.shaper(2,end,i);
    sim_data.T = sim_data.dt(i);
    sim_data.motion_law_function = @ActuatedDofsMod;
    K = [sim_data.coeff(1:2,i) sim_data.coeff(3:4,i) sim_data.coeff(5:6,i)];
    sim_data.cMat=Poly656Coef(sim_data.lims,K);
    sol = ode45(@(time,orientation) IntegrableInverseDynamics(cdpr_p,cdpr_v,...
        sim_data,i,time,orientation,geom_fun),[0 sim_data.dt(i)],...
        [sim_data.p(4:6,i);0;0;0],ut.ode45_options);
    t = linspace(0,sol.x(end),100);
    [y yp] = deval(sol,t);
    condition = 1;
    sim_data.dt(i) = sim_data.dt(i)*10/9;
    while condition
        sim_data.dt(i) = sim_data.dt(i)*0.9;
        sim_data.T = sim_data.dt(i);
        solinit.x = t; solinit.y = y; solinit.parameters = sim_data.coeff(:,i);
        tic
        sol = bvp5c(@(time,orientation,k) IntegrableInverseDynamicsMod(cdpr_p,cdpr_v,...
            sim_data,i,time,orientation,k,geom_fun),@(ya,yb,k) bcfun(ya,yb,k,sim_data,i),solinit,ut.bvp_options);
        toc
        if (sol.stats.maxerr> ut.bvp_options.RelTol)
            sim_data.dt(i) = sim_data.dt(i)*10/9;
            sim_data.T = sim_data.dt(i);
            condition = 0;
            break;
        end
        sim_data.coeff(:,i) = sol.parameters;
        K = [sim_data.coeff(1:2,i) sim_data.coeff(3:4,i) sim_data.coeff(5:6,i)];
        sim_data.cMat=Poly656Coef(sim_data.lims,K);
        sol = ode45(@(time,orientation) IntegrableInverseDynamics(cdpr_p,cdpr_v,...
            sim_data,i,time,orientation,geom_fun),[0 sim_data.dt(i)],...
            [sim_data.p(4:6,i);0;0;0],ut.ode45_options);
        t = linspace(0,sol.x(end),100);
        [y yp] = deval(sol,t);
        for j=1:length(t)
            pStart = sim_data.p(1:cdpr_p.n_cables,i);
            pEnd = sim_data.p(1:cdpr_p.n_cables,i+1);
            normalizedTime = t(j)/sim_data.dt(i);
            dofs = sim_data.motion_law_function(sim_data,normalizedTime,...
                geom_fun,pStart,pEnd,sim_data.dt(i));
            cdpr_v = UpdateIKZeroOrd(dofs(1,:)',y(1:3,j),cdpr_p,cdpr_v);
            cdpr_v = UpdateIKFirstOrd(dofs(2,:)',y(4:6,j),cdpr_p,cdpr_v);
            cdpr_v = UpdateIKSecondOrd(dofs(3,:)',yp(4:6,j),cdpr_p,cdpr_v);
            cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
            cdpr_v = CalcTotalLoadsStateSpace(cdpr_v,cdpr_p);
            cdpr_v = CalcCablesDynamicTension(cdpr_v);
            if (any(cdpr_v.tension_vector-10<0) || any(cdpr_v.tension_vector-1000>0))
                condition = 0;
                break
            end
        end
    end
    sim_data.cMat=Poly656Coef(sim_data.lims,[sim_data.coeff(1:2,i) sim_data.coeff(3:4,i) sim_data.coeff(5:6,i)]);
    out = GenerateOutputUnderActuated(i,cdpr_p,cdpr_v,sim_data,geom_fun,ut);
    output(i).t = out.t;
    output(i).platform = out.platform;
    output(i).cables(:) = out.cable(:);
    output(i).tension_vector = out.tension_vector;
    output(i).coefficients = sim_data.coeff(:,i);
    
end

end