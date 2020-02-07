function [S,index] = ComputeMultiModeShaper(fr,ind,shaper_order,ut)

    [S_conv,index_conv] = ComputeThreeModeShaper(fr,shaper_order);
    z = zeros(1,length(fr));
    n_pulse = 1+length(fr)*shaper_order;
    %[guess,~] = ZeroVibrationInputShaping(n_pulse-1,sum(1./f)/2*shaper_order/(n_pulse));
    t_start = 1/(8*max(fr)*shaper_order);
    t_end = 8/(min(fr)*shaper_order);
    trials = 500;
    i = 0;
    for t = t_start:(t_end-t_start)/trials:t_end
    [guess,~] = ZeroVibrationInputShapingNImpulse(n_pulse,t);
    [s1,f_val] = fsolve(@(x) MultiModeVibrationNullFunction(x,fr,z,shaper_order),[guess(1,:)';guess(2,2:end)'],ut.fsolve_options_grad);
    if (norm(f_val)<0.001)
    S = [s1(1:(length(s1)+1)/2)';0 s1((length(s1)+1)/2+1:end)'];
    S(2,:) = S(2,:)-min(S(2,:));
    [S(2,:),idx] = sort(S(2,:));
    S(1,:) = S(1,idx);
    index = length(S);
    if (~any(S(1,:)<0) && (floor(S(1,1)*10000)==floor(S(1,end)*10000)))
        i=i+1;
        shaper(:,:,i) =S;
    end
    end
    end
    [~,~,n] = size(shaper);
    for i=1:n
      mat(:,i) =  floor(shaper(:,1,i).*10000);
    end
    [~,c,~] = unique(mat','rows');
    shaper_min = shaper(:,:,c);
    [~,idx] = min(shaper_min(2,end,:));
    S_dir = shaper_min(:,:,idx);
    index_dir = index;
    
    S = S_dir;
    index = index_dir;
end