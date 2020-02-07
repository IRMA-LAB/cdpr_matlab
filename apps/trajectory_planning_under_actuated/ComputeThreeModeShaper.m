function [S,index] = ComputeThreeModeShaper(f,shaper_order)

    
    [shaper_max,shaperL_max] = ZeroVibrationInputShaping(shaper_order,1/max(f));
    [shaper_med,shaperL_med] = ZeroVibrationInputShaping(shaper_order,1/median(f));
    [shaper_min,shaperL_min] = ZeroVibrationInputShaping(shaper_order,1/min(f));
    index = 0;
    for i=1:shaperL_max
        for j=1:shaperL_med
            for k=1:shaperL_min
              index = index +1;
              S(1,index) = shaper_max(1,i)*shaper_med(1,j)*shaper_min(1,k);
              S(2,index) = shaper_max(2,i)+shaper_med(2,j)+shaper_min(2,k);
            end
        end
    end
    [SS,how] = sort(S,2);
    S = [S(1,how(2,:));SS(2,:)];
end