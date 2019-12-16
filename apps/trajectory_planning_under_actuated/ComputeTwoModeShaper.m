function [S,index] = ComputeTwoModeShaper(data,ind,shaper_order)

    period_max = (max(data.info(ind).nat_period)+max(data.info(ind+1).nat_period))/2;
    period_min = (min(data.info(ind).nat_period)+min(data.info(ind+1).nat_period))/2;
    
    [shaper_max,shaperL_max] = ZeroVibrationInputShaping(shaper_order,period_max);
    [shaper_min,shaperL_min] = ZeroVibrationInputShaping(shaper_order,period_min);
    index = 0;
    for i=1:shaperL_max
        for j=1:shaperL_min
              index = index +1;
              S(1,index) = shaper_max(1,i)*shaper_min(1,j);
              S(2,index) = shaper_min(2,i)+shaper_max(2,j);
        end
    end

end