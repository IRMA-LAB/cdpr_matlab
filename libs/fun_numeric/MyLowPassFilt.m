function now_filt_value = MyLowPassFilt(now_raw_value,prev_filt_value,t,ut)

alpha = ut.cutt_off_freq * ut.t_interval;
if (t > 0)
    now_filt_value = prev_filt_value + alpha * (now_raw_value - prev_filt_value);
else
    now_filt_value = now_raw_value;
end

end