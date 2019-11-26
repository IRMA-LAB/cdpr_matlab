function der = FiniteDifferentiation(val,val_prev,diff_prev,t,ut)

der = (val-val_prev)/ut.t_interval;
der = MyLowPassFilt(der,diff_prev,t,ut);

end