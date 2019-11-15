function [pos,vel,acc] = Interpolate(sp,time,dt)

pos = ppval(sp,time);

if (time>=2*dt)
    pm1 = ppval(sp,time-dt);
    pm2 = ppval(sp,time-2.*dt);
    M = [1 0      0
        1 (dt)   (dt)^2;
        1 (2*dt) (2*dt)^2];
    F = [pm2;pm1;pos];
    c = linsolve(M,F);
    vel = [0 1 2*(2*dt)]*c;
    acc = [0 0 2]*c;
else
    if (time>=dt)
        pm1 = ppval(sp,time-dt);
        vel = (pos-pm1)/dt;
        acc = 0;
    else
        vel = 0;
        acc = 0;
    end
end



end