function pose_n = NormalizeOrientation(cdpr_p,pose_n)

if (cdpr_p.platform.rotation_parametrization ~= RotationParametrizations.QUATERNION )
    for j=1:3
        if (pose_n(3+j)>0)
            fact = floor(pose_n(3+j)/(2*pi));
        else
            fact = ceil(pose_n(3+j)/(2*pi));
        end
        pose_n(3+j) = pose_n(3+j)-fact*(2*pi);
        
    end
    if (cdpr_p.platform.rotation_parametrization == RotationParametrizations.TILT_TORSION)
        if (pose_n(5)<0)
            pose_n(5) = -pose_n(5);
            pose_n(4) = pose_n(4)-pi;
        elseif (pose_n(5)>pi)
            pose_n(5) = 2*pi-pose_n(5);
            pose_n(4) = pose_n(4)-pi;
        end
        if (pose_n(4)>pi)
            pose_n(4) = 2*pi-pose_n(4);
        elseif (pose_n(4)<-pi)
            pose_n(4) = 2*pi+pose_n(4);
        end
    end
end

end