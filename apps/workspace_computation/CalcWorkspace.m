function out = CalcWorkspace(cdpr_p,cdpr_v,ut,out,folder,rec,ws_info)

switch ws_info.workspace_type
    case WorkspaceTypes.TRANSLATIONAL
        switch cdpr_p.motion_pattern
            case MotionPatterns.TWO_T
                if (cdpr_p.platform.rotation_parametrization ==  RotationParametrizations.QUATERNION)
                    if (cdpr_p.n_cables<cdpr_p.pose_dim-1)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim-1)
                        out = CalcTranslWorkspaceOverPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcTranslWorkspaceCompletelyPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                else
                    if (cdpr_p.n_cables<cdpr_p.pose_dim)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim)
                        out = CalcTranslWorkspaceOverPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcTranslWorkspaceCompletelyPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                end
            case MotionPatterns.THREE_T
                if (cdpr_p.platform.rotation_parametrization ==  RotationParametrizations.QUATERNION)
                    if (cdpr_p.n_cables<cdpr_p.pose_dim-1)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim-1)
                        out = CalcTranslWorkspaceOverSpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcTranslWorkspaceCompletelySpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                else
                    if (cdpr_p.n_cables<cdpr_p.pose_dim)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim)
                        out = CalcTranslWorkspaceOverSpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcTranslWorkspaceCompletelySpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                end
            otherwise
                disp('Workspace type and motion pattern makes no sense');
        end
    case WorkspaceTypes.ORIENTATION
        switch cdpr_p.motion_pattern
            case MotionPatterns.ONE_R_TWO_T
                if (cdpr_p.platform.rotation_parametrization ==  RotationParametrizations.QUATERNION)
                    if (cdpr_p.n_cables<cdpr_p.pose_dim-1)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim-1)
                        out = CalcOrientWorkspaceOverPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcOrientWorkspaceCompletelyPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                else
                    if (cdpr_p.n_cables<cdpr_p.pose_dim)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim)
                        out = CalcOrientWorkspaceOverPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcOrientWorkspaceCompletelyPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                end
            case MotionPatterns.THREE_R_THREE_T % Only TILT_TORSION parametrization implemented
                if (cdpr_p.platform.rotation_parametrization ==  RotationParametrizations.QUATERNION)
                    if (cdpr_p.n_cables<cdpr_p.pose_dim-1)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim-1)
                        out = CalcOrientWorkspaceOverSpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcOrientWorkspaceCompletelySpatial(cdpr_p,cdpr_v,ut,ws_info,rec);
                    end
                else
                    if (cdpr_p.n_cables<cdpr_p.pose_dim)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim)
                        out = CalcOrientWorkspaceOverSpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcOrientWorkspaceCompletelySpatial(cdpr_p,cdpr_v,ut,ws_info,rec); 
                    end
                end
            otherwise
                disp('Workspace type and motion pattern makes no sense');
        end
    case WorkspaceTypes.REACHABLE
        switch cdpr_p.motion_pattern
            case MotionPatterns.ONE_R_TWO_T
                if (cdpr_p.platform.rotation_parametrization ==  RotationParametrizations.QUATERNION)
                    if (cdpr_p.n_cables<cdpr_p.pose_dim-1)
                        out = CalcReachWorkspaceUnderPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim-1)
                        out = CalcReachWorkspaceOverPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcReachWorkspaceCompletelyPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                else
                    if (cdpr_p.n_cables<cdpr_p.pose_dim)
                        out = CalcReachWorkspaceUnderPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim)
                        out = CalcReachWorkspaceOverPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcReachWorkspaceCompletelyPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                end
            case MotionPatterns.THREE_R_THREE_T
                if (cdpr_p.platform.rotation_parametrization ==  RotationParametrizations.QUATERNION)
                    if (cdpr_p.n_cables<cdpr_p.pose_dim-1)
                        out = CalcReachWorkspaceUnderSpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim-1)
                        out = CalcReachWorkspaceOverSpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcReachWorkspaceCompletelySpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                else
                    if (cdpr_p.n_cables<cdpr_p.pose_dim)
                        out = CalcReachWorkspaceUnderSpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim)
                        out = CalcReachWorkspaceOverSpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcReachWorkspaceCompletelySpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                end
            otherwise
                disp('Workspace type and motion pattern makes no sense');
        end
    case WorkspaceTypes.TOTAL_ORIENTATION
        switch cdpr_p.motion_pattern
            case MotionPatterns.ONE_R_TWO_T
                if (cdpr_p.platform.rotation_parametrization ==  RotationParametrizations.QUATERNION)
                    if (cdpr_p.n_cables<cdpr_p.pose_dim-1)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim-1)
                        out = CalcTotOrientWorkspaceOverPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcTotOrientWorkspaceCompletelyPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                else
                    if (cdpr_p.n_cables<cdpr_p.pose_dim)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim)
                        out = CalcTotOrientWorkspaceOverPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcTotOrientWorkspaceCompletelyPlanar(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                end
            case MotionPatterns.THREE_R_THREE_T
                if (cdpr_p.platform.rotation_parametrization ==  RotationParametrizations.QUATERNION)
                    if (cdpr_p.n_cables<cdpr_p.pose_dim-1)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim-1)
                        out = CalcTotOrientWorkspaceOverSpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcTotOrientWorkspaceCompletelySpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                else
                    if (cdpr_p.n_cables<cdpr_p.pose_dim)
                        disp('Workspace type is not well defined for under-actuated CDPRs');
                    elseif(cdpr_p.n_cables>cdpr_p.pose_dim)
                        out = CalcTotOrientWorkspaceOverSpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    else
                        out = CalcTotOrientWorkspaceCompletelySpatial(cdpr_p,cdpr_v,ut,ws_info,rec); % TODO
                    end
                end
            otherwise
                disp('Workspace type and motion pattern makes no sense');
        end
end

out = ReorderResults(cdpr_p,ws_info.tension_limits,out);

end