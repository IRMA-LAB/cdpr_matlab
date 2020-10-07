function vect = CableLengthFromOscillation(l,cdpr_p,cdpr_v,pose_s)

vect = [];
pose = pose_s.val;
for i=1:length(pose)
[cdpr_v,constraint_l] = CalcKinZeroOrdConstr(pose(1:3,i),pose(4:end,i),l,cdpr_p,cdpr_v);
vect = [vect;constraint_l];
end

end