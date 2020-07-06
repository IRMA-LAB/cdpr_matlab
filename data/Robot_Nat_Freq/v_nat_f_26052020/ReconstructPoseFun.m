function vect = ReconstructPoseFun(points,marker,p)

ref_p = p(1:3);
R = RotXYZ(p(4:6));

if(~isnan(mean(points(:,1))))
    vect = ref_p-points(:,1);
else
    vect = [];
end

for i=1:length(marker)
    if(~isnan(mean(points(:,i+1))))
        vect = [vect;ref_p+R*marker(i).p-points(:,i+1)] ;
    end
end

end