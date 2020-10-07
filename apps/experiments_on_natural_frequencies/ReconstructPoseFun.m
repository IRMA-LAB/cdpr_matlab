function [vect,mat] = ReconstructPoseFun(points,marker,p)

ref_p = p(1:3);
R = RotXYZ(p(4:6));
H = HtfTaytBryan(p(4:6));

if(~isnan(mean(points(:,1))))
    vect = ref_p-points(:,1);
    mat = [eye(3) zeros(3)];
else
    vect = [];
    mat = [];
end

for i=1:length(marker)
    if(~isnan(mean(points(:,i+1))))
        vect = [vect;ref_p+R*marker(i).p-points(:,i+1)] ;
        mat2 = [eye(3) -Anti(R*marker(i).p)*H];
        mat = [mat;mat2];
    end
end

end