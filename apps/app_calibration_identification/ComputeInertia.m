function paramters = ComputeInertia(cdpr_p,data)

    n = length(data.dl);
    identification_matrix = [];
    for i = 1:n
        name = strcat('pose',int2str(i-1));
        name2 = strcat('pose_d',int2str(i-1));
        name3 = strcat('pose_dd',int2str(i-1));
        p = data.(name);
        v = data.(name2);
        a = data.(name3);
        for j = 1:length(p)
            identification_matrix = [identification_matrix;ComputeIdentifMatrix(p(:,i),v(:,i),a(:,i),cdpr_p)];
        end
    end
    parameters = linsolve(identification_matrix(:,2:end),-identification_matrix(:,1));
    dev = StdDeviation(identification_matrix(:,2:end),parameters,-identification_matrix(:,1));

end