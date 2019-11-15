function out = GenerateCdprOutput(var,index,out)

outputFieldsPlatform = fieldnames(var.platform);
n_elem = length(var.cable);
outputFieldsCable = fieldnames(var.cable(1));
out.index = [out.index index];
for i=1:numel(outputFieldsPlatform)
    out.platform.(outputFieldsPlatform{i}) = [out.platform.(outputFieldsPlatform{i}) var.platform.(outputFieldsPlatform{i})];
end

for j = 1:n_elem
    for i = 1:numel(outputFieldsCable)
        out.cable(j).(outputFieldsCable{i}) = [out.cable(j).(outputFieldsCable{i}) var.cable(j).(outputFieldsCable{i})];
    end
end
end