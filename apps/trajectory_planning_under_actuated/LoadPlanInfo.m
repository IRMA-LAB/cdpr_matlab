function out = LoadPlanInfo(name)

json.startup;
direct = pwd;
cd ..\..\data\planning_files\
out =  json.read(strcat(name,'.json'));
for i=1:length(out)
  out2(i) = json.read(strcat(out(i).tech_info));
end
cd(direct);
out = rmfield(out,'tech_info');
for i=1:length(out)
  switch out(i).tr_geom
    case 'LINE'
      out(i).geometricFunction = @LineFunction;
    case 'CIRCLE'
      out(i).geometricFunction = @CircleEquation;
  end
end
out = rmfield(out,'tr_geom');
fn = fieldnames(out2(1));

for i=1:length(out)
  for k=1:numel(fn)
    out(i).(fn{k}) = out2(i).(fn{k});
  end
end

end