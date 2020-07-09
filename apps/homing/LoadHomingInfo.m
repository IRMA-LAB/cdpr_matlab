function out = LoadHomingInfo(name)

json.startup;
out =  json.read(strcat(name,'.json'));

end