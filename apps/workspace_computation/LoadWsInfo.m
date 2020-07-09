function out = LoadWsInfo(name)

json.startup;
out =  json.read(strcat(name,'.json'));

end