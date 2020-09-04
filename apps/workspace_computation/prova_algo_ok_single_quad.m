clc
clear all
close all

limits = [-2 2;
          -2 2;
          -2 2;
          -2 2];
mesh = 5;
n = length(limits);
%e_mesh = (mesh-1)/2;
ctr = (limits(:,1)+ limits(:,2))/2;
deltas = (limits(:,2)-limits(:,1))/(mesh-1);
iter = zeros(n,1);  %dir = ones(n,1);
iter(n) = -1;
i=n;

cnt = 0;
res = [];
conf = ctr;
ig_s = zeros(length(ctr));
prev = ig_s(:,end);
ig = [];
cmp = [];
while 1
  iter(i) = iter(i)+1;
  conf(i) = ctr(i) + iter(i)*deltas(i);
  if (iter(i)>mesh)
    iter(i) = -1;
    prev = ig_s(:,i);
    i = i-1;
    if (i == 0)
      break;
    end
  else
    if i==n
      cnt = cnt+1;
      ig = [ig prev];
      res = [res conf];
      new = [conf;NaN;prev];
      cmp = [cmp new];
      prev = res(:,cnt);
      if (iter(i) == 0)
        ig_s(:,i) = prev;
      end
      
    else
      if (iter(i) == 0)
        ig_s(:,i) = ig_s(:,i+1);
      end
      i = i+1;
    end
  end
end

