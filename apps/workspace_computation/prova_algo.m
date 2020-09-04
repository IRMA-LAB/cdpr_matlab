clc
clear all
close all

limits = [-2 2;
          -2 2;
          -2 2];
mesh = 5;
n = length(limits);
e_mesh = (mesh-1)/2;
ctr = (limits(:,1)+ limits(:,2))/2;
deltas = (limits(:,2)-limits(:,1))/(mesh-1);
iter = zeros(n,1);  dir = ones(n,1);
iter(n) = -1;
i=n;

cnt = 0;
res = [];
conf = ctr;
ig = NaN(length(ctr),1);
ig_s = zeros(length(ctr));

while 1
  iter(i) = iter(i)+1;
  conf(i) = ctr(i) + dir(i)*iter(i)*deltas(i);
  if ((dir(n)==1 && iter(n)==0))
    for j=1:n-1
      if ((dir(j)==1 && iter(j)==1))
        ig_s(:,j) = conf;
      end
    end
  end
  if (iter(i)>e_mesh)
    if (dir(i)==1)
      dir(i) = -1;
      iter(i) = 0;
      ig(:,end) = [];
      ig = [ig ig_s(:,i)];
    else
      dir(i) = 1;
      iter(i) = -1;
      i = i-1;
      ig(:,end) = [];
      ig = [ig ig_s(:,i)];
      if (i == 0)
        break;
      end
    end
  else
    if i==n
      cnt = cnt+1;
      res(:,cnt) = conf;
      ig = [ig res(:,cnt)];
    else
      i = i+1;
    end
  end
end
