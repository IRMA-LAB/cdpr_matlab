function [t,c,p0] = JoinTrajectories(tr,n,dt)

for i = 1:length(tr)
  if (i == 1)
    t = tr(i).t;
    for j=1:n
      c(j).length = tr(i).cables(j).length;
      c(j).speed = tr(i).cables(j).speed;
      c(j).acceleration = tr(i).cables(j).acceleration;
    end
  else
    t = [t (tr(i).t+t(end))+dt];
    for j=1:n
      c(j).length = [c(j).length tr(i).cables(j).length];
      c(j).speed = [c(j).speed tr(i).cables(j).speed];
      c(j).acceleration = [c(j).acceleration tr(i).cables(j).acceleration];
    end
  end
  t = [t t(end)+dt:dt:t(end)+tr(i).t(end)-dt];
  d1 = size(t);
  d2 = size(c(1).length);
  diff = d1(2)-d2(2);
  if (i == length(tr))
    for j=1:n
      c(j).length = [c(j).length c(j).length(end).*ones(1,diff)];
      c(j).speed = [c(j).speed zeros(1,diff)];
      c(j).acceleration = [c(j).acceleration zeros(1,diff)];
    end
  else
    for j=1:n
      c(j).length = [c(j).length c(j).length(end):(tr(i+1).cables(j).length-c(j).length(end))/(diff-1):tr(i+1).cables(j).length];
      c(j).speed = [c(j).speed zeros(1,diff)];
      c(j).acceleration = [c(j).acceleration zeros(1,diff)];
    end
  end
end

p0 = [tr(1).platform.pose(:,1); tr(1).platform.pose_d(:,1)];


end