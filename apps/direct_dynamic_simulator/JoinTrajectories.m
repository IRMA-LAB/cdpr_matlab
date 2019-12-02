function [t,c,p0] = JoinTrajectories(tr,n,dt)

for i = 1:length(tr)
  if (i == 1)
    t = tr(i).t;
    for j=1:n
      c(j).complete_length = tr(i).cables(j).complete_length;
      c(j).complete_speed = tr(i).cables(j).complete_speed;
      c(j).complete_acceleration = tr(i).cables(j).complete_acceleration;
    end
  else
    t = [t (tr(i).t+t(end))+dt];
    for j=1:n
      c(j).complete_length = [c(j).complete_length tr(i).cables(j).complete_length];
      c(j).complete_speed = [c(j).complete_speed tr(i).cables(j).complete_speed];
      c(j).complete_acceleration = [c(j).complete_acceleration tr(i).cables(j).complete_acceleration];
    end
  end
  t = [t t(end)+dt:dt:t(end)+tr(i).t(end)-dt];
  d1 = size(t);
  d2 = size(c(1).complete_length);
  diff = d1(2)-d2(2);
  if (i == length(tr))
    for j=1:n
      c(j).complete_length = [c(j).complete_length c(j).complete_length(end).*ones(1,diff)];
      c(j).complete_speed = [c(j).complete_speed zeros(1,diff)];
      c(j).complete_acceleration = [c(j).complete_acceleration zeros(1,diff)];
    end
  else
    for j=1:n
      c(j).complete_length = [c(j).complete_length c(j).complete_length(end):(tr(i+1).cables(j).complete_length-c(j).complete_length(end))/(diff-1):tr(i+1).cables(j).complete_length];
      c(j).complete_speed = [c(j).complete_speed zeros(1,diff)];
      c(j).complete_acceleration = [c(j).complete_acceleration zeros(1,diff)];
    end
  end
end

p0 = [tr(1).platform.pose(:,1); tr(1).platform.pose_d(:,1)];


end