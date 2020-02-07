function [mat,l] = ZeroVibrationInputShaping(order,T)


den = 0;
for i= 0:order
    den = den+nchoosek(order,i);
end
mat = zeros(2,order+1);
for i=0:order
   mat(1,i+1) = nchoosek(order,i)/den;
   mat(2,i+1) = i*T/2;
end
l = length(mat);
% switch order
%     case 1
%         mat = [0.5 0.5;
%             0    0.5*T];
%         l=2;
%     case 2
%         mat = [0.25 0.5 0.25;
%             0    0.5*T  T];
%         l=3;
%     case 3
%         mat = [1/8 3/8 3/8 1/8;
%             0    0.5*T  T 1.5*T];
%         l=4;
%     case 4
%         mat = [1/16 4/16 6/16 4/16 1/16;
%             0    0.5*T  T 1.5*T 2*T];
%         l=5;
%     otherwise
%         mat = [1;
%             0];
%         l=1;
% end

end