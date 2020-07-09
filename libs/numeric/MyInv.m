function mat = MyInv(m)

mat = linsolve(m,eye(length(m)));
% dim = size(m);
% if (dim(1)~=dim(2))
%     m = m'*m;
% end
% dim = size(m);
% for i=1:dim(1)
%     v = zeros(dim(1),1);
%     v(i) = 1;
%     mat(i,:) = linsolve(m,v);
% end

end