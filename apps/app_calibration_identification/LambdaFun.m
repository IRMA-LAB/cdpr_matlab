function mat = LambdaFun(v)

mat = [v(1) 0 0 v(2) v(3) 0;
       0 v(2) 0 v(1) 0 v(3);
       0 0 v(3) 0 v(1) v(2)];

end