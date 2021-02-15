function constraint = DK_fun(p,l,cdpr_parameters,cdpr_variables)

[~,constraint] = CalcKinZeroOrdConstr(p(1:3),p(4:6),l,...
  cdpr_parameters,cdpr_variables);

end