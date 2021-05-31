function heu = heuristic_cost(newnodepos, goal)
heu = sqrt((newnodepos(1) - goal(1)).^2 + (newnodepos(2) - goal(2)).^2);