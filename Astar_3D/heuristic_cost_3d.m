function heu = heuristic_cost_3d(newnodepos, goal)
heu = sqrt((newnodepos(1) - goal(1)).^2 + (newnodepos(2) - goal(2)).^2 + (newnodepos(3) - goal(3)).^2);