function adm = check_admissibility_nolos(nodepos, obstacles)
%%%checks if the input node(s) are inside a box

global domain;
N = 2;

%%%check solution domain admissibility
admissdomain = true(size(nodepos,1), 1);
for i = 1:N
    in = nodepos(:, i) >= domain(i,1) & nodepos(:,i) <= domain(i,2);
    admissdomain = admissdomain & in;
end
adm = admissdomain;


%%%check obstacle admissibility
obsnum = obstacles.number;
admissobst = true(size(nodepos,1), 1);
for i = 1:obsnum
    currentobstacle = obstacles.vertices{i};
    %%%below is the NOT(admissible) condition: being ON or INSIDE an
    %%%obstacle
    [in, on] = inpolygon(nodepos(:,1), nodepos(:,2), currentobstacle(:,1), currentobstacle(:,2));  
    %%%Correct the output to be "admissibility conditions
    admissobst = admissobst & (not(in) & not(on));   
end

adm = adm & admissobst;