function adm = check_admissibility_3d(nodepos, parentnode, obstacles)
%%%checks if the input node(s) are inside a box

global domain;
N = 3;

%%%check solution domain admissibility
numnodes = size(nodepos, 1);
admissdomain = true(numnodes, 1);
for i = 1:N
    in = nodepos(:, i) >= domain(i,1) & nodepos(:,i) <= domain(i,2); %check whether "nodepos" lies inside obstacles       
    admissdomain = admissdomain & in;
end
adm = admissdomain;

if sum(adm) == 0        %none of the nodes sent in for checking lie inside the domain
    return;
end

%%%check obstacle admissibility
obsnum = obstacles.number;
admissobst = true(size(nodepos,1), 1);
for i = 1:obsnum
    currentobstacle = obstacles.vertices{i};
    %%%below is the NOT(admissible) condition: being ON or INSIDE an
    %%%obstacle
    [in, on] = inpolygon(nodepos(:,1), nodepos(:,2), currentobstacle(:,1), currentobstacle(:,2));  
    %%%Correct the output to be "admissibility conditions"
    admissobst = admissobst & (not(in) & not(on));      
end

%%%for the nodes that admissible so far, check the line of sight condition
for i = 1:numnodes
    if admissobst(i) == 0   %this node is already inadmissible
        continue;
    else
        for j = 1:obsnum
            obc = [obstacles.vertices{j}; obstacles.vertices{j}(1,:)]; %current obstacle (close it up on itself)
            obvn = size(obc, 1);    %number of vertices + 1 (repeated the 1st vertex)
            for k = 1:obvn-1
                if admissobst(i) == 0
                    break;   %chage with break
                else
                    %%%line 1: parent-node pair
                    p1 = parentnode(i, 1:2)'; p2 = nodepos(i, 1:2)';    %fix this: currently only a 2d check
                    %%%line 2: obstacle edge
                    p3 = obc(k,:)'; p4 = obc(k+1,:)';
                    Amat = [(p2 - p1), (p3 - p4)];
                    cA = cond(Amat);
                    if cA < 1e12
                        bvec = p3 - p1;    
                        lamyu = Amat\bvec;
                        if ((lamyu(1) < 0 || lamyu(1) > 1) || (lamyu(2) < 0 || lamyu(2) > 1)) %does not intersect
                            continue;
                        else   %intersects, and so, in-admissible
                            admissobst(i) = 0;
                        end
                    end
                end                
            end
        end
    end
end

adm = adm & admissobst;