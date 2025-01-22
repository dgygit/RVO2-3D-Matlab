function newvel = OrcaNewVel(plane,prefv,lb,ub,maskstc)       
    H = [2 0 0; 0 2 0; 0 0 2];
    c = [-2*prefv(1); -2*prefv(2); -2*prefv(3)];
    if isempty(plane)
        A = [];
        b = [];
    else
        A = -plane(:,1:3);
        b = plane(:,4);
    end
    Aeq = [];
    beq = [];
    options = optimoptions('quadprog', 'OptimalityTolerance', 1e-3,'ConstraintTolerance',1e-5);
    [x, fval, exitflag] = quadprog(H, c, A, b, Aeq, beq, lb, ub,[],options);
    if(exitflag<0)
        % import t >= (-ax - by - cz - d)/norm([a,b,c])
        % 如果有静止物体，静止物体的约束不能放松，仍要添加进去
        % no loose static obstacles
        c = [0; 0; 0; 1];
        A = [-plane(:,1:3),-vecnorm(plane(:,1:3),2,2)];
        if any(maskstc)
            A(maskstc,4) = 0;
        end
        lb = [lb;-Inf];
        ub = [ub;Inf];
        [x, fval, exitflag] = linprog(c, A, b, Aeq, beq, lb, ub);
    end
    newvel = [0 0 0];
    if exitflag >0
        newvel = x';
    end
end