function [pos, tar] = generate_initial_state(num, lb, lp, type)
%GENERATE INITIAL STATE generates agents initial pos and target pos
%   Parameters:
%   NUM - The amount of agents
%   LB - Bottom of 3D space
%   LP - Top if 3D space
%   TYPE - 'Circle' or 'Random'
    pos = [];
    tar = [];
    if strcmp(type,'Circle')
        ag = 0:2*pi/num:2*pi-2*pi/num;
        pos = [cos(ag'), sin(ag'), zeros(num,1)].*(lp(1)-lb(1))./2 + (lp+lb)./2;
        tar = [cos(ag'+pi), sin(ag'+pi), zeros(num,1)].*(lp(1)-lb(1))./2 + (lp+lb)./2;
    end
    if strcmp(type,'Random')
        pos = rand(num,3).*(lp-lb) + lb;
        tar = rand(num,3).*(lp-lb) + lb;
    end
end