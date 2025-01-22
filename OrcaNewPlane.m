%% this function takes 8 inputs and output the vo planes
%% input:
%% p: self position
%% v: self velocity,   both are vector 1*3
%% op: obstacles position
%% ov: obstacles velocity,   both are matrix n*3
%% exd: the extra distance
%% th: time horizon
%% dt: minimum time duration
%% rto: the ratio of avoid other obstacles
%% output:
%% plane: the vo planes,     matrix n*4
%% maskstc: static obstacles mask,   vector n*1
function [plane,maskstc] = OrcaNewPlane(p,v,op,ov,exd,th,dt,rto)
    %% Create agent ORCA planes. 
    num = size(op,1);
    if num < 1
        plane = [];
        maskstc = [];
        return;
    end
    itv = 1 ./ th * ones(num,1);
    rep = op - p;
    rev = v - ov;             % 如果有静止物体，v设为0
    distsq = dot(rep,rep,2);
    exdsq = exd.^2;
    plane = zeros(num,4);
    u = zeros(num,3);
    % 静止处理  handle static condition
%     maskstc = maskstatic;
    maskstc = vecnorm(ov,2,2) < 1e-6;   % 这里认为小于一定速度的为静止，如果有其他判断方法，也可以直接传入一个logical数组 if speed < ?, static
    rev(maskstc,:) = rev(maskstc,:) .* 0;      % 则相对速度也当然为0  set v to 0
    itv(maskstc,:) = 1/5;    % 而时间窗口可以适当放低，因为静止物体不会有主动接近的可能性 loose time horizon
    
    maskclloid = distsq > exdsq;
    %% No collision.
    w = rev - itv .* rep;
    % Vector from cutoff center to relative velocity.
    wLengthSq = dot(w,w,2);
    dotProduct = dot(w,rep,2);
    maskcircle = (maskclloid) & (dotProduct < 0) & (dotProduct .* dotProduct > exdsq .* wLengthSq);
    % Circle edge
    wLength = sqrt(wLengthSq);
    unitW = w ./ wLength;

    plane(maskcircle,1:3) = unitW(maskcircle,:);
    tu = (exd .* itv - wLength) .* unitW;
    u(maskcircle,:) = tu(maskcircle,:);
    % Cone edge
    maskcone = (maskclloid)&(~maskcircle);
    a = distsq;
    b = dot(rep,rev,2);
    crosspv = cross(rep,rev,2);
    c = dot(rev,rev,2) - (dot(crosspv,crosspv,2)./(distsq - exdsq));
    t = (b + sqrt(abs(b .* b - a .* c))) ./ a;
    ww = rev - t .* rep;
    wwLength = vecnorm(ww,2,2);
    unitWW = ww ./ (wwLength + 1e-8);

    plane(maskcone,1:3) = unitWW(maskcone,:);
    tu = (exd .* t - wwLength) .* unitWW;
    u(maskcone,:) = tu(maskcone,:);
    %% Collision
    its = 1 ./ dt;
    w = rev - its .* rep;
    wLength = vecnorm(w,2,2);
    unitW = w ./ wLength;

    plane(~maskclloid,1:3) = unitW(~maskclloid,:);
    tu = (exd .* its - wLength) .* unitW;
    u(~maskclloid,:) = tu(~maskclloid,:);
    %% Plane Point
    point = v + rto .* u;       % 如果有静止物体，静止物体的ratio应该为1，从而避免碰撞 static obsacle, rto shout be 1
    point(maskstc,:) = u(maskstc,:);      % 此处对v置0存疑，如果ratio为1，无论v是多少，得到的plane都应该是VO的边界
    plane(:,4) = -(dot(plane(:,1:3),point,2));
end
