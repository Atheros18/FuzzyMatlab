function [value,isterminal,direction] = stop_events_2d(~, y, pars)
% y = [x; yW; psi; v; r; theta; thetadot]
th = y(6); v = y(4); r = y(5); x = y(1); yW = y(2);

value = [ ...
    (pi/2) - abs(th);     % stop if |theta| > pi/2
    8      - abs(v);      % stop if |v| > 8 m/s
    8      - abs(r);      % stop if |r| > 8 rad/s
    5      - max(abs([x yW]))  % stop if position leaves 5 m box
];
isterminal = [1;1;1;1];
direction  = [0;0;0;0];
end
