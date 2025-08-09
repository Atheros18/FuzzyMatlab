function [value,isterminal,direction] = stop_events(~,y)
X = y(1); th = y(3);
% stop if cart leaves ±2 m or angle leaves ±90°
value = [ 2 - abs(X);  (pi/2) - abs(th) ];
isterminal = [1; 1];   % stop integration
direction  = [0; 0];
end
