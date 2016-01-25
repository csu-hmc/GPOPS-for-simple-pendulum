function phaseout = pendContinuous(input)
m = input.auxdata.m;
d = input.auxdata.d;
g = input.auxdata.g;
u = input.phase.control;

x = input.phase.state(:,1);
v = input.phase.state(:,2);
% keyboard
xdot = v;
vdot = -m.*g.*d.*sin(x)+u;

phaseout.dynamics = [xdot vdot];
phaseout.integrand = u.^2;