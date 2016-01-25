%-------------------------------------------------------------------------%
%------------------ 1 Degree of Freedom Pendulum Problem -----------------%
%-------------------------------------------------------------------------%
% Parameters:
%-------------------------------------------------------------------------%
%------------------------ Data Required by Problem -----------------------%
%-------------------------------------------------------------------------%
function sol = pendmain(T, targetangle, N)
tic
auxdata.d = 1;        %	length of pendulum (m)
auxdata.m = 1;        %	mass of pendulum (kg)
auxdata.I = 1;        % moment of inertia relative to pivot (kg m^2)
auxdata.g = 9.81;     %	gravity (m s^-2)   

% t represents time (s)
% x represents angle (rad)
% v represents angular velocity (rad/s)
% u represents torque (N*m)

%-------------------------------------------------------------------------%
%------------------------------- Bounds ----------------------------------%
%-------------------------------------------------------------------------%

t0   = 0;                   % initial time
tf   = T;                   % final time
xMin = -4*pi;                 % minimum bound on movement
xMax =  4*pi;                 % maximum bound on movement
uMin = -200;                 % minimum bound on torque
uMax =  200;                 % maximum bound on torque
vMin = 0;                   % minimum bound on velocity
vMax =  10;                 % maximum bound on velocity
% N    =  10;                 %

bounds.phase.initialtime.lower  = t0;
bounds.phase.initialtime.upper  = t0;
bounds.phase.finaltime.lower    = tf; 
bounds.phase.finaltime.upper    = tf;
bounds.phase.initialstate.lower = [0, vMin];
bounds.phase.initialstate.upper = [0, vMin];
bounds.phase.state.lower        = [xMin, vMin];
bounds.phase.state.upper        = [xMax, vMax];
bounds.phase.finalstate.lower   = [xMin, vMin];
bounds.phase.finalstate.upper   = [xMax, vMax];
bounds.phase.control.lower      = uMin;
bounds.phase.control.upper      = uMax;
bounds.phase.integral.lower     = 0;
bounds.phase.integral.upper     = 50000;

% see pendEndpoint.m
% The final angle  is required to be 2pi larger than initial angle
% The final velocity  is required to be 1 rad/s larger than initial velocity

guess.phase.time        = t0 + (0:N-1)'*(tf-t0)/(N-1);
rng('shuffle');

guess.phase.state       = [zeros(N,1) , zeros(N,1)];
guess.phase.control     = zeros(N,1);
guess.phase.integral    = 0;

bounds.eventgroup.lower = [targetangle 0];
bounds.eventgroup.upper = [targetangle 0];

%-------------------------------------------------------------------------%
%------------------------------ Problem Setup ----------------------------%
%-------------------------------------------------------------------------%
setup.name                        = 'pend-Problem';
setup.functions.continuous        = @pendContinuous;
setup.functions.endpoint          = @pendEndpoint;
setup.auxdata                     = auxdata;
setup.bounds                      = bounds;
setup.guess                       = guess;
setup.nlp.solver                  = 'ipopt';
setup.derivatives.supplier        = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.scales.method               = 'none';
setup.mesh.method                 = 'hp-PattersonRao';
setup.derivatives.dependencies    = 'sparseNaN';
setup.mesh.tolerance              = 1e-5; 
setup.method                      = 'RPM-Differentiation';

%-------------------------------------------------------------------------%
%---------------------- Solve Problem Using GPOPS2 -----------------------%
%-------------------------------------------------------------------------%
toc
comp.t = toc;
output = gpops2(setup);
output.result.nlptime
sol = output.result.solution;
sol.comt = comp.t;
sol.costfunc = sol.phase.integral;
objective = output.result.objective;
filename = 'sol.mat';
save(filename, 'sol')

% make movie of the solution
	disp('Hit ENTER to generate animation...');
	pause
    L = 1;
	X = output.result.solution.phase.state(:,1);
    T = output.result.solution.phase.time;
    avi = VideoWriter('pendGPOPS.avi');
    avi.FrameRate = 15;
    open(avi);
	figure(2);
	clf;
	set(gcf,'Position',[5 100 650 650]);
	set(gcf, 'color', 'white');
	s = 1.5*L;
	for i=1:size(X)
		plot([-s s],[0 0],'k','LineWidth',2);
		hold on
		plot([0 L*cos(X(i)-pi/2)], [0 L*sin(X(i)-pi/2)],'b-o','LineWidth',2);
		axis('equal');
		axis('square');
		axis([-s s -s s]);
		title(['t = ' num2str(T(i),'%8.3f')]);
		if (i==1)
			F = getframe(gca);
			frame = [1 1 size(F.cdata,2) size(F.cdata,1)];
		else
			F = getframe(gca,frame);
		end
		writeVideo(avi,F);
		drawnow;
		hold off;
	end
% 	close(avi);
%-------------------------------------------------------------------------%
%------------------------------ Plot Solution ----------------------------%
%-------------------------------------------------------------------------%
% 
figure(1)
plot(sol.phase.time,sol.phase.state(:,1)*180/pi,'linewidth',2);

hold on
plot(sol.phase.time,sol.phase.state(:,2)*180/pi,'r','linewidth',2);
xlabel('Time [s]');
ylabel('State (degree)');
legend('State','Velocity');

    
figure(2)
plot(sol.phase.time,sol.phase.control,'linewidth',2);
xlabel('Time [s]');
ylabel('Torque [N.m]');

end