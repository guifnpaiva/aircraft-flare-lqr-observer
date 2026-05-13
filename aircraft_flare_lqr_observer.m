%[text] ### Setup
% Clear the workspace
clear all; close all; clc

% Select three distinct colors
co = lines(3);  

% Simulation time vector
t = 0:0.002:30;

% Number of initial conditions
nCond = 3;
Nt    = numel(t);

% Preallocation 
h_MF      = zeros(Nt, nCond);  % without observer
h_OBS0    = zeros(Nt, nCond);  % observer with x_hat(0) = 0
h_OBSinit = zeros(Nt, nCond);  % observer with well-initialized x_hat(0)
%[text] ### Linearized System
A = [ -0.6 -0.76 0    0; 
      1    0     0    0; 
      0    102.4 -0.4 0; 
      0    0     1    0 ];

B = [ -2.375; 0; 0; 0 ];

C = [ 1 0 0 0; 
      0 1 0 0; 
      0 0 0 1 ];

D = 0;

sys= ss(A,B,C,D);
%[text] ### Checking Controllability
rankCo = rank(ctrb(A, B)) % Fully controllable
%[text] ### 1) Computing the LQR Gain Vector K
Q = diag([1.6^2 1.6^2 1e-4 1e-5]);
R = 1;
K=lqr(A,B,Q,R)
%[text] ### 2) Landing Response for Different Initial Conditions
%[text] Given the system
%[text]{"align":"center"} $\\dot{x} =\\textrm{Ax}+\\textrm{Bu}${"editStyle":"visual"}
%[text] given that
%[text]{"align":"center"} $u={-K}\_1 x\_1 {\\;-K}\_2 x\_{2\\;} \\;{-K}\_3 x\_3 \\;{-K}\_4 x\_4 +K\_3 \\dot{h\_d } +K\_4 h\_d \\;${"editStyle":"visual"}
%[text] it can be rewritten as:
%[text]{"align":"center"} $u=-\\textrm{Kx}+K\_r r${"editStyle":"visual"}
%[text] where,
%[text]{"align":"center"} $K=\\left\\lbrack \\begin{array}{cccc}\nK\_1  & K\_2  & K\_3  & K\_4 \n\\end{array}\\right\\rbrack ,{\\;\\;\\;\\;\\;\\;K}\_r =\\left\\lbrack \\begin{array}{cc}\nK\_3  & K\_4 \n\\end{array}\\right\\rbrack${"editStyle":"visual"}
%[text] and the reference $r:${"editStyle":"visual"}
%[text]{"align":"center"} $r=\\left\\lbrack \\begin{array}{c}\n\\dot{\\left\\lbrack h\_d \\right\\rbrack } \\\\\nh\_d \n\\end{array}\\right\\rbrack${"editStyle":"visual"}
%[text] the closed-loop dynamics are:
%[text]{"align":"center"} $\\dot{x} =\\textrm{Ax}+B\\left(-\\textrm{Kx}+K\_r r\\right)${"editStyle":"visual"}
%[text]{"align":"center"} $\\dot{x} =\\left(A-\\textrm{BK}\\right)x+\\left({\\textrm{BK}}\_r \\right)r${"editStyle":"visual"}
Kr = [K(3) K(4)];

% Define the controlled system
SYS_MF = ss(A-B*K, B*Kr, C, D);

% Reference h_d(t)
seg = t <= 15;
hd       = 100*exp(-t/5);
hd(~seg) = 20 - t(~seg); 

% Reference \dot{hd}(t)
hdot = gradient(hd, t);

% Reference signal r
r = [hdot.'  hd.'];

% Initial conditions
X0 = [ 0, asin(-16/256), -16, 120;
       0, asin(-20/256), -20, 100;
       0, asin(-24/256), -24, 80 ];

% Plot signals
f = figure(1);
set(f, 'Visible', 'on')
hold off;
set(f, 'Units', 'normalized');
set(f, 'Position', [0  0  0.2  1]);

subplot(3,1,1);
hold on; grid on;


for i = 1:3
    Y_MF = lsim(SYS_MF, r, t, X0(i,:)); 
    h_MF(:,i) = Y_MF(:,3);

    plot(t, h_MF(:,i), 'LineWidth', 1.5);
end

% Plot the altitude reference
plot(t, r(:,2), 'k--', 'LineWidth', 2);

ylim([-20 130]);

xlabel('Time [s]');
ylabel('Altitude [ft]');
legend('h(t) X0_1','h(t) X0_2','h(t) X0_3','Reference (r)','Location','Best');
title('Without Observer')
%[text] ### 3) Reduced-Order Observer
%[text] To build the reduced-order observer, a similarity transformation is used to place the system in the form:
%[text]{"align":"center"} $\\left\\lbrack \\begin{array}{c}\n\\dot{x\_m } \\\\\n\\dot{x\_d } \n\\end{array}\\right\\rbrack =\\underset{A\_{\\textrm{new}} }{\\underbrace{\\left\\lbrack \\begin{array}{cc}\nA\_m  & A\_{12} \\\\\nA\_{21}  & A\_d \n\\end{array}\\right\\rbrack } } \\left\\lbrack \\begin{array}{c}\nx\_m \\\\\nx\_d \n\\end{array}\\right\\rbrack +\\underset{B\_{\\textrm{new}} }{\\underbrace{\\left\\lbrack \\begin{array}{c}\nB\_m \\\\\nB\_d \n\\end{array}\\right\\rbrack } } u${"editStyle":"visual"}
%[text]{"align":"center"} $y=\\underset{C\_{\\textrm{new}} }{\\underbrace{\\left\\lbrack \\begin{array}{cc}\nI & 0\n\\end{array}\\right\\rbrack } } \\left\\lbrack \\begin{array}{c}\nx\_m \\\\\nx\_d \n\\end{array}\\right\\rbrack${"editStyle":"visual"}
%[text] This requires a transformation matrix $T${"editStyle":"visual"} to reorder the states so that
%[text]{"align":"center"} $x\_{\\textrm{new}} ={\\left\\lbrack \\begin{array}{cc}\nx\_m  & x\_d \n\\end{array}\\right\\rbrack }^T \\;\\;\\;\\;\\;\\longrightarrow \\;\\;\\;\\;\\;x\_{\\textrm{new}} ={\\left\\lbrack \\begin{array}{cccc}\nx\_1  & x\_2  & x\_4  & x\_3 \n\\end{array}\\right\\rbrack }^T${"editStyle":"visual"}
%[text] - $x\_m =${"editStyle":"visual"} Measured states
%[text] - $x\_d =${"editStyle":"visual"} Unmeasured states \
%[text] then, the transformed system is:
%[text]{"align":"center"} $A\_{\\textrm{new}} =\\textrm{TA}T^{-1} ,\\;\\;\\;B\_{\\textrm{new}} =\\textrm{TB},\\;\\;C\_{\\textrm{new}} =CT^{-1}${"editStyle":"visual"}
%[text] choosing T as:
%[text]{"align":"center"} $T=\\left\\lbrack \\begin{array}{c}\nC\\\\\n\\left\\lbrack \\begin{array}{cccc}\n0 & 0 & 1 & 0\n\\end{array}\\right\\rbrack \n\\end{array}\\right\\rbrack \\;=\\;\\left\\lbrack \\begin{array}{cccc}\n1 & 0 & 0 & 0\\\\\n0 & 1 & 0 & 0\\\\\n0 & 0 & 0 & 1\\\\\n0 & 0 & 1 & 0\n\\end{array}\\right\\rbrack${"editStyle":"visual"}
% Reorder into measured + unmeasured states
T = [C;[0 0 1 0]];
%T = [C;randn(1,4)]
ss_new = ss2ss(sys,T);

% Split the matrices (Am, A12, A21, Ad)
[n,m] = size(B);
[p,~] = size(C);

Am  = ss_new.A(1:p,1:p); 
A12 = ss_new.A(1:p,p+1:n);
A21 = ss_new.A(p+1:n,1:p); 
Ad  = ss_new.A(p+1:n,p+1:n);
Bm  = ss_new.B(1:p,1:m);
Bd  = ss_new.B(p+1:n,1:m);
%[text] The expression for the measured state is:
%[text]{"align":"center"} $\n\\\[\n\\begin{array}{|c|}\n\\hline\n\\qquad \\dot{x\_m } =A\_m x\_m +A\_{12} x\_d +B\_m u \\qquad \\\\\[2pt\]\n\\hline\n\\end{array}\n\\\]\n$
%[text] however, note that:
%[text]{"align":"center"} $y=\\underset{C\_{\\textrm{new}} }{\\underbrace{\\left\\lbrack \\begin{array}{cc}\nI & 0\n\\end{array}\\right\\rbrack } } \\left\\lbrack \\begin{array}{c}\nx\_m \\\\\nx\_d \n\\end{array}\\right\\rbrack \\;\\;\\;\\;\\;\\longrightarrow \\;\\;\\;\\;y=x\_m ,\\;\\;\\;\\textrm{logo}\\;\\;\\;\\;\\dot{y} =\\dot{x\_m }${"editStyle":"visual"}
%[text] and therefore:
%[text]{"align":"center"} $\\bar{y} =A\_{12} x\_d${"editStyle":"visual"}
%[text]{"align":"center"} $\\bar{y} =\\dot{y} \\;-A\_m y-B\_m u${"editStyle":"visual"}
%[text] the expression for the unmeasured state is
%[text]{"align":"center"} $\n\\\[\n\\begin{array}{|c|}\n\\hline\n\\qquad \\dot{x\_d } =A\_{21} x\_m +A\_d x\_d +B\_d u \\qquad \\\\\[2pt\]\n\\hline\n\\end{array}\n\\\]\n$
%[text] which can be rewritten as:
%[text]{"align":"center"} $\\dot{x\_d } =A\_d x\_d +\\bar{u}${"editStyle":"visual"}
%[text]{"align":"center"} $\\bar{u} =A\_{21} x\_m +B\_d u${"editStyle":"visual"}
%[text] from the manipulations above, the system becomes
%[text]{"align":"center"} $\\begin{array}{l}\n\\dot{x\_d } =A\_d x\_d +\\bar{u} \\\\\n\\bar{y} =A\_{12} x\_d \n\\end{array}${"editStyle":"visual"}
%[text] which has the form required for a full-order observer, therefore:
%[text]{"align":"center"} $\\dot{\\hat{x\_d } } =A\_d \\hat{x\_d } +\\bar{u\\;} +L\\left(\\bar{y} -C\\hat{x\_d } \\right)${"editStyle":"visual"}
%[text] expanding:
%[text]{"align":"center"} $\n\\\[\n\\begin{array}{|c|}\n\\hline\n\\qquad \\dot{\\hat{x\_d } } =\\left(A\_d -{\\mathrm{LA}}\_{12} \\right)\\hat{x\_d } +(A\_{21}-LA\_m)y+(B\_d-LB\_m)u\\qquad \\\\\[2pt\]\n\\hline\n\\end{array}\n\\\]\n$
%[text] it is clear that the problem reduces to pole placement for $A\_d -{\\textrm{LA}}\_{12}${"editStyle":"visual"}:
poles = eig(SYS_MF) % Dominant pole = -0.2916

% Place the observer pole at least 2x faster than the dominant pole
[L]=(place(Ad',A12', -0.6));
L=L'
%[text] ### 4) Closed-Loop Response with Observer
%[text] writing the system in matrix form:
%[text]{"align":"center"} $\\left\\lbrack \\begin{array}{c}\n\\dot{x\_m } \\\\\n\\dot{x\_d } \\\\\n\\dot{\\hat{x\_d } } \n\\end{array}\\right\\rbrack =\\left\\lbrack \\begin{array}{ccc}\nA\_m  & A\_{12}  & 0\\\\\nA\_{21}  & A\_d  & 0\\\\\nA\_{21}  & LA\_{12}  & A\_d -LA\_{12} \n\\end{array}\\right\\rbrack \\left\\lbrack \\begin{array}{c}\nx\_m \\\\\nx\_d \\\\\n\\hat{x\_d } \n\\end{array}\\right\\rbrack +\\left\\lbrack \\begin{array}{c}\nB\_m \\\\\nB\_d \\\\\nB\_d \n\\end{array}\\right\\rbrack u${"editStyle":"visual"}
%[text]{"align":"center"} 
Atot=[ Am  A12   zeros(p,n-p);
       A21 Ad    zeros(n-p,n-p);
       A21 L*A12 Ad-L*A12 ];
Btot=[ Bm; Bd; Bd ];
Ctot=T\[eye(p) zeros(p,n-p) zeros(p,n-p);zeros(n-p,p) zeros(n-p,n-p) eye(n-p)]; % identity matrix
Dtot= 0;
ss2_completo=ss(Atot,Btot,Ctot,Dtot);

Ttot = [T zeros(n,n-p); zeros(n-p,n) eye(n-p)];

% Final system - open loop
SYS_F_OL = ss2ss(ss2_completo, inv(Ttot));

% Final system - closed loop
SYS_F_CL = ss([SYS_F_OL.A - SYS_F_OL.B * K * SYS_F_OL.C], ...
             [SYS_F_OL.B * Kr], [SYS_F_OL.C], 0);

%[text] A deviation can be observed between the altitude curves of the system without observer and the system with the observer initialized at $x\_0 =0${"editStyle":"visual"}. This error is a direct consequence of the observer initial estimation error. 
%[text] In the original state-feedback system, the control law uses the real state vector $x\\left(t\\right)${"editStyle":"visual"}. In the reduced-order observer system, the law uses a state estimate, specifically the estimate of the unmeasured state $x\_{3\\left(\\right.} \\left(t\\right)${"editStyle":"visual"}. Since the observer is initialized with $x\_3 \\left(0\\right)=0${"editStyle":"visual"}, while in reality $x\_3 \\left(0\\right)\\;\\not= 0${"editStyle":"visual"} for the considered initial conditions, an initial estimation error is introduced.
%[text] Thus, the error observed in the plot is the transient effect of the estimation error. As the observer converges, this error vanishes and the observer-based response approaches the ideal full-state feedback response.
% Plot
subplot(3,1,2);
hold on; grid on;

% Plot signals
for i = 1:3
    Y_OBS0 = lsim(SYS_F_CL, r, t, [X0(i,:), 0]); 
    h_OBS0(:,i) = Y_OBS0(:,4);

    plot(t, h_OBS0(:,i), 'LineWidth', 1.5);
end

% Plot the reference
plot(t, r(:,2), 'k--', 'LineWidth', 2);

ylim([-20 130]);

xlabel('Time [s]');
ylabel('Altitude [ft]');
legend('h(t) X0_1','h(t) X0_2','h(t) X0_3','Reference (r)','Location','Best');
title('With Observer (X_0 = 0)')
%[text] ### 5) Observer Initialization Discussion
%[text] In practice, initializing the observer with a zero initial condition introduces an error:
%[text]{"align":"center"} $e\\left(0\\right)=x\_3 \\left(0\\right)-\\hat{x\_3 } \\left(0\\right)${"editStyle":"visual"}
%[text] For example, in the case where $x\_3 \\left(0\\right)=-20${"editStyle":"visual"}, we have:
%[text]{"align":"center"} $e\\left(0\\right)=-20-0=-20\\;\\textrm{ft}/s${"editStyle":"visual"}
%[text] this error directly affects the control law:
%[text]{"align":"center"} $u=-K\_1 x\_1 -K\_2 x\_2 -K\_3 \\left(\\hat{x\_3 } -\\dot{h\_d } \\right)-K\_4 \\left(x\_4 -h\_d \\right)${"editStyle":"visual"}
%[text] In a real implementation, this can create sharper initial control-surface commands, possible actuator saturation, acceleration peaks, and, in extreme cases, violations of safety envelopes.
%[text] 
%[text] Therefore, real applications try to initialize the observer as close as possible to the real state. In this case, one way to estimate the initial state is to use the relationships among the variables. From the assignment statement:
%[text]{"align":"center"} $\\Delta \\theta\_0 =\\frac{0\\ldotp 4}{102\\ldotp 4}\\dot{h\_o } \\;\\;\\;\\;\\;\\;\\longrightarrow \\;\\;\\;\\dot{h\_o } =256\\;\\Delta \\theta\_0 \\;${"editStyle":"visual"}
%[text] therefore, one way to initialize $\\hat{x\_3 } \\left(0\\right)${"editStyle":"visual"} is:
%[text]{"align":"center"} $\\hat{x\_3 } \\left(0\\right)=256{\\cdot \\;x}\_2 \\left(0\\right)${"editStyle":"visual"}
% Plot
subplot(3,1,3);
hold on; grid on;


% Plot signals
for i = 1:3
    Y_OBSinit = lsim(SYS_F_CL, r, t, [X0(i,:)  256*X0(i,2)]); 
    h_OBSinit(:,i) = Y_OBSinit(:,4);

    plot(t, h_OBSinit(:,i), 'LineWidth', 1.5);
end

% Plot the reference
plot(t, r(:,2), 'k--', 'LineWidth', 2);

ylim([-20 130]);

xlabel('Time [s]');
ylabel('Altitude [ft]');
legend('h(t) X0_1','h(t) X0_2','h(t) X0_3','Reference (r)','Location','Best');
title('With Observer (estimated X_0)')

%[text] 
% Comparative plot
f = figure(2);
set(f, 'Visible', 'on')
hold off;
set(f, 'Units','normalized', ...
               'Position', [0.20 0.60 0.8 0.40]); 

subplot(1,2,1);
hold on; grid on;

for i = 1:3
   c = co(i,:);

    plot(t, h_MF(:,i), 'LineWidth', 1.5, 'Color', c, 'LineStyle','-');
    plot(t, h_OBS0(:,i), 'LineWidth', 1.5, 'Color', c, 'LineStyle','--');
end

xlabel('Time [s]');
ylabel('Altitude [ft]');
legend('IC 1: $h(t)$',  'IC 1: $\hat{h}(t)$', ...
       'IC 2: $h(t)$',  'IC 2: $\hat{h}(t)$', ...
       'IC 3: $h(t)$',  'IC 3: $\hat{h}(t)$', ...
       'Location','best', 'Interpreter','latex');
title('Without Observer vs. With Observer (X_0 = 0)')

subplot(1,2,2);
hold on; grid on;

for i = 1:3
   c = co(i,:);

    plot(t, h_MF(:,i), 'LineWidth', 1.5, 'Color', c, 'LineStyle','-');
    plot(t, h_OBSinit(:,i), 'LineWidth', 1.5, 'Color', c, 'LineStyle','--');
end

xlabel('Time [s]');
ylabel('Altitude [ft]');
legend('IC 1: $h(t)$',  'IC 1: $\hat{h}(t)$', ...
       'IC 2: $h(t)$',  'IC 2: $\hat{h}(t)$', ...
       'IC 3: $h(t)$',  'IC 3: $\hat{h}(t)$', ...
       'Location','best', 'Interpreter','latex');
title('Without Observer vs. With Observer (X_0 estimated)')
%[text] ### 6) Nichols Chart
%[text] The Nichols chart shows that the transfer functions of the state-feedback system and the state-feedback system with reduced-order observer are almost identical. This follows from the separation principle; therefore, the observer does not significantly change the stability margins or the shape of the equivalent-loop Nichols diagram.
G_SYS = ss(A, B, K, 0);
G_SYS_obs = ss(SYS_F_OL.A, SYS_F_OL.B, K*SYS_F_OL.C, 0);

f = figure(3);
set(f, 'Visible', 'on')
hold on; grid on;

set(f, 'Units','normalized', ...
               'Position', [0.20 0.00 0.25 0.45]);

nichols(G_SYS);
nichols(G_SYS_obs);

grid

xlim([-225 -45]);
ylim([-30 30]);

title('Nichols - Without Observer vs With Observer');
legend('System without observer', ...
       'System with observer', ...
       'Location','Best', 'Interpreter','latex');
%[text] ### 
%[text] ### 7) Discretization - Nichols
%[text] A linear system has a general solution of the form:
%[text]{"align":"center"} $x\\left(t\\right)=\\sum\_i c\_i e^{\\lambda\_1 t}${"editStyle":"visual"}
%[text] therefore, the time constant for each real pole is:
%[text]{"align":"center"} $\\tau\_i =\\frac{1}{\\left|\\lambda\_i \\right|}${"editStyle":"visual"}
%[text] To choose the sampling period, the fastest system dynamics must be considered (the leftmost pole), therefore:
fast_pole = min(real(eig(SYS_F_CL))) % Fastest pole -3.4752
tau_rapido = 1/abs(fast_pole)
%[text] and the sampling period is selected as:
%[text]{"align":"center"} $T\_s \<t\_{\\textrm{rapido}}${"editStyle":"visual"}
%[text] for this system:
%[text]{"align":"center"} $T\_s \<0\\ldotp 2877${"editStyle":"visual"}
%[text] comparing the Nichols chart for different values of $\\tau\_s${"editStyle":"visual"}
Tvec = [0.05 0.1 0.2 0.3];

f = figure(4);
set(f, 'Visible', 'on')
hold on; grid on;

set(f, 'Units','normalized', ...
               'Position', [0.45 0.00 0.25 0.45]);

% System without discretization
nichols(G_SYS_obs);

% Nichols plot for different Ts values
for k = 1:length(Tvec)
    ts = Tvec(k);
    G_SYS_obs.OutputDelay = ts/2;
    nichols(G_SYS_obs);
end

grid
legend('System without delay', ...
       '$\tau_s$ = 0.05 s', ...
       '$\tau_s$ = 0.1 s', ...
       '$\tau_s$ = 0.2 s', ...
       '$\tau_s$ = 0.3 s', ...
       'Location','Best', 'Interpreter','latex');

xlim([-225 -45]);
ylim([-30 30]);
title('Nichols - with equivalent sampling delay');
%[text] For this work, the selected value was
%[text]{"align":"center"} $\\tau\_s =0\\ldotp 2${"editStyle":"visual"}
%[text] because it satisfies the criterion of being faster than the fastest pole and, as shown by the chart, preserves adequate gain and phase margins.
%[text] ### **8) Discretized Controller Simulation**
f = figure(5);
set(f, 'Visible', 'on')
hold on; grid on;

set(f, 'Units','normalized', ...
               'Position', [0.70 0.00 0.30 0.45]);

for i = 1:3
    IC_PLANT = X0(i,:);
    IC_X_HAT = 256*X0(i,2);

    % Discrete controller simulation
    simout = sim('DiscreteControllerR2024.slx');
    controle = simout.simulation.signals.values(:,1);
    saida = simout.simulation.signals.values(:,2);
    tempo = simout.simulation.time;

    % Plot the responses for each case
    c = co(i,:);
    plot(tempo, saida, 'LineWidth', 1.5, 'Color', c, 'LineStyle','-');
    plot(t, h_OBSinit(:,i), 'LineWidth', 1.5, 'Color', c, 'LineStyle','--');
end

title(['Discretized vs Continuous Controller Response']);
legend('Discrete: $x_{01}$',  'Continuous: $x_{01}$', ...
       'Discrete: $x_{02}$',  'Continuous: $x_{02}$', ...
       'Discrete: $x_{03}$',  'Continuous: $x_{03}$', ...
       'Location','best', 'Interpreter','latex');


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":35.7}
%---
