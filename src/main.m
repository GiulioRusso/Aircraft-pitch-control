clear all
close all
clc

%% Systems Theory project 2021-2022
% Pitch control of an aircraft in steady cruise
% Course: Systems Theory 2021-22, Prof. Gianluca Antonelli
% Author: Giulio Russo

%% Linear model
%{

    The model describes the longitudinal motion of the aircraft in steady
    cruise (straight flight at constant altitude and speed). It is the
    linearised pitch model of the CTMS "Aircraft Pitch" example
    (University of Michigan).

    The full equations of motion are a set of six coupled nonlinear
    differential equations; under the steady-cruise assumption they are
    decoupled and linearised into the longitudinal equations below, with:

    alpha = angle of attack: angle between the aircraft reference line
    (the chord line of the wing) and the velocity vector of the aircraft
    relative to the air

    q = pitch rate (normalised: dteta/dt = omega*q)

    teta = pitch angle: angle between the aircraft longitudinal axis and
    the horizontal

    delta = elevator deflection angle. It is the control input

    mu = rho*S*c/(4*m), depends on air density rho, wing planform area S,
    mean aerodynamic chord c and aircraft mass m

    omega = 2*U/c, depends on the equilibrium flight speed U and the mean
    chord c

    CD = drag coefficient

    CL = lift coefficient

    CW = weight coefficient

    CM = pitching moment coefficient

    gamma = flight path angle (teta - alpha)

    iyy = normalised moment of inertia about the pitch axis

    sigma = 1/(1 + mu*CL), depends on mu and CL

    eta = mu*sigma*CM, depends on mu, sigma and CM



    With numerical values from experimental data we obtain the model:

    dalpha   -0.313   56.7    0   alpha   0.232
    dq     = -0.0139  -0.426  0 * q     + 0.0203 * delta
    dteta     0       56.7    0   teta    0

                alpha
    y = 0 0 1 * q     + 0 * delta
                teta

%}

% State (dynamic) matrix
A=[
    -0.313      56.7       0
    -0.0139    -0.426      0
     0          56.7       0
   ];

% Input matrix
B=[
    0.232
    0.0203
    0
   ];

% Output matrix
C=[0 0 1]; % measure teta

% Feedthrough matrix
D=0;

% Initial conditions
deg=15;
x0=[0 0 deg*pi/180]'; % [alpha q teta]
%{
    teta starts at 15 degrees, a typical take-off pitch attitude
%}

% Noise
odg_w=1e-4;
%{
    Order of magnitude of the white Gaussian noise on the output
%}





%% Task 1 - Eigenvalues, time constants, reachability and observability

fprintf("\n Eigenvalues of the state matrix \n");
% Eigenvalues of the state matrix
eigA=eig(A)
%{
     0.0000 + 0.0000i
    -0.3695 + 0.8860i
    -0.3695 - 0.8860i

    >> eig(A)=roots(poly(A))

    Eigenvalues of the state matrix:
    the system has three eigenvalues. A zero eigenvalue, associated with a
    constant natural mode, and two complex conjugate eigenvalues with
    negative real part, associated with two sinusoidal natural modes
    enveloped by an exponential that converges to zero

    A system is stable if all its modes are bounded: there is one zero
    eigenvalue associated with a constant natural mode, and two eigenvalues
    with negative real part associated with two sinusoidal modes enveloped
    by a converging exponential

    The zero eigenvalue must be checked for regularity

    a.m. (algebraic multiplicity) = 1
    g.m. (geometric multiplicity) = n-rank(A-lambda*eye(n))
    >> mg0=3-rank(A-0*eye(3))
    >> ans = 1

    Since 1<=gm(lambda)<=am(lambda)<=n, if the a.m. is one the g.m. is
    also one, so the eigenvalue is regular



    Transfer function (the poles of the transfer function are the roots of
    the characteristic polynomial, obtainable with poly(A)):
    >> [num,den] = ss2tf(A,B,C,D)
    >> num =  0         0    1.1510    0.1774
    >> den = 1.0000    0.7390    0.9215         0
    one pole at zero and all other poles with negative real part, which
    further confirms stability



    Free response: in open_loop.slx we apply a constant input
    corresponding to an equilibrium state. The input is zero (for linear
    systems the equilibrium does not depend on which constant input is
    applied). With this input the system is at equilibrium. Perturbing it
    with a nonzero du for a time T (e.g. 3 seconds) the output settles to
    a finite value. Starting from the perturbation of the equilibrium a
    forced response is triggered. When the perturbation is switched off,
    the system evolves freely with zero input and the initial conditions
    left by the forced response. Since the response settles to a finite
    value, the system is stable

    Therefore the system is simply (marginally) stable: the regular zero
    eigenvalue (the pitch angle integrates the pitch rate) prevents
    asymptotic stability
%}
% Plot the eigenvalues
ALLFUNCS.Plot_eig('tc',eigA);
pause;

fprintf("\n Time constant \n");
% Time constant of the system
tau=-1/min(real(eigA))
% Final simulation time
tf=1.5*4*tau;
%{
    min(.) picks the fastest mode, so its time constant is the smallest one

    1.5*4 = look a bit beyond 4 tau to catch any drift
%}

fprintf("\n REACHABILITY MATRIX \n")
% Reachability matrix
R=ctrb(A,B)
% Check whether the system is completely reachable
ALLFUNCS.VerificaProprieta(R, "reachable");

fprintf("\n OBSERVABILITY MATRIX \n")
% Observability matrix
O=obsv(A,C)
% Check whether the system is completely observable
ALLFUNCS.VerificaProprieta(O, "observable");





%% Task 2 - State feedback controller with a 5 s time constant
fprintf("\n\n\n State feedback \n");

fprintf("\n Discretised model \n");
% Discretisation parameters
f=10; % samples per second
T=1/f; % seconds per sample
% Discretise the model (forward Euler: A_d = I + T*A, B_d = T*B)
A_d=eye(3,3)+T*A
B_d=T*B
C_d=C

fprintf("\n Eigenvalues of the state matrix \n");
% Eigenvalues of the state matrix
eigA_d=eig(A_d)
alphai_d=min(real(eigA_d));
%{
    With two eigenvalues of modulus < 1 and one of modulus = 1 the system
    is simply (marginally) stable
%}
% Plot the eigenvalues
ALLFUNCS.Plot_eig('td',eigA_d);
pause;

fprintf("\n Time constant \n");
% Time constant of the system
tau_d=floor(-1/min(log(abs(eigA_d)))) % steps
% Final simulation time
tf_d=floor(1.5*4*tau_d); % total simulation steps
k_d=1:tf_d; % vector of simulation steps
%{
    min(.) picks the fastest mode, so its time constant is the smallest one

    1.5*4 = look a bit beyond 4 tau to catch any drift
%}

fprintf("\n REACHABILITY MATRIX \n")
% Reachability matrix
R_d=ctrb(A_d,B_d)
% Check whether the system is completely reachable
ALLFUNCS.VerificaProprieta(R_d, "reachable after discretisation");

fprintf("\n OBSERVABILITY MATRIX \n")
% Observability matrix
O_d=obsv(A_d,C_d)
% Check whether the system is completely observable
ALLFUNCS.VerificaProprieta(O_d, "observable after discretisation");

% Required time constant
tau_rs=5; % seconds
tau_rs_d=tau_rs*f; % steps
% Final simulation time
tf_rs_d=floor(1.5*4*tau_rs_d); % total simulation steps
k_rs=1:tf_rs_d; % vector of simulation steps
% Initial conditions of the system
x0_rs=[0 0 deg*pi/180]';  % [alpha q teta]
% Desired eigenvalues
ldes_abs_rs=exp(-1/tau_rs_d);
ldes_rs=[ldes_abs_rs ldes_abs_rs-0.0001 ldes_abs_rs-0.0002]
%{
    The eigenvalues are spaced apart because place has numerical problems
    placing several eigenvalues at the same point. From an engineering
    point of view, moving them slightly changes nothing, given how tau is
    defined and the approximation already in the model

    Assigning eigenvalues all with modulus less than one gives
    asymptotically stable dynamics, so the state is expected to converge
    to zero
%}
% Plot the desired eigenvalues
ALLFUNCS.Plot_eig('td',eigA_d,ldes_rs');
pause;

% Compute the feedback gain
[Kr_rs]=ALLFUNCS.RetroazioneStato(A_d,B_d,C_d,ldes_rs);

x_rs(:,1)=x0_rs;
for i=1:tf_rs_d
    u_rs(:,i)=Kr_rs*x_rs(:,i);
    y_rs(:,i)=C_d*x_rs(:,i)+odg_w*normrnd(0,1);
    if(i<tf_rs_d)
        x_rs(:,i+1)=A_d*x_rs(:,i)+B_d*u_rs(:,i);
    end
end

% Plot state feedback
ALLFUNCS.Plot_rs(k_rs,x_rs,y_rs,true);
pause;





%% Task 3 - Output feedback with 0.05 rad reference, zero initial conditions and 100 ms sampling time
fprintf("\n\n\n Output feedback \n");

% Desired output
yd=0.05;
% Initial conditions
x0_ru=[0 0 0]'; % [alpha q teta]
% Desired eigenvalues
dec_ru=1.175; % speed-up of the time constant relative to the system tau
tau_ru_des=floor(tau_d/dec_ru); % corresponding desired tau
ldes_abs_ru=exp(-1/tau_ru_des); % corresponding abs of ldes
ldes_ru=[ldes_abs_ru ldes_abs_ru-0.0001 ldes_abs_ru-0.0002] % faster than eigA_d
%{
    In discrete time, faster eigenvalues must lie further left than the
    open-loop ones, but not too close to 0. There is no direct
    proportionality as in continuous time, because of the logarithmic
    relation between tau and the modulus of lambda
%}
% Plot the eigenvalues
ALLFUNCS.Plot_eig('td',eigA_d,ldes_ru');
pause;

% Compute the feedback gain
[Kr_ru]=ALLFUNCS.RetroazioneStato(A_d,B_d,C_d,ldes_ru);

% Compute the feedforward gain
[Ky]=ALLFUNCS.RetroazioneUscitaCicloAperto(A_d,B_d,C_d,Kr_ru);

x_ru(:,1)=x0_ru;
for i=1:tf_d
    u_ru(:,i)=(yd*Ky)+Kr_ru*x_ru(:,i);
    y_ru(:,i)=C_d*x_ru(:,i)+odg_w*normrnd(0,1);
    if(i<tf_d)
        x_ru(:,i+1)=A_d*x_ru(:,i)+B_d*u_ru(:,i);
    end
end

% Plot output feedback
ALLFUNCS.Plot_ru(k_d,x_ru,y_ru,true);
pause;





%% Task 4.1 - Deterministic observer
fprintf("\n\n\n Deterministic observer (Luenberger) \n");

% Initial conditions of the system
x0_od=[0 0 deg*pi/180]; % [alpha q teta]
% Required initial conditions of the observer
x0hat_od=[0.05 0 0.01]'; % [alpha q teta]

% Desired observer eigenvalues
dec_od=2; % speed-up of the time constant relative to the system tau
tau_od_des=floor(tau_d/dec_od); % corresponding desired tau
ldes_abs_od=exp(-1/tau_od_des); % corresponding abs of ldes
ldes_od=[ldes_abs_od ldes_abs_od-0.0001 ldes_abs_od-0.0002]
% Desired controller eigenvalues
ldes_odr=[(alphai_d+ldes_od(1))/2 (alphai_d+ldes_od(1))/2-0.0001 (alphai_d+ldes_od(1))/2-0.0002]
%{
    The observer eigenvalues must be the fastest. Ideally the observer is
    a sensor, so it should be instantaneous: faster than the system and
    also faster than the feedback loop
%}
% Plot the eigenvalues
ALLFUNCS.Plot_eig('td',eigA_d,ldes_odr',ldes_od');
pause;

% Observer
[Ko_od]=ALLFUNCS.OsservatoreDeterministico(A_d,C_d,ldes_od);

% Controller
[Kr_od]=ALLFUNCS.RetroazioneStato(A_d,B_d,C_d,ldes_odr);

x_od(:,1)=x0_od;
xhat_od(:,1)=x0hat_od;
for i=1:tf_d
    u_od(:,i)=Kr_od*xhat_od(:,i); % feed back the state estimate
    y_od(:,i)=C_d*x_od(:,i)+odg_w*normrnd(0,1);
    if(i<tf_d)
        x_od(:,i+1)=A_d*x_od(:,i)+B_d*u_od(:,i);
        xhat_od(:,i+1)=A_d*xhat_od(:,i)+B_d*u_od(:,i)+Ko_od*(y_od(:,i)-C_d*xhat_od(:,i));
    end
end

% Plot deterministic observer
ALLFUNCS.Plot_od(k_d,x_od,xhat_od,y_od,true);
pause;





%% Task 4.2 - Stochastic observer
fprintf("\n\n\n Stochastic observer (Kalman) \n");

% Desired controller eigenvalues
ldes_k=ldes_odr;
% Plot the eigenvalues
%ALLFUNCS.Plot_eig('td',eigA_d,ldes_k');
%pause;

% Controller
[Kr_k]=ALLFUNCS.RetroazioneStato(A_d,B_d,C_d,ldes_k);

% Initial conditions
x0_k=x0_od; % [alpha q teta]
% Kalman filter initialisation
xhat_k(:,1)=[0.05 0 0.01]'; % a posteriori estimate
xhats_k(:,1)=xhat_k(:,1); % a priori estimate
% Statistical parameters
Ps=[1e-1 0 0; 0 1e-3 0; 0 0 1e-1]; % estimation error covariance
R_w=[1e-1 0 0; 0 1e-3 0; 0 0 1e-1]; % process noise covariance
R_v=1e3; % measurement noise covariance

% Controller-observer
[x_k,xhat_k,y_k,K_k,Phist,Khist]=ALLFUNCS.OsservatoreStocastico(tf_d,Kr_k,xhat_k(:,1),xhats_k(:,1),x0_k,A_d,B_d,C_d,Ps,R_v,odg_w,R_w);

% Plot Kalman
ALLFUNCS.Plot_Kalman(k_d,x_k,xhat_k,y_k,Phist',Khist',true);