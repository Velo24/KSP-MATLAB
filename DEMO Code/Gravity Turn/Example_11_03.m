% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
function Example_11_03(T2W,hturn,tscale)
% ~~~~~~~~~~~~~~~~~~~~~~
%{
This program numerically integrates Equations 11.6 through
11.8 for a gravity turn trajectory.
User M-functions required: rkf45
User subfunction requred: rates

Inputs~~~~~~~~~~~~~~~~~~
T2W     Target thrust to weight of the launch.

hturn   Height at which pitchover begins (m).

tscale  Time scale which control duration of time to be simulated.
~~~~~~~~~~~~~~~~~~~~~~~~
%}
% ----------------------------------------------
deg     = pi/180;           % ...Convert degrees to radians
g0      = 9.81;             % ...Sea-level acceleration of gravity (m/s)
Re      = 600e3;            % ...Radius of the earth (m)
hscale  = 5e3;              % ...Density scale height (m)
rho0    = 1.225;            % ...Sea level density of atmosphere (kg/m^3)
mu      = 3.5316e12;        % ...Standard gravitational parameter (m^3/s^2)
p0      = 1;                % ...Sea level pressure (atm)

CDA0    = 3.8452;           % ...Drag coefficient (assumed constant)
m0      = 83.33*1000;       % ...Lift-off mass (kg)
Max_T   = 1500;             % ...Maximum trhust (kN)
n       = 83.09/19.33;      % ...Mass ratio
Ispmin  = 320;              % ...Specific impulse at sea level (s)
Ispmax  = 360;              % ...Specific impulse in vaccum (s)

mfinal  = m0/n;             % ...Burnout mass (kg)
Thrust  = T2W*m0*g0;        % ...Rocket thrust (N)
m_dot0  = Thrust/Ispmin/g0; % ...Propellant mass flow rate (kg/s)
mprop   = m0 - mfinal;      % ...Propellant mass (kg)
tburn   = mprop/m_dot0;     % ...Burn time (s)
t0      = 0;                % ...Initial time for the numerical integration
tf      = tscale*tburn;     % ...Final time for the numerical integration
tspan   = [t0,tf];          % ...Range of integration

% ...Initial conditions:
v0      = 0;                % ...Initial velocity (m/s)
gamma0  = 89*deg;           % ...Initial flight path angle (rad)
x0      = 0;                % ...Initial downrange distance (km)
h0      = 0;                % ...Initial altitude (km)
vD0     = 0;                % ...Initial value of velocity loss due to drag (m/s)
vG0     = 0;                % ...Initial value of velocity loss due to gravity (m/s)

%...Check that target T2W is possible
if 1000*Max_T/(m0*g0) < T2W
    disp('Target Thrust to weight not possible.')
    return
elseif T2W <= 1
    disp('Target thrust to weight must be greater than 1.')
    return
end

%...Initial conditions vector:
f0 = [v0; gamma0; x0; h0; vD0; vG0; m0];

%...Call to Runge-Kutta numerical integrator 'rkf45'
% rkf45 solves the system of equations df/dt = f(t):

[t,f] = ode45(@rates, tspan, f0);

%...t is the vector of times at which the solution is evaluated
%...f is the solution vector f(t)
%...rates is the embedded function containing the df/dt's

%...Solution f(t) returned on the time interval [t0 tf]:
v       = f(:,1)*1.e-3;     % ...Velocity (km/s)
gamma   = f(:,2)/deg;       % ...Flight path angle (degrees)
x       = f(:,3)*1.e-3;     % ...Downrange distance (km)
h       = f(:,4)*1.e-3;     % ...Altitude (km)
vD      = -f(:,5)*1.e-3;    % ...Velocity loss due to drag (km/s)
vG      = -f(:,6)*1.e-3;    % ...Velocity loss due to gravity (km/s)
m       = f(:,7);           % ...Vessel mass (tons)

%...Determine if rocket fully rotated
i_return = find(gamma < 0);
if isempty(i_return)
    disp('Rocket did not fully rotate, gravity turn incomplete.')
    output2
    return
end

i_peak = i_return(1);
if gamma(i_peak-1) < 0
    disp('Something went wrong...')
    output2
    return
end

i_peak = i_peak-1;

%...Calculate remaining deltaV to enter circular orbit
v2orb = sqrt(mu/(Re+h(i_peak)*1000))-v(i_peak)*1000;

%...Correct gamma values for output
gamma(h < hturn/1000) = 90;

%...Calculate fuel wieght
mf = m-mfinal;

for i = 1:length(t)
    Rho  = rho0 * exp(-h(i)*1000/hscale); %...Air density
    q(i) = 1/2*Rho*v(i)^2;                %#ok<NASGU,AGROW>
                                         %...Dynamic pressure
end

output
return

    %~~~~~~~~~~~~~~~~~~~~~~~~~
    function dydt = rates(~,y)
    %~~~~~~~~~~~~~~~~~~~~~~~~~
    % Calculates the time rates df/dt of the variables f(t)
    % in the equations of motion of a gravity turn trajectory.
    %-------------------------
        
        v     = y(1); % ...Velocity
        gamma = y(2); % ...Flight path angle
        x     = y(3); % ...Downrange distance
        h     = y(4); % ...Altitude
        vD    = y(5); % ...Velocity loss due to drag
        vG    = y(6); % ...Velocity loss due to gravity
        m     = y(7); % ...Mass of ship
        
        CDA = (m-mfinal)*0.2;                     % ...Kerbal Calculates drag wierd
        g   = g0/(1 + h/Re)^2;                    % ...Gravitational variation with altitude h
        
        if h > 70000
            p = 0;                                % ...Atmosphere cut-off in kerbal
        else
            p = p0 * exp(-h/hscale);              % ...Exponential pressure variation with altitude
        end
        
        rho = rho0*p;                             % ...Density with pressure
        Isp = Ispmin+(Ispmax-Ispmin)*(1-p);       % ...Specific impulse
        D   = 1/2 * rho*v^2 * 0.008 * (CDA0+CDA); % ...Drag [Equation 11.1]
        
        %...When time t exceeds the burn time, set the thrust
        % and the mass flow rate equal to zero:
        if mfinal < m
            T     = T2W*m*g0;  % ...Current thrust
            m_dot = -T/Isp/g0; % ...Propellant mass flow rate
        else
            T     = 0; % ...Current thrust
            m_dot = 0; % ...Propellant mass flow rate
        end
        
        %...Define the first derivatives of v, gamma, x, h, vD and vG
        % ("dot" means time derivative):
        %v_dot = T/m - D/m - g*sin(gamma); % ...Equation 11.6
        
        %...Start the gravity turn when h = hturn:
        if h <= hturn && gamma == gamma0
            gamma_dot = 0;
            v_dot     = T/m - D/m - g;
            x_dot     = 0;
            h_dot     = v;
            vG_dot    = -g;
        else
            v_dot     = T/m - D/m - g*sin(gamma);
            gamma_dot = -1/v*(g - v^2/(Re + h))*cos(gamma); % ...Equation 11.7
            x_dot     = Re/(Re + h)*v*cos(gamma);           % ...Equation 11.8(1)
            h_dot     = v*sin(gamma);                       % ...Equation 11.8(2)
            vG_dot    = -g*sin(gamma);                      % ...Gravity loss rate
        end
        
        vD_dot = -D/m; % ...Drag loss rate
        
        %...Load the first derivatives of f(t) into the vector dfdt:        dydt(1) = v_dot;
        dydt(1) = v_dot;
        dydt(2) = gamma_dot;
        dydt(3) = x_dot;
        dydt(4) = h_dot;
        dydt(5) = vD_dot;
        dydt(6) = vG_dot;
        dydt(7) = m_dot;
        dydt = dydt';
    end

    %~~~~~~~~~~~~~~
    function output
    %~~~~~~~~~~~~~~
        fprintf('\n\n -----------------------------------\n')
        fprintf('\n        Pitchover altitude = %10g m ',hturn)
        fprintf('\n                 Burn time = %10g s ',t(i_peak))
        fprintf('\n               Final speed = %10g km/s',v(i_peak))
        fprintf('\n Remaining deltaV to orbit = %10g m/s',v2orb)
        fprintf('\n                  Altitude = %10g km ',h(i_peak))
        fprintf('\n        Downrange distance = %10g km ',x(i_peak))
        fprintf('\n                 Drag loss = %10g km/s',vD(i_peak))
        fprintf('\n              Gravity loss = %10g km/s',vG(i_peak))
        fprintf('\n\n -----------------------------------\n')
        
        figure(1)
        subplot(2,1,1)
        plot(x, h)
        xlabel('Downrange Distance (km)')
        ylabel('Altitude (km)')
        axis fill
        grid
        
        subplot(2,1,2)
        plot(h, v)
        xlabel('Altitude (km)')
        ylabel('Speed (km/s)')
        axis([-inf, inf, -inf, inf])
        grid
        
        traj.gamma = gamma(1:i_peak);
        traj.h     = h(1:i_peak)*1000;
        traj.T     = T2W*m(1:i_peak)*(1e-3)*g0/Max_T;
        traj.hturn = hturn;
        traj.mf    = mf(1:i_peak);
        assignin('base','traj',traj)
        
    end %output

    %~~~~~~~~~~~~~~
    function output2
    %~~~~~~~~~~~~~~        
        figure(1)
        subplot(2,1,1)
        plot(x, h)
        axis equal
        xlabel('Downrange Distance (km)')
        ylabel('Altitude (km)')
        axis([-inf, inf, 0, inf])
        grid
        
        subplot(2,1,2)
        plot(h, v)
        xlabel('Altitude (km)')
        ylabel('Speed (km/s)')
        axis([-inf, inf, -inf, inf])
        grid
        
    end %output2

end %Example_11_03
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~