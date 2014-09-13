%{
This code excecutes an energy centered impulse burn to transit between
Kerbin and Mun provided the spacecraft is in a co-planar orbit with Mun and
its orbit is circular.
%}

%% Pre-Flihgt Check
if ~exist('vessel','var')
    disp('vessel structure not available, please ~~~.')
    return
end

%% Open Communication
% Register COM port
t = tcpip('127.0.0.1', 25001, 'NetworkRole', 'server');
fopen(t);
fprintf('Communications: ON\n')
pause(5)

%% Engage Fly-By-Wire
% Turn on auto-pilot
twrite(t, uint8(32), 'uint8');
twrite(t, uint8(1), 'uint8');
while t.bytesAvailable < 1    
end
retcode = tread(t, 1, 'uint8'); %#ok<*NASGU>
fprintf('Computers:      ON\n')
pause(1)

%% Tare Navigation
% Get position
twrite(t, uint8(16), 'uint8');
while t.bytesAvailable < 25
end
retcode = tread(t, 1, 'uint8');
pos0    = tread(t, 3, 'double');

% Get velocity
twrite(t, uint8(17), 'uint8');
while t.bytesAvailable < 25
end
retcode = tread(t, 1, 'uint8');
vel0    = tread(t, 3, 'double');

% Get Mun position
twrite(t, uint8(23), 'uint8');
twrite(t, int32(0), 'int32');
while t.bytesAvailable < 25    
end
retcode = tread(t, 1, 'uint8');
MunPos0 = tread(t, 3, 'double');

% Get time
twrite(t, uint8(21), 'uint8');
while t.bytesAvailable < 9
end
retcode = tread(t, 1, 'uint8');
t0      = tread(t, 1, 'double');

fprintf('Navigation:     ON\n')
pause(1)

%% Engage Engines
twrite(t, uint8(34), 'uint8');
twrite(t, single(0), 'single');
while t.bytesAvailable < 1    
end
retcode = tread(t, 1, 'uint8');
fprintf('Engines:        ON\n')
pause(1)

%% Return Orbital Information
% Get angles between orbits
Munphase = acos(pos0*MunPos0'/(norm(pos0)*norm(MunPos0)));
Munlead  = acos(vel0*MunPos0'/(norm(vel0)*norm(MunPos0)));

% Correct if Mun is lagging
if abs(Munlead) > pi/2
    Munphase = -1*Munphase;
end

% Print data
fprintf('\nOrbital Information\n')
fprintf('Mun Phase:              %6.4g rad\n',Munphase)
fprintf('Mun Orbtial Radius:     %6.4g m\n',12000000)
fprintf('Mun Orbtial Period:     %6.0f s\n',138984)
fprintf('Craft Phase:            %6.4g rad\n',0)
fprintf('Craft Orbtial Altitude: %6.4g m\n',100000)
fprintf('Craft Orbtial Period:   %6.1f s\n',1958.1)
fprintf('Mission Time:           %6.4g s\n',0)
fprintf('Kerbin Radius:          %6.4g m\n',600000)
fprintf(['Standard Gravitational\n'...
         'Parameter (mu):        %6.5g (m^3)/(s^2)\n\n'],3.5316e12)

%% Get Burn Time and DeltaV
tTBurn = input('Please enter time till burn in seconds:');
deltav = input('Please enter deltaV of the burn in m/s:');

%% Calibrate Engines
% Get vessel mass
twrite(t, uint8(24), 'uint8');
while t.bytesAvailable < 5    
end
retcode = tread(t, 1, 'uint8');
mass0   = tread(t, 1, 'single');
v0      = norm(vel0);

% Simplify expressions
a = -vessel.maxThrust/(vessel.isp*9.81);
b = double(mass0*1000);
c = vessel.maxThrust;

% Get energy function
massfun = @(t) b+a*t;
vfun    = @(t) v0+c/a*(log(massfun(t)/b));
Efun    = @(t) 0.5*(massfun(t))*(vfun(t))^2;

% Pre-compute values
E0 = Efun(0);

% Get centered energy timing
tBurnTotal  = fminsearch(@(t) abs(vfun(t)-v0-deltav),0.5);
tHalfEnergy = fminsearch(@(t) abs(2*(Efun(t)-E0)-(Efun(tBurnTotal)-E0)),0.5);

% Calculate mission time at burn start
TruetTBurn = tTBurn-tHalfEnergy;

fprintf('\nEngines Calibrated\n')

%% Burn Control
% Control gains
K = diag([1 1 1]);
C = diag([1 1 1]);

% Burn phase
phase = 1;
fprintf('Waiting for burn window...\n')
UTruetTBurn = TruetTBurn+t0;
fprintf('Time Till Burn: %5.1f s\n',TruetTBurn)

while 1
    %% Burn Phase Control
    
    % Get time
    twrite(t, uint8(21), 'uint8');
    while t.bytesAvailable < 9
    end
    retcode = tread(t, 1, 'uint8');
    time    = tread(t, 1, 'double');
    
    switch phase
        case 1            
            % Update phase
            if time >= UTruetTBurn
                % Change phase
                fprintf('Burning...\n')
                phase = 2;
                tBurnStart = time;
                
                % Command throttle
                twrite(t, uint8(34), 'uint8');
                twrite(t, single(1), 'single');
                while t.bytesAvailable < 1
                end
                retcode = tread(t, 1, 'uint8');
            end
        case 2
            % Update phase
            if time >= tBurnStart+tBurnTotal
                % Change phase
                phase = 3;
                fprintf('Burn complete\n')
                
                % Command throttle
                twrite(t, uint8(34), 'uint8');
                twrite(t, single(0), 'single');
                while t.bytesAvailable < 1
                end
                retcode = tread(t, 1, 'uint8');
            end
        case 3
            % Get velocity
            twrite(t, uint8(17), 'uint8');
            while t.bytesAvailable < 25
            end
            retcode = tread(t, 1, 'uint8');
            vel     = tread(t, 3, 'double');
            
            % Check burn accuracy
            velerr = norm(vel0)+deltav-norm(vel);
            fprintf('\nReport\n')
            fprintf('Velocity Error: %6.1f m/s\n',velerr)
            
            break;
    end
    
    %% Attitude Control
    % Get velocity
    twrite(t, uint8(17), 'uint8');
    while t.bytesAvailable < 25
    end
    retcode = tread(t, 1, 'uint8');
    vel     = tread(t, 3, 'double');
    
    % Get quaternion rotation
    twrite(t, uint8(27), 'uint8');
    while t.bytesAvailable < 17
    end
    retcode = tread(t, 1, 'uint8');
    q = tread(t, 4, 'single');
    
    % Calculate command quaternion
    C0 = quat_rotation(q);
    r  = vrrotvec(C0'*[0 1 0]',Rx(pi/2)*vel');
    Ci = vrrotvec2mat(r);
    Cc = (Ci*C0')';
    qc = quat_from_c(Cc,0);
        
    % Get gyro rates
    twrite(t, uint8(15), 'uint8');
    while t.bytesAvailable < 13
    end
    retcode = tread(t, 1, 'uint8');
    w = tread(t, 3, 'single');
    
    % Control law
    qem = [ qc(1)  qc(4) -qc(3) -qc(2);
          -qc(4)  qc(1)  qc(2) -qc(3);
           qc(3) -qc(2)  qc(1) -qc(4);
           qc(2)  qc(3)  qc(4)  qc(1)];
    qep = qem*q([2 3 4 1])';
    qe  = qep(1:3);
    u = -K*qe-C*w';
    
    % Command rotations
    twrite(t,uint8(33), 'uint8');
    twrite(t,single(-u(2)), 'single');
    twrite(t,single(-u(1)), 'single');
    twrite(t,single(-u(3)), 'single');
    while t.bytesAvailable < 1
    end
    retcode = tread(t, 1, 'uint8');
    

end

%% End Flight
% Command throttle
twrite(t, uint8(34), 'uint8');
twrite(t, single(0), 'single');
while t.bytesAvailable < 1
end
retcode = tread(t, 1, 'uint8');

% Turn off auto-pilot
twrite(t, uint8(32), 'uint8');
twrite(t, uint8(0), 'uint8');
while t.bytesAvailable < 1    
end
retcode = tread(t, 1, 'uint8'); %#ok<*NASGU>
pause(1)