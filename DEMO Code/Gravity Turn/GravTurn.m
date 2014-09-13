%{
This code flies the gravity turn provided by the traj structure.
%}

%% Pre-Flihgt Check
if ~exist('traj','var')
    disp('traj structure not available, please run Example_11_03 first.')
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
pos0 = tread(t, 3, 'double');

twrite(t, uint8(17), 'uint8');
while t.bytesAvailable < 25
end
retcode = tread(t, 1, 'uint8');
vel0 = tread(t, 3, 'double');

% Get quaternion
twrite(t, uint8(18), 'uint8');
while t.bytesAvailable < 17
end
retcode = tread(t, 1, 'uint8');
q0 = tread(t, 4, 'single');
fprintf('Navigation:     ON\n')
pause(1)

%% Engage Engines
twrite(t, uint8(35), 'uint8');
while t.bytesAvailable < 1    
end
retcode = tread(t, 1, 'uint8');
fprintf('Engines:        ON\n')
pause(1)

%% Spool Up Engines
fprintf('\nLaucnhing...')
twrite(t, uint8(34), 'uint8');
twrite(t, single(1), 'single');
while t.bytesAvailable < 1    
end
retcode = tread(t, 1, 'uint8');

%% Get Time
twrite(t, uint8(21), 'uint8');
while t.bytesAvailable < 9
end
retcode = tread(t, 1, 'uint8');
T0 = tread(t, 1, 'double');

%% Launch
twrite(t, uint8(35), 'uint8');
while t.bytesAvailable < 1    
end
retcode = tread(t, 1, 'uint8');

%% Control Attitude
% Control gains
K = diag([1  1  1]);
C = diag([1 1 1]);
pitchover = false;
while 1
    % Get position
    twrite(t, uint8(16), 'uint8');
    while t.bytesAvailable < 25
    end
    retcode = tread(t, 1, 'uint8');
    pos = tread(t, 3, 'double');
    
    % Determine altitude
    alt = norm(pos)-norm(pos0);
    
    % Determine remaining fuel
    mf = interp1(traj.h,traj.mf,0.8*alt);
    
    % Determine launch stage
    if alt > traj.hturn && ~pitchover
        fprintf('\nBeginning gravity turn\n')
        pitchover = true;
    end
    if mf <= 0
        break
    end
    
    % Calculate target gamma
    gamma = interp1(traj.h,traj.gamma,alt)/180*pi;
    gamma_true = (pi/2-gamma)+real(acos((pos*pos0')/(norm(pos)*norm(pos0))));
    
    % Calculate command quaternion
    Cc = quat_rotation(q0,0);
    Cc = Rz(gamma_true)*Cc;
    qc = quat_from_c(Cc,0);
    
    % Get quaternion rotation
    twrite(t, uint8(18), 'uint8');
    while t.bytesAvailable < 17
    end
    retcode = tread(t, 1, 'uint8');
    q = tread(t, 4, 'single');
    
    % Get gyro rates
    twrite(t, uint8(15), 'uint8');
    while t.bytesAvailable < 13
    end
    retcode = tread(t, 1, 'uint8');
    w = tread(t, 3, 'single');
    pause(0.01)
    
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
    
    % Calculate target throttle
    T = interp1(traj.h,traj.T,alt);
    
    % Command throttle
    twrite(t, uint8(34), 'uint8');
    twrite(t, single(T), 'single');
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

% Seperate launch vehicle
twrite(t, uint8(35), 'uint8');
while t.bytesAvailable < 1    
end
retcode = tread(t, 1, 'uint8');

% Turn off auto-pilot
twrite(t, uint8(32), 'uint8');
twrite(t, uint8(0), 'uint8');
while t.bytesAvailable < 1    
end
retcode = tread(t, 1, 'uint8'); %#ok<*NASGU>
fprintf('Launch Complete\n')
pause(1)