%%

clear 
clc
close all

mex -setup C++
addpath("Acquisizioni/")
addpath("Acquisizioni_12062025/Statica/")
addpath("Acquisizioni_12062025/Acquisizione1/")
addpath("Acquisizioni_12062025/Acquisizione2/")
addpath("Acquisizioni_12062025/Acquisizione post calib/")
addpath("EventBasedAlgorithm/")
addpath("Calibrazione_12062025/")
addpath('/usr/local/gtsam_toolbox')

import gtsam.*
import gtsam.symbol_shorthand.*

%% Load data

% Load imu and event data from file with drift to position
fileID = "due_giri.mat";
[data] = imucamData(fileID);
% Load calibration parameters from file
jsonFile = fileread('calibration_12062025.json');
[calibration] = readJsonFile(jsonFile);

%% Gravity and gyro bias from calibration

% Gravity vector from calibration, imu sdr [m/s^2]
gravity = [9.80621;-0.038735;-0.0722544];
g = norm(gravity);
% Gyroscope bias fom calibration [rad/s]
gyroBias =  [0.0158015;-0.0160041;0.0264244];
% Transformation matrix IMU->Camera
T_cam_imu = [ -0.999966   9.463e-05  -0.00821939  -0.0021056;
               7.506e-05   0.999997   0.00238107  -0.00548146;
               0.0082196   0.0023804  -0.999963   -0.0658417;
                      0          0           0           1 ];

%% Pre processing

% Camera data
data.events.timeStamp = double(data.events.timeStamp)*1e-6;                % [s]
data.events.x = double(data.events.x);                                     % [pixel]
data.events.y = double(data.events.y);                                     % [pixel]
data.frames.timeStamp = double(data.frames.timeStamp)*1e-6;                % [s]

% IMU data
data.imu.timeStamp = double(data.imu.timeStamp)*1e-6;                      % [s]
data.imu.timeStamp = data.imu.timeStamp-calibration.imu.timeShift;

data.imu.accX = double(data.imu.accX)*g;                                   % [m/s^2]
data.imu.accY = double(data.imu.accY)*g;                                   % [m/s^2]
data.imu.accZ = double(data.imu.accZ)*g;                                   % [m/s^2]
data.imu.gyroX = double(data.imu.gyroX)*(pi/180);                          % [rad/s]
data.imu.gyroY = double(data.imu.gyroY)*(pi/180);                          % [rad/s]
data.imu.gyroZ = double(data.imu.gyroZ)*(pi/180);                          % [rad/s]

% Remove duplicates 
[uniqueTimeStamp,uniqueidx] = unique(data.imu.timeStamp);
data.imu = data.imu(uniqueidx,:);

% Relative time from first frame acquisition
data.events.timeStamp = data.events.timeStamp-data.imu.timeStamp(1);       % cam data
data.frames.timeStamp = data.frames.timeStamp-data.imu.timeStamp(1);       % frames data
data.imu.timeStamp = data.imu.timeStamp-data.imu.timeStamp(1);             % imu data

%% Keyframe definition

% calcola norma velocità angolare dalla imu
gyroNorm = sqrt(data.imu.gyroX.^2 + data.imu.gyroY.^2 + data.imu.gyroZ.^2);
% soglia di moto
motionThreshold = deg2rad(5); 
inMotion = gyroNorm > motionThreshold;
idx_motion = find(inMotion);
if isempty(idx_motion)
    error("Non è stato rilevato alcun movimento.");
end
% intervallo di moto
t_start_motion = data.imu.timeStamp( idx_motion(1) );
t_end_motion   = data.imu.timeStamp( idx_motion(end) );

% Definizione intervallo tra keyframe nelle fasi statiche e di moto
dt_static  = 0.5;  % [s]
dt_motion  = 0.05; % [s]

% Fase statica iniziale
timeStatic1 = 0:dt_static:(t_start_motion - dt_static);
keyframeStatic1 = size(timeStatic1,2);
% Fase di moto
timeMotion = t_start_motion:dt_motion:t_end_motion;
keyframeMotion = size(timeMotion,2);
% Fase staica finale
timeStatic2 = (t_end_motion + dt_static):dt_static:data.imu.timeStamp(end);
keyframeStatic2 = size(timeStatic2,2);

% Vettore tempo totale
keyframe_time = [timeStatic1,timeMotion,timeStatic2];
% aggiorno variabile keyframe_num
keyframe_num = numel(keyframe_time);

% Visualise keyframe number
frameID = 229;
fprintf('keyframe ID %d \n',frameID);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%% FASE STATICA INIZIALE - IMU ONLY %%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Sistemi di riferiemento

% Media misure fase statica
staticmeas = 1000*t_start_motion;
meanAcc = [mean(data.imu.accX(1:staticmeas));mean(data.imu.accY(1:staticmeas));...
    mean(data.imu.accZ(1:staticmeas))];
meanGyro = [mean(data.imu.gyroX(1:staticmeas));mean(data.imu.gyroY(1:staticmeas));...
    mean(data.imu.gyroZ(1:staticmeas))];

% Definizione gravità
gravityWorld = [0;0;-g];                % gravità nel frame mondo
a = -meanAcc / norm(meanAcc);           % direzione gravità nel frame IMU
b = gravityWorld / norm(gravityWorld);  % direzione gravità nel frame Mondo

% Initial guess accelerometer bias
accBias = meanAcc + a*g;

% Rotation matrix IMU->World
R0 = body2worldRotationMatrix(b,a);

% Rotation and translastion form imu to cam
R_cam_imu = T_cam_imu(1:3,1:3);
t_cam_imu = T_cam_imu(1:3,4);
% Trasformation matrix from imu to cam
T_cam_imu_pose = Pose3(Rot3(R_cam_imu), Point3(t_cam_imu));
% Transformation matrix from cam to imu
T_imu_cam_pose = T_cam_imu_pose.inverse();

% Sistema mondo enu
T_world = eye(4);
% Sistema imu iniziale espresso in coordinate mondo
radius = [0.26;0;0];
T_imu_world = eye(4);
T_imu_world(1:3, 1:3) = R0;
T_imu_world(1:3,4) = radius;
% Sistema iniziale camera espresso nel mondo
T_cam_world = T_imu_world * inv(T_cam_imu);

%% Parametri factor graph

% Parametri del rumore
accNoiseDensity = 0.002;          % m/s^2 / sqrt(Hz)
gyroNoiseDensity = 0.00018;       % rad/s / sqrt(Hz)
accRandWalk = 4.0e-5;             % m/s^3 / sqrt(Hz)
gyroRandWalk = 0.001;             % rad/s^2 / sqrt(Hz)

% Matrici di covarianza
accCov = accNoiseDensity^2 * eye(3);
gyroCov = gyroNoiseDensity^2 * eye(3);
biasAccCov = accRandWalk^2 * eye(3);
biasOmegaCov = gyroRandWalk^2 * eye(3);

% Inizializzazione parametri del factor graph
params = PreintegrationCombinedParams.MakeSharedU(g);
params.setAccelerometerCovariance(accCov);
params.setGyroscopeCovariance(gyroCov);
params.setIntegrationCovariance(1e-3*eye(3));
params.setBiasAccCovariance(biasAccCov);
params.setBiasOmegaCovariance(biasOmegaCov);
params.getUse2ndOrderCoriolis();
% Consider the transformation between camera and imu
params.setBodyPSensor(Pose3(Rot3(R_cam_imu), Point3(t_cam_imu)));

%% Inizializzazione factor graph

initialEstimate = Values;
graph = NonlinearFactorGraph();

% Posa iniziale imu nel sistema mondo
Pose_imu0_world = Pose3(Rot3(R0), Point3(radius));
% Posa iniziale camera nel sistema mondo
Pose_cam0_world = Pose_imu0_world.compose(T_imu_cam_pose);
Pose_cam0_world = Pose3(Rot3(Pose_cam0_world.rotation().matrix()), Point3(radius));

% Inserimento values iniziali nel grafo
initialPose = Pose_cam0_world;
initialVelocity = Point3(0,0,0);
initialBias = imuBias.ConstantBias(Point3(accBias),Point3(gyroBias));
initialEstimate.insert(P(0), initialPose);
initialEstimate.insert(V(0), initialVelocity);
initialEstimate.insert(B(0), initialBias);

%% Predizione e creazione fattori imu

for i = 1:keyframeStatic1-1
    j = i + 1;
    % Estrazione misure tra keyframe
    idx = data.imu.timeStamp >= timeStatic1(i) & data.imu.timeStamp <= timeStatic1(j);
    times = data.imu.timeStamp(idx);
    accMeas = [data.imu.accX(idx), data.imu.accY(idx), data.imu.accZ(idx)];
    accMeas = accMeas';
    gyroMeas = [data.imu.gyroX(idx), data.imu.gyroY(idx), data.imu.gyroZ(idx)];
    gyroMeas = gyroMeas';

    if numel(times) < 2
        warning("Keyframe %d → %d: troppo pochi dati IMU", i, j);
        continue
    end

    dtMeas = diff(times);
    biasPrev = initialEstimate.atConstantBias(B(i-1));
    pim = PreintegratedCombinedMeasurements(params, biasPrev);
    
    for k = 1:length(dtMeas)
        pim.integrateMeasurement(accMeas(:,k), gyroMeas(:,k), dtMeas(k));
    end
    
    % Predizione nuovo stato
    pose_i = initialEstimate.atPose3(P(i-1));
    vel_i = initialEstimate.atVector(V(i-1));
    nav_i = NavState(pose_i, vel_i);
    nav_j = pim.predict(nav_i, biasPrev);
    pose_j = nav_j.pose();
    vel_j = nav_j.velocity();
    % Inserimento nuovo stato nel grafo
    initialEstimate.insert(P(i), pose_j);
    initialEstimate.insert(V(i), vel_j);
    initialEstimate.insert(B(i), biasPrev);
    % Aggiunta fattori imu
    factor = CombinedImuFactor(P(i-1), V(i-1), P(i), V(i), B(i-1), B(i), pim);
    graph.add(factor);
    % Reset preintegrazione
    pim.resetIntegrationAndSetBias(biasPrev);

end

%% Aggiunta fattori di prior

% Prior sulla posa iniziale
priorPoseNoise = noiseModel.Diagonal.Sigmas(1e-2 * ones(6,1));
priorVelocityNoise = noiseModel.Diagonal.Sigmas(1e-1 * ones(3,1));
priorBiasNoise = noiseModel.Diagonal.Sigmas(10* ones(6,1));
graph.add(PriorFactorPose3(P(0), initialPose, priorPoseNoise));
graph.add(PriorFactorVector(V(0), initialVelocity, priorVelocityNoise));
graph.add(PriorFactorConstantBias(B(0), initialBias, priorBiasNoise));

% Prior sulla fase statica iniziale completa

for i = 1:2:keyframeStatic1
    priorPoseNoise = noiseModel.Diagonal.Sigmas(1e-2*ones(6,1));
    priorVelocityNoise = noiseModel.Diagonal.Sigmas(1e-1 * ones(3,1));
    priorBiasNoise = noiseModel.Diagonal.Sigmas(10* ones(6,1));
    graph.add(PriorFactorPose3(P(i),initialPose, priorPoseNoise));
    graph.add(PriorFactorVector(V(i), initialVelocity, priorVelocityNoise));
    graph.add(PriorFactorConstantBias(B(i), initialBias, priorBiasNoise));
end

%% Optimisation

optimizer = LevenbergMarquardtOptimizer(graph,initialEstimate);
result = optimizer.optimizeSafely();
% Check optimisation results
errorBeforeOpt = graph.error(initialEstimate);
errorAfterOpt = graph.error(result);
disp(['Error before optimization: ', num2str(errorBeforeOpt)]);
disp(['Error after optimization: ', num2str(errorAfterOpt)]);

% Salvo le pose in un oggetto values col quale ricostruirò la triaettoria
% finale
finalEstimate = Values;
% Inserimento dei risultati 
for i = 0:keyframeStatic1-1
    finalEstimate.insert(P(i),result.atPose3(P(i)));
    finalEstimate.insert(V(i),result.atVector(V(i)));
    finalEstimate.insert(B(i),result.atConstantBias(B(i)));
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%% FASE DI MOTO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Costruzione della time surface

% Memory allocation
S_on = cell(keyframeMotion,1);
S_off = cell(keyframeMotion,1);
Tp_on = cell(keyframeMotion,1);
Tp_off = cell(keyframeMotion,1);
Tp = cell(keyframeMotion,1);
Tnp = cell(keyframeMotion,1);
numEventsInWindow = zeros(keyframeMotion,1);

% Time Surface parameters
events = data.events;
resolution = calibration.camera.resolution; 
tau = 0.001;     % exponential decay [s]
delta = 0.025;   % sliding window [s]
Nmax = 50000;    % max events
Nmin = 1000;     % min events

% Build Time surfaces
for i = 1:keyframeMotion

    % keyframe timestamp
    t = timeMotion(i);
    % build time surfaces
    [Tp_on{i},Tp_off{i},Tp{i},Tnp{i},S_on{i},S_off{i},numEventsInWindow(i)] = buildTimeSurfaces(events,t,resolution,tau,delta,Nmax,Nmin);

end

%% Detect Arc* corners

corners = cell(keyframeMotion,1);  
keyframeValid = false(keyframeMotion,1);

% ARC* Algorithm parameteres
radius = 5;
numCirclePoints = 16;
minConsecutive = 4;
threshold = 0.02;
minDistance = 4;

for i = 1:keyframeMotion

    % detect corner on Tp
    arcCorners = detectArcCorners(Tp_on{i}, Tp_off{i}, S_on{i}, S_off{i}, ...
        radius, numCirclePoints, minConsecutive, threshold, minDistance);
    if ~isempty(arcCorners)
        corners{i} = arcCorners;     
        keyframeValid(i) = true;
    else
        corners{i} = [];
        keyframeValid(i) = false;   
    end

end

%% LKT Optical flow

% Memory allocation
validKeyframes = [];
droppedKF = [];
% Optical flow parameters
trackLength = 5;
maxError = 0.5;
trackedFeatures = cell(keyframeMotion-trackLength, 1);

for i = 1:(keyframeMotion-trackLength)

    if isempty(corners{i})
        droppedKF = [droppedKF, i];
        continue;
    end
    % Current keyframe corners
    cornerPoints = corners{i}(:,1:2);  % [x, y]
    % Initialise tracker
    tracker = vision.PointTracker('MaxBidirectionalError', maxError);
    initialize(tracker,cornerPoints,Tnp{i});
    % Save validity and tracked positions
    validMask = true(size(cornerPoints,1), 1);
    pointTracks = cornerPoints;
    % Track corners on the (tarckLength-1) next keyframes
    for j = 1:(trackLength-1)
        nextIdx = i + j;
        [points, validity] = step(tracker, Tnp{nextIdx});
        % Update only the still valid corners 
        validMask = validMask & validity;
        pointTracks = cat(3, pointTracks, points);  % Nx2xT
    end
    % Fiter only the valid ones in all keyframes
    validIdx = find(validMask);
    trackedPoints = pointTracks(validIdx, :, :);  % Nx2xT
    % Save track
    if numel(validIdx) < 8
        droppedKF = [droppedKF,i];
        continue;
    else
        trackedFeatures{i} = trackedPoints;
    end    
end

%% Definizione finestre di ottimizzazione sull base delle features 

% calcola numero di features tracciate per keyframe
numTracks = zeros(length(trackedFeatures),1);
for i=1:length(trackedFeatures)
    if ~isempty(trackedFeatures{i})
        numTracks(i) = size(trackedFeatures{i},1);
    end
end

% definizione finestre adattative (considero finestre di 50 keyframe con
% ovelap di 20)
minTracks = 20;
windowSize = 50;
overlap = 20;
step = windowSize-overlap;
windows = [];
i = 1;
while i < keyframeMotion-windowSize
    windowIdx = i:(i+windowSize-1);
    if mean(numTracks(windowIdx)) > minTracks
        windows = [windows; i i+windowSize-1];
        i = i + windowSize - overlap;  % scorro con overlap
    else
        i = i + 10; % trascuro se troppo povero
    end
end
disp("Finestre definite:")
disp(windows)

%% Ricostruzione della traiettoria a finestre imu-only

% Non considero le windows, ma considero dal primo keyframe del moto in poi
% sempre le finestre costanti di 50 con overlap di 20

for w = 0:step:(keyframeMotion-windowSize)

    % Definizione intervalli finestre con indici riferiti ai keyframeMotion
    globalStart = keyframeStatic1+w;
    globalEnd = keyframeStatic1+w+windowSize-1;
    % Intervallo di tempo della finestra
    timeWindow = timeMotion(globalStart:globalEnd);

    % Creazione nuovo grafo e nuovo oggetto values
    graph = NonlinearFactorGraph();
    initialEstimate = Values;

    if w == 0
        % Prima finestra:
        % Recupero stato iniziale dall'ultimo result della fase statica
        initialPose = finalEstimate.atPose3(P(keyframeStatic1-1));
        initialVelocity = finalEstimate.atVector(V(keyframeStatic1-1));
        initialBias = finalEstimate.atConstantBias(B(keyframeStatic1-1));
        % Definizione dello stato iniziale per il nuovo grafo
        initialEstimate.insert(P(0), initialPose);
        initialEstimate.insert(V(0), initialVelocity);
        initialEstimate.insert(B(0), initialBias);
        % Inserisco prior morbido
        graph.add(PriorFactorPose3(P(0), initialPose, noiseModel.Diagonal.Sigmas(1*ones(6,1))));
    else
       % Recupero come stato iniziale le pose dell'overlap
       for j = 1:overlap
           globalIdx = globalStart+j-1;
           pose_j = finalEstimate.atPose3(P(globalIdx));
           velocity_j = finalEstimate.atVector(V(globalIdx));
           bias_j = finalEstimate.atConstantBias(B(globalIdx));
           % Definizione dello stato iniziale per il nuovo grafo
           initialEstimate.insert(P(j-1),pose_j);
           initialEstimate.insert(V(j-1),velocity_j);
           initialEstimate.insert(B(j-1),bias_j);
           % Inserisco prior più forti
           graph.add(PriorFactorPose3(P(j-1),pose_j, noiseModel.Diagonal.Sigmas(1e-2*ones(6,1))));
           graph.add(PriorFactorVector(V(j-1),velocity_j, noiseModel.Diagonal.Sigmas(1e-1*ones(3,1))));
           graph.add(PriorFactorConstantBias(B(j-1),bias_j, noiseModel.Diagonal.Sigmas(1*ones(6,1))));
       end
    end

    % Predizione e creazione fattori imu
    if w == 0
        predStart = 1;
    else
        predStart = overlap;
    end

    for i = predStart:windowSize-1
        j = i + 1;
        % Estrazione misure tra keyframe
        idx = data.imu.timeStamp >= timeWindow(i) & data.imu.timeStamp <= timeWindow(j);
        times = data.imu.timeStamp(idx);
        accMeas = [data.imu.accX(idx), data.imu.accY(idx), data.imu.accZ(idx)];
        accMeas = accMeas';
        gyroMeas = [data.imu.gyroX(idx), data.imu.gyroY(idx), data.imu.gyroZ(idx)];
        gyroMeas = gyroMeas';
    
        if numel(times) < 2
            warning("Keyframe %d → %d: troppo pochi dati IMU", i, j);
            continue
        end
    
        dtMeas = diff(times);
        biasPrev = initialEstimate.atConstantBias(B(i-1));
        pim = PreintegratedCombinedMeasurements(params, biasPrev);
        
        for k = 1:length(dtMeas)
            pim.integrateMeasurement(accMeas(:,k), gyroMeas(:,k), dtMeas(k));
        end
        
        % Predizione nuovo stato
        pose_i = initialEstimate.atPose3(P(i-1));
        vel_i = initialEstimate.atVector(V(i-1));
        nav_i = NavState(pose_i, vel_i);
        nav_j = pim.predict(nav_i, biasPrev);
        pose_j = nav_j.pose();
        vel_j = nav_j.velocity();
        % Inserimento nuovo stato nel grafo
        initialEstimate.insert(P(i), pose_j);
        initialEstimate.insert(V(i), vel_j);
        initialEstimate.insert(B(i), biasPrev);
        % Aggiunta fattori imu
        factor = CombinedImuFactor(P(i-1), V(i-1), P(i), V(i), B(i-1), B(i), pim);
        graph.add(factor);
        % Reset preintegrazione
        pim.resetIntegrationAndSetBias(biasPrev);
    
    end

    % Aggiunta fattori di prior
    % prior sulla velocità tangenziale nel piano XY del mondo
    v_mag = 0.0544;
    priorVelocityNoise = noiseModel.Diagonal.Sigmas(1*ones(3,1));
    for i = predStart:5:windowSize-1
        pose_i = initialEstimate.atPose3(P(i));
        R_w_c = pose_i.rotation().matrix();  % rotazione world<-camera
        dir_in_world = R_w_c(:,3);           % asse Z camera, visto nel mondo
        % azzera la componente verticale (in world ENU la quota è Z)
        dir_in_world(3) = 0;
        dir_in_world = dir_in_world / norm(dir_in_world);  % tangente sul piano XY
        v_i = Point3(dir_in_world * v_mag);
        graph.add(PriorFactorVector(V(i), v_i, priorVelocityNoise));
    end

    % Ottimizzazione finestra
    
    optimizer = LevenbergMarquardtOptimizer(graph,initialEstimate);
    result = optimizer.optimizeSafely();
    % Check optimisation results
    errorBeforeOpt = graph.error(initialEstimate);
    errorAfterOpt = graph.error(result);
    disp(['Error before optimization: ', num2str(errorBeforeOpt)]);
    disp(['Error after optimization: ', num2str(errorAfterOpt)]);

    for j = 0:(windowSize-1)
        globalIdx = globalStart + j;
        if finalEstimate.exists(P(globalIdx))
            finalEstimate.update(P(globalIdx), result.atPose3(P(j)));
            finalEstimate.update(V(globalIdx), result.atVector(V(j)));
            finalEstimate.update(B(globalIdx), result.atConstantBias(B(j)));
        else
            finalEstimate.insert(P(globalIdx), result.atPose3(P(j)));
            finalEstimate.insert(V(globalIdx), result.atVector(V(j)));
            finalEstimate.insert(B(globalIdx), result.atConstantBias(B(j)));
        end
    end

end

% Calcolo quanti keyframe rimangono da stimare
% Npose_ottimizzate = double(finalEstimate.size() / 3);
Npose_ottimizzate = 1152;
keyframeRimasti = (keyframeStatic1 + keyframeMotion) - Npose_ottimizzate;

if keyframeRimasti <= 0
    disp("Nessun keyframe rimanente da processare");
else
    % dimensione della finestra
    newKeyframesFinal = keyframeRimasti;
    windowSize = 1 + newKeyframesFinal;   % 1 di partenza + i rimanenti

    % nuovo grafo
    graph = NonlinearFactorGraph();
    initialEstimate = Values;

    % Inserisco il punto finale di finalEstimate come iniziale del nuovo
    % grafo
    idxLast = Npose_ottimizzate - 1;
    pose_last = finalEstimate.atPose3(P(idxLast));
    vel_last  = finalEstimate.atVector(V(idxLast));
    bias_last = finalEstimate.atConstantBias(B(idxLast));
    initialEstimate.insert(P(0), pose_last);
    initialEstimate.insert(V(0), vel_last);
    initialEstimate.insert(B(0), bias_last);
    % prior forte su questo punto iniziale
    graph.add(PriorFactorPose3(P(0), pose_last, noiseModel.Diagonal.Sigmas(1e-2*ones(6,1))));
    graph.add(PriorFactorVector(V(0), vel_last, noiseModel.Diagonal.Sigmas(1e-1*ones(3,1))));
    graph.add(PriorFactorConstantBias(B(0), bias_last, noiseModel.Diagonal.Sigmas(1*ones(6,1))));

    % tempo dei keyframe da processare
    windowsStart = keyframeMotion - keyframeRimasti + 1;
    windowsEnd   = keyframeMotion;
    timeWindow   = timeMotion(windowsStart-1:windowsEnd);

    % predStart
    predStart = 1;  % dopo il nodo iniziale

    % ciclo di predizione e preintegrazione
    for i = predStart:(windowSize-1)
        local_i = i;        % su timeWindow
        local_j = i + 1;
        if local_j > numel(timeWindow)
            break;
        end
        idx = data.imu.timeStamp >= timeWindow(local_i) & data.imu.timeStamp <= timeWindow(local_j);
        times = data.imu.timeStamp(idx);
        accMeas  = [data.imu.accX(idx), data.imu.accY(idx), data.imu.accZ(idx)]';
        gyroMeas = [data.imu.gyroX(idx), data.imu.gyroY(idx), data.imu.gyroZ(idx)]';

        if numel(times) < 2
            warning("pochi dati IMU tra keyframe %d → %d", local_i, local_j);
            continue
        end

        dtMeas = diff(times);
        biasPrev = initialEstimate.atConstantBias(B(i-1));
        pim = PreintegratedCombinedMeasurements(params, biasPrev);

        for k = 1:length(dtMeas)
            pim.integrateMeasurement(accMeas(:,k), gyroMeas(:,k), dtMeas(k));
        end

        pose_i = initialEstimate.atPose3(P(i-1));
        vel_i  = initialEstimate.atVector(V(i-1));
        nav_i  = NavState(pose_i, vel_i);
        nav_j  = pim.predict(nav_i, biasPrev);

        pose_j = nav_j.pose();
        vel_j  = nav_j.velocity();

        initialEstimate.insert(P(i), pose_j);
        initialEstimate.insert(V(i), vel_j);
        initialEstimate.insert(B(i), biasPrev);

        graph.add(CombinedImuFactor(P(i-1), V(i-1), P(i), V(i), B(i-1), B(i), pim));
    end

    % prior di velocità tangenziale sul piano xy
    v_mag = 0.0544;
    priorVelocityNoise = noiseModel.Diagonal.Sigmas(1*ones(3,1));
    for i = predStart:5:(windowSize-1)
        pose_i = initialEstimate.atPose3(P(i));
        R_w_c  = pose_i.rotation().matrix();
        dir_in_world = R_w_c(:,3);
        dir_in_world(3) = 0;
        dir_in_world = dir_in_world / norm(dir_in_world);
        v_i = Point3(dir_in_world * v_mag);
        graph.add(PriorFactorVector(V(i), v_i, priorVelocityNoise));
    end

    % ottimizzazione
    optimizer = LevenbergMarquardtOptimizer(graph,initialEstimate);
    result = optimizer.optimizeSafely();

    disp("Error before optimization: " + graph.error(initialEstimate));
    disp("Error after optimization: " + graph.error(result));

    % salvataggio solo degli ultimi 29
    for j = 1:(windowSize - 1)
        globalIdx = idxLast + j;
        finalEstimate.insert(P(globalIdx), result.atPose3(P(j)));
        finalEstimate.insert(V(globalIdx), result.atVector(V(j)));
        finalEstimate.insert(B(globalIdx), result.atConstantBias(B(j)));
    end
end



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%% FINESTRE VISUAL LANDMARK OPTIMIZATION %%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% parametri di proiezione
focalLength = calibration.camera.focalLength;
principalPoint = calibration.camera.principalPoint;
resolution = calibration.camera.resolution;
radialDistortion = calibration.camera.distortionCoefficients(1:2);
tangentialDistortion = calibration.camera.distortionCoefficients(3:4);
intrinsics = cameraIntrinsics(focalLength, principalPoint, resolution, ...
    'RadialDistortion', radialDistortion, ...
    'TangentialDistortion', tangentialDistortion);

K = Cal3_S2(focalLength(1), focalLength(2), 0, principalPoint(1), principalPoint(2));
measNoise = noiseModel.Isotropic.Sigma(2, 10.0);

landmarkIdCounter = 1;

for w = 1:size(windows,1)

    win_start = windows(w,1);
    win_end   = windows(w,2);

    fprintf("Ottimizzazione landmark per finestra %d (%d-%d)\n", w, win_start, win_end);

    % nuovo grafo locale
    graph = NonlinearFactorGraph();
    initialEstimate = Values;

    % inserisco le pose già ottimizzate dall'IMU
    for i = win_start:win_end
        pose_i = finalEstimate.atPose3(P(i));
        vel_i  = finalEstimate.atVector(V(i));
        bias_i = finalEstimate.atConstantBias(B(i));
        initialEstimate.insert(P(i), pose_i);
        initialEstimate.insert(V(i), vel_i);
        initialEstimate.insert(B(i), bias_i);

        % prior molto stretti (pose + vel + bias)
        graph.add(PriorFactorPose3(P(i), pose_i, noiseModel.Diagonal.Sigmas(1e-3*ones(6,1))));
        graph.add(PriorFactorVector(V(i), vel_i, noiseModel.Diagonal.Sigmas(1e-2*ones(3,1))));
        graph.add(PriorFactorConstantBias(B(i), bias_i, noiseModel.Diagonal.Sigmas(1e-1*ones(6,1))));
    end

    % selezione baseline nella finestra
    baselineFound = false;
    for k = win_start:(win_end-trackLength)
        tracks = trackedFeatures{k};
        if isempty(tracks), continue; end
        pts1 = squeeze(tracks(:,:,1));
        pts2 = squeeze(tracks(:,:,end));
        if size(pts1,1) < 20, continue; end
        disp = pts2 - pts1;
        parallax = sqrt(sum(disp.^2,2));
        if mean(parallax) < 15, continue; end

        [E,inliersIdx] = estimateEssentialMatrix(pts1, pts2, intrinsics, ...
            'Confidence',99,'MaxNumTrials',5000);
        if nnz(inliersIdx)<20, continue; end

        baselineFound = true;
        kf1 = k;
        kf2 = k+trackLength-1;
        fprintf("Baseline trovata nella finestra %d: %d → %d\n",w,kf1,kf2);
        break
    end

    if ~baselineFound
        warning("Nessuna baseline nella finestra %d",w);
        continue
    end

    % triangolazione landmark
    inliers1 = pts1(inliersIdx,:);
    inliers2 = pts2(inliersIdx,:);

    pose1 = finalEstimate.atPose3(P(kf1));
    pose2 = finalEstimate.atPose3(P(kf2));

    % world->camera
    R1 = pose1.rotation().matrix();
    t1 = pose1.translation();
    R2 = pose2.rotation().matrix();
    t2 = pose2.translation();

    % ortonormalizzazione
    [U,~,Vv] = svd(R1); R1=U*Vv'; if det(R1)<0, R1(:,3)=-R1(:,3); end
    [U,~,Vv] = svd(R2); R2=U*Vv'; if det(R2)<0, R2(:,3)=-R2(:,3); end

    R_cw1 = R1';
    t_cw1 = -R_cw1*t1;
    R_cw2 = R2';
    t_cw2 = -R_cw2*t2;

    Proj1 = cameraProjection(intrinsics,rigidtform3d(R_cw1,t_cw1));
    Proj2 = cameraProjection(intrinsics,rigidtform3d(R_cw2,t_cw2));

    % undistort
    inliers1_ud = undistortPoints(inliers1,intrinsics);
    inliers2_ud = undistortPoints(inliers2,intrinsics);
    landmarks = triangulate(inliers1_ud,inliers2_ud,Proj1,Proj2);

    % inserisci landmark come valori iniziali
    for j=1:size(landmarks,1)
        lid = landmarkIdCounter;
        landmarkIdCounter = landmarkIdCounter + 1;
        initialEstimate.insert(L(lid), Point3(landmarks(j,:)'));
        % aggiungi fattori di proiezione
        graph.add(GenericProjectionFactorCal3_S2(Point2(inliers1_ud(j,1),inliers1_ud(j,2)), ...
            measNoise, P(kf1), L(lid), K));
        graph.add(GenericProjectionFactorCal3_S2(Point2(inliers2_ud(j,1),inliers2_ud(j,2)), ...
            measNoise, P(kf2), L(lid), K));
    end

    % puoi opzionalmente aggiungere anche i keyframe intermedi
    for k = kf1+1:kf2-1
        if isempty(trackedFeatures{k}), continue; end
        points_k = squeeze(trackedFeatures{k}(:,:,1));
    end

    % ottimizzazione
    optimizer = LevenbergMarquardtOptimizer(graph,initialEstimate);
    result = optimizer.optimizeSafely();

    % aggiorna finalEstimate
    % for i = win_start:win_end
    %     finalEstimate.update(P(i), result.atPose3(P(i)));
    %     finalEstimate.update(V(i), result.atVector(V(i)));
    %     finalEstimate.update(B(i), result.atConstantBias(B(i)));
    % end

    % aggiorna landmark globali
    for lid = 1:(landmarkIdCounter-1)
        if result.exists(L(lid))
            finalEstimate.insert(L(lid), result.atPoint3(L(lid)));
        end
    end

    fprintf("Landmark optimization finestra %d terminata.\n",w);

end











%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%FASE STATICA FINALE - IMU ONLY%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% nuovo grafo per la fase statica finale
graph = NonlinearFactorGraph();
initialEstimate = Values;

% nodo iniziale dalla fine della fase di moto
idxLast = keyframeStatic1+keyframeMotion-1;  % indice globale
pose_last = finalEstimate.atPose3(P(idxLast));
vel_last  = finalEstimate.atVector(V(idxLast));
bias_last = finalEstimate.atConstantBias(B(idxLast));
initialEstimate.insert(P(0), pose_last);
initialEstimate.insert(V(0), vel_last);
initialEstimate.insert(B(0), bias_last);
% prior forte sul primo nodo
graph.add(PriorFactorPose3(P(0), pose_last, noiseModel.Diagonal.Sigmas(1e-2*ones(6,1))));
graph.add(PriorFactorVector(V(0), vel_last, noiseModel.Diagonal.Sigmas(1e-1*ones(3,1))));
graph.add(PriorFactorConstantBias(B(0), bias_last, noiseModel.Diagonal.Sigmas(1*ones(6,1))));

% predizione per la fase statica finale
for i = 1:keyframeStatic2-1
    j = i + 1;

    idx = data.imu.timeStamp >= timeStatic2(i) & data.imu.timeStamp <= timeStatic2(j);
    times = data.imu.timeStamp(idx);
    accMeas  = [data.imu.accX(idx), data.imu.accY(idx), data.imu.accZ(idx)]';
    gyroMeas = [data.imu.gyroX(idx), data.imu.gyroY(idx), data.imu.gyroZ(idx)]';

    if numel(times) < 2
        warning("Pochi dati IMU tra keyframe %d → %d", i, j);
        % in caso continuo a copiare la predizione
        pose_j = initialEstimate.atPose3(P(i-1));
        vel_j  = initialEstimate.atVector(V(i-1));
    else
        dtMeas = diff(times);
        biasPrev = initialEstimate.atConstantBias(B(i-1));
        pim = PreintegratedCombinedMeasurements(params, biasPrev);
        for k = 1:length(dtMeas)
            pim.integrateMeasurement(accMeas(:,k), gyroMeas(:,k), dtMeas(k));
        end
        % predizione
        pose_i = initialEstimate.atPose3(P(i-1));
        vel_i  = initialEstimate.atVector(V(i-1));
        nav_i  = NavState(pose_i, vel_i);
        nav_j  = pim.predict(nav_i, biasPrev);
        pose_j = nav_j.pose();
        vel_j  = nav_j.velocity();
        graph.add(CombinedImuFactor(P(i-1), V(i-1), P(i), V(i), B(i-1), B(i), pim));
    end

    % inserisco nodo successivo
    initialEstimate.insert(P(i), pose_j);
    initialEstimate.insert(V(i), vel_j);
    initialEstimate.insert(B(i), bias_last);
end

% prior su ogni nodo statico
priorPoseNoise = noiseModel.Diagonal.Sigmas(1e-2*ones(6,1));
priorVelocityNoise = noiseModel.Diagonal.Sigmas(1e-1*ones(3,1));
for i = 0:2:keyframeStatic2-1
    graph.add(PriorFactorPose3(P(i), pose_last, priorPoseNoise));
    graph.add(PriorFactorVector(V(i), Point3(0,0,0), priorVelocityNoise));
end

% ottimizzazione
optimizer = LevenbergMarquardtOptimizer(graph,initialEstimate);
result = optimizer.optimizeSafely();

% inserisco i risultati in finalEstimate
for i = 0:keyframeStatic2-1
    globalIdx = idxLast + 1 + i;
    finalEstimate.insert(P(globalIdx), result.atPose3(P(i)));
    finalEstimate.insert(V(globalIdx), result.atVector(V(i)));
    finalEstimate.insert(B(globalIdx), result.atConstantBias(B(i)));
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%% PLOT FINALI %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot result trajectory

poses_x = zeros(keyframe_num,1);
poses_y = zeros(keyframe_num,1);
poses_z = zeros(keyframe_num,1);
landmarks_x = [];
landmarks_y = [];
landmarks_z = [];

% Estrai le pose dal risultato ottimizzato
for i = 0:keyframe_num-1
    pose_i = finalEstimate.atPose3(P(i));
    translation = pose_i.translation();
    poses_x(i+1) = translation(1);
    poses_y(i+1) = translation(2);
    poses_z(i+1) = translation(3);
end
% Estrazione landmarks
for lid = 0:landmarkIdCounter-1
    key = L(lid);
    if finalEstimate.exists(key)
        pt = finalEstimate.atPoint3(key);
        landmarks_x = [landmarks_x; pt(1)];
        landmarks_y = [landmarks_y; pt(2)];
        landmarks_z = [landmarks_z; pt(3)];
    end
end

% Plot della traiettoria 3D
figure()
plot3(poses_x, poses_y, poses_z, '*-', 'LineWidth', 1);
grid on; hold on;
scatter3(landmarks_x, landmarks_y, landmarks_z, 20, 'r', 'filled','o','MarkerEdgeColor','k');
xlabel('X [m]','FontSize',14);
ylabel('Y [m]','FontSize',14);
zlabel('Z [m]','FontSize',14);
title('Optimized trajectory and landmarks','FontSize',16);
set(gca, 'FontSize', 12); % aumenta la dimensione di tutti i tick labels
legend('Optimized trajectory','Landmarks','FontSize',12)
% axis equal

%% Plot biases

biasGyro = zeros(3,keyframe_num);  
biasAcc = zeros(3,keyframe_num);    

% Extract bias results after optimisation
for i = 1:keyframe_num
    estBias = finalEstimate.atConstantBias(B(i-1));
    % Accelerometer bias
    biasAcc(:,i) = estBias.accelerometer;
    % Gyroscope bias
    biasGyro(:,i) = estBias.gyroscope;
end

% Time vector
tKeyframe = 0:keyframe_num-1;
% Plot 
figure()
subplot(2,1,1);
plot(tKeyframe, biasAcc',LineWidth=1);
title('Optimized accelerometer bias','FontSize',16);
xlabel('Time [s]','FontSize',14);
ylabel('Bias [m/s^2]','FontSize',14);
legend('X','Y','Z','FontSize',12);
grid on;
subplot(2,1,2);
plot(tKeyframe, biasGyro',LineWidth=1);
title('Optimized gyroscope bias','FontSize',16);
xlabel('Time [s]','FontSize',14);
ylabel('Bias [rad/s]','FontSize',14);
legend('X','Y','Z','FontSize',12);
set(gca, 'FontSize', 12); % aumenta la dimensione di tutti i tick labels
grid on;

%% Plot velocity

velocities = zeros(3, keyframe_num);
for i = 0:keyframe_num-1
    velocities(:, i+1) = finalEstimate.atVector(V(i));
end

figure;
plot(tKeyframe, velocities','LineWidth',1);
xlabel('Time [s]','FontSize',14);
ylabel('Velocity [m/s]','FontSize',14);
legend('Vx','Vy','Vz','FontSize',12);
title('Optimized linear velocity','FontSize',16);
set(gca, 'FontSize', 12); % aumenta la dimensione di tutti i tick labels
grid on;

%% Plot euler angles

% Prealloca array per assetto
roll  = zeros(keyframe_num, 1);
pitch = zeros(keyframe_num, 1);
yaw   = zeros(keyframe_num, 1);

for i = 0:keyframe_num-1
    pose_i = finalEstimate.atPose3(P(i));
    R = pose_i.rotation().matrix();  % matrice 3x3
    rot = Rot3(R);

    % Estrai angoli di Eulero (ZYX convention → yaw-pitch-roll)
    angles = rot.rpy();  % ritorna [roll; pitch; yaw]
    roll(i+1)  = angles(1);
    pitch(i+1) = angles(2);
    yaw(i+1)   = angles(3);

end
yaw_deg = rad2deg(yaw);
yaw_deg_unwrapped = unwrap(deg2rad(yaw_deg));

figure()
plot(tKeyframe, rad2deg(roll), 'LineWidth', 1); hold on;
plot(tKeyframe, rad2deg(pitch), 'LineWidth', 1);
plot(tKeyframe, rad2deg(yaw),'--','LineWidth', 1);
plot(tKeyframe, rad2deg(yaw_deg_unwrapped),'LineWidth',1);
legend('Roll [°]', 'Pitch [°]', 'Yaw [°]','True Yaw [°]','FontSize',12);
xlabel('Time [s]','FontSize',14);
ylabel('Angle [°]','FontSize',14);
title('Camera attitude - Euler angles','FontSize',16);
set(gca, 'FontSize', 12); % aumenta la dimensione di tutti i tick labels
grid on;

%% Plot groundtruth, attidue and result trajectory

r = 0.26;  % radius

% Grountruth
theta = linspace(0,4*pi,keyframe_num);
groundtruth = [r*cos(theta);r*sin(theta);zeros(1,keyframe_num)];

% Preallocazione
traj_attitude = zeros(3, keyframe_num);
for i = 0:keyframe_num-1
    pose_i = finalEstimate.atPose3(P(i));        % Posa stimata (camera o IMU)
    R_i = pose_i.rotation().matrix();     % rotation matrix
    radial_dir = R_i(:,1);   % X direction in ENU
    traj_attitude(:, i+1) = r * radial_dir;
end

% Traiettoria originale dal grafo
traj_opt = zeros(3, keyframe_num);
for i = 0:keyframe_num-1
    pose_i = finalEstimate.atPose3(P(i));
    t = pose_i.translation();
    traj_opt(:, i+1) = [t(1); t(2); t(3)];
end

figure;
plot3(traj_opt(1,:), traj_opt(2,:), traj_opt(3,:), '-*', 'LineWidth', 1);
hold on
plot3(traj_attitude(1,:), traj_attitude(2,:), traj_attitude(3,:), '-o', 'LineWidth', 1);
plot3(groundtruth(1,:), groundtruth(2,:), groundtruth(3,:), '-d', 'LineWidth', 1);
grid on; 
axis equal;
xlabel('X [m]', 'FontSize', 14); 
ylabel('Y [m]', 'FontSize', 14); 
zlabel('Z [m]', 'FontSize', 14);
legend('Optimized trajectory', 'Trajectory from attitude', 'Groundtruth', 'FontSize', 12);
title('Trajectory comparison', 'FontSize', 16);
set(gca, 'FontSize', 12); % aumenta la dimensione di tutti i tick labels