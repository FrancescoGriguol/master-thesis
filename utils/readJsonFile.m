function [calibration] = readJsonFile(jsonFile)
% -------------------------------------------------------------------------
% Function che legge il file .json di calibrazione e crea una struttura più
% semplice che contiene solo i dati di calibrazioni necessari all'algoritmo
% di Visual Inertial Odometry
% -------------------------------------------------------------------------

% Decodifica del file .json
calibData = jsondecode(jsonFile);

% Riscrittura parametri come double e non come string
calibData.cameras.C0.focalLength = cellfun(@str2double, calibData.cameras.C0.focalLength);
calibData.cameras.C0.principalPoint = cellfun(@str2double, calibData.cameras.C0.principalPoint);
calibData.cameras.C0.distortionCoeffiecients = cellfun(@str2double, calibData.cameras.C0.distortionCoefficients);
calibData.cameras.C0.resolution = cellfun(@str2double, calibData.cameras.C0.resolution);
calibData.imus.S0.transformationToC0 = cellfun(@str2double, calibData.imus.S0.transformationToC0);
T_cam_imu = reshape(calibData.imus.S0.transformationToC0, [4,4])';
calibData.imus.S0.omegaOffsetAvg = cellfun(@str2double, calibData.imus.S0.omegaOffsetAvg);
calibData.imus.S0.accOffsetAvg = cellfun(@str2double, calibData.imus.S0.accOffsetAvg);

% Creazione della nuova struttura
calibration = struct();

% Assegnazione campi della nuova struttura
calibration.name = calibData.cameras.C0.name;

calibration.camera = struct();
calibration.camera.K = [calibData.cameras.C0.focalLength(1) 0 calibData.cameras.C0.principalPoint(1) ...
                        0 calibData.cameras.C0.focalLength(2) calibData.cameras.C0.principalPoint(2) ...
                        0 0 1]; % matrice intrinseca 3x3
calibration.camera.focalLength = calibData.cameras.C0.focalLength(:)'; % distanza focale
calibration.camera.principalPoint = calibData.cameras.C0.principalPoint(:)'; % centro immagine
calibration.camera.distortionCoefficients = calibData.cameras.C0.distortionCoefficients(:)'; % coefficienti di distorsione
calibration.camera.resolution = calibData.cameras.C0.resolution(:)';

calibration.imu = struct();
calibration.imu.T_cam_imu = T_cam_imu;
calibration.imu.timeShift = str2double(calibData.imus.S0.timeOffsetMicros)*1e-6; % [s] (t_imu = t_cam + timeshift)
calibration.imu.omegaMax =  str2double(calibData.imus.S0.omegaMax); % velocità angolare massima
calibration.imu.accMax = str2double(calibData.imus.S0.accMax); % accelerazione massima
calibration.imu.omegaOffsetAvg = calibData.imus.S0.omegaOffsetAvg; % bias medio giroscopio
calibration.imu.accOffsetAvg = calibData.imus.S0.accOffsetAvg; % bias medio accelerometro
calibration.imu.omegaOffsetVar = str2double(calibData.imus.S0.omegaOffsetVar); % deviazione standard bias giroscopio
calibration.imu.accOffsetVar = str2double(calibData.imus.S0.accOffsetVar); % deviazione standard bias accelerometro
calibration.imu.omegaNoiseDensity = str2double(calibData.imus.S0.omegaNoiseDensity); % noise density giroscopio 
calibration.imu.accNoiseDensity = str2double(calibData.imus.S0.accNoiseDensity); % noise density accelerometro
calibration.imu.omegaNoiseRandWalk = str2double(calibData.imus.S0.omegaNoiseRandomWalk); % random walk giroscopio
calibration.imu.accNoiseRandWalk = str2double(calibData.imus.S0.accNoiseRandomWalk); % random walk accelerometro

end