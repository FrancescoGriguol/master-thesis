% -------------------------------------------------------------------------
% Function che crea una tabella efficiente con i dati delle acquisizioni
% dal file .mat generato con aedat4tomat-master a partire dai file .aedat4
% originali
% -------------------------------------------------------------------------
function [data] = imucamData(fileID)

% Caricamento del file .mat ottenuto dalla conversione del .aedat4
load(fileID)

% Creazione della struttura di output
data = struct();

% Imu
timeStamp(:,1) = double(aedat.data.imu6.timeStamp);
temperature(:,1) = double(aedat.data.imu6.temperature);
accX(:,1) = double(aedat.data.imu6.accelX);
accY(:,1) = double(aedat.data.imu6.accelY);
accZ(:,1) = double(aedat.data.imu6.accelZ);
gyroX(:,1) = double(aedat.data.imu6.gyroX);
gyroY(:,1) = double(aedat.data.imu6.gyroY);
gyroZ(:,1) = double(aedat.data.imu6.gyroZ);

data.imu = table(timeStamp,temperature,accX,accY,accZ,gyroX,gyroY,gyroZ);

clear timeStamp temperature accX accY accZ gyroX gyroY gyroZ

% Events
timeStamp(:,1) = double(aedat.data.polarity.timeStamp);
x(:,1) = double(aedat.data.polarity.x);
y(:,1) = double(aedat.data.polarity.y);
polarity(:,1) = aedat.data.polarity.polarity;

data.events = table(timeStamp,x,y,polarity);

clear timeStamp x y polarity

% Frames
data.frames.timeStamp(:,1) = double(aedat.data.frame.timeStamp);
data.frames.samples = aedat.data.frame.samples;
data.frames.size = double(aedat.data.frame.size);

end