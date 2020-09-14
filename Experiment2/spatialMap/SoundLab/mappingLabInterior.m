% % Mapping the interior of the lab [6.68 x 5.4] m

clc;
clear all;
close all;

%% Add Libraries
addpath(genpath('m_files'));
addpath(genpath('srcSignal'));
addpath(genpath('lib'));

%% Clear all COM ports
clearAllPort();

%% Add location to save the data and enable macros
% directoryLoc = 'D:\AudioAnalysisLab\OneDrive - Aalborg Universitet\Aalborg University\OneDrive - Aalborg Universitet\Aalborg University\AudioAnalysisLab\13Nov2019\';
saveData = true;
plotEnabled = true;
comportEnabled = true;
recordEnabled = false;
echolocationMode = true;
% offset = 36;
offset = 0;

%% Initializing setup
setup=defaultMiscSetup([]);
setup=defaultSignalSetup(setup);
setup=defaultArraySetup(setup);
setup=defaultRirGenSetup(setup);
setup=defaultRoomSetup(setup);
setup=defaultEmSetup(setup);
setup = defaultNLSSetup(setup);
setup.fftLength = 2^11;
setup.signal.nfft = 2^11;
setup.signal.tolerance = 10;
% srcFactor = 0.054;
srcFactor = 1;
distSteps = 44;

%% Generate Broadband Signal

% Length of the sound burst
setup.signal.lengthBurst = 1500;
% length of the signal
setup.signal.lengthSignal = 5000;
% sampling frequency
sampleRate = 48000;
% resample signal by  factor
sampleFactor = 1;
setup.signal.sampFreq = sampleRate;

%% Applying window to the source signal
nwin = 440;
winbkman = blackman(nwin);
winbkman = winbkman(nwin/2+1:end);
rng default
% [bhp,ahp]=butter(4,4/24,'high');

% winSignal = filter(bhp,ahp,randn(setup.signal.lengthBurst,1));
winSignal = randn(setup.signal.lengthBurst,1);
winSignal(end-length(winbkman)+1:end) = winSignal(end-length(winbkman)+1:end).*winbkman;
winSignal = 0.9*winSignal/max(abs(winSignal));

%% Initializing COM port for motor control
% if comportEnabled
    [serialInterface1, serialInterface2]=initializeSerialInterface();
    % Momentarily stop
    fprintf(serialInterface1, '0');
        % Lidar Distance
    fclose(serialInterface2);
    fopen(serialInterface2);
    lidarDist = str2num(fscanf(serialInterface2))/100;
    while isempty(lidarDist)
         disp('Range Sensor value not read. Restart');
         lidarDist = str2num(fscanf(serialInterface2))/100;
    end
% end

%% Initialize PortAudio
if echolocationMode
    deviceList=playrec('getDevices');
    deviceID=16;
    inputChannel=[1:2];
    outputChannel=[1:2];
    recordTime=1;

    deviceForPlayRec=deviceList(deviceID);

    if playrec('isInitialised')
         playrec('reset');
    end
    playrec('init',sampleRate,deviceID,deviceID);
    pause(3) % This pause is necessary after initialization step 
end
distCounter = 1;
for k=1:distSteps
    %% Echolocation
    if echolocationMode
        % Initialize Source Signal
        soundPlayback(:,k)=[winSignal;...
                  zeros(setup.signal.lengthSignal-setup.signal.lengthBurst,1)];

        disp('Probing the environment...');
        recordPage=playrec('playrec',srcFactor*soundPlayback(:,k)*ones(1,length(outputChannel)),outputChannel,recordTime*sampleRate,inputChannel);

        while playrec('isFinished',recordPage)<1
        end
        disp('Done recording!');
        recordData=playrec('getRec',recordPage);

        % Removing hardware delay
        [pks, locs]=findpeaks(recordData(:,2),'MinPeakProminence',0.001);
        rirEstFft = fft(recordData(locs(1):end,1), setup.signal.nfft)./fft(soundPlayback(:,k), setup.signal.nfft);
        rirEst = ifft(rirEstFft);

        signals.observ = recordData(locs(1)+offset:end,1);
        signals.clean = soundPlayback(:,k);
        signals.observ = signals.observ(1:44000,1);
        tmpObserve = signals.observ;
        if saveData
            save(fullfile("D:\AudioAnalysisLab\OneDrive - Aalborg Universitet\Aalborg University\OneDrive - Aalborg Universitet\Aalborg University\AudioAnalysisLab\labMapping\"+"labMappingRawObservedData"+datestr(now,30)+".wav"),"tmpObserve");
            save(fullfile("D:\AudioAnalysisLab\OneDrive - Aalborg Universitet\Aalborg University\OneDrive - Aalborg Universitet\Aalborg University\AudioAnalysisLab\labMapping\"+"labMappingRawObservedData"+datestr(now,30)+".mat"),"tmpObserve")
        end
        % Estimate RIR from received and observed signals
        signals.signalObservFft = fft(signals.observ, setup.signal.nfft); % include direct path comp
        signals.signalCleanFft = fft(signals.clean, setup.signal.nfft);

        % NLS single channel
        [estimates, costFunction]=rNlsEst(signals,setup);
        toaEstimates.rNlsEst=estimates.rNlsEst.toa;
        nlsDist = toaEstimates.rNlsEst/setup.signal.sampFreq*setup.room.soundSpeed/2;
%         while ~(nlsDist <= ((100+setup.signal.tolerance)/100*lidarDist)) && ~(nlsDist >= ((100-setup.signal.tolerance)/100)*lidarDist)
%             recordPage=playrec('playrec',srcFactor*soundPlayback(:,k)*ones(1,length(outputChannel)),outputChannel,recordTime*sampleRate,inputChannel);
%             while playrec('isFinished',recordPage)<1
%             end
%             recordData=playrec('getRec',recordPage);
%                     [pks, locs]=findpeaks(recordData(:,2),'MinPeakProminence',0.001);
%             rirEstFft = fft(recordData(locs(1):end,1), setup.signal.nfft)./fft(soundPlayback(:,k), setup.signal.nfft);
%             rirEst = ifft(rirEstFft);
% 
%             signals.observ = recordData(locs(1)+offset:end,1);
%             signals.clean = soundPlayback(:,k);
%             signals.observ = signals.observ(1:44000,1);
% 
%             % Estimate RIR from received and observed signals
%             signals.signalObservFft = fft(signals.observ, setup.signal.nfft); % include direct path comp
%             signals.signalCleanFft = fft(signals.clean, setup.signal.nfft);
% 
%             % NLS single channel
%             [estimates, costFunction]=rNlsEst(signals,setup);
%             toaEstimates.rNlsEst=estimates.rNlsEst.toa;
%             nlsDist = toaEstimates.rNlsEst/setup.signal.sampFreq*setup.room.soundSpeed/2;
%         end
%         disp("2nd Measurement ...")
% %         nlsDist = toaEstimates.rNlsEst/setup.signal.sampFreq*setup.room.soundSpeed/2;

        if plotEnabled
            figure(200);
            plot(rirEst)
            vline(toaEstimates.rNlsEst)
            legend('RIR')
            title('NLS Est')
        end
        % Momentarily stop
        pause(3);
        nlsDistData(k) = nlsDist;
        toaData(k)=toaEstimates.rNlsEst;
        rirData(:,k) = rirEst;
        receivedData(:,k) = signals.observ;
        probedData(:,k) = signals.clean;
        hardwareDelay(k) = locs(1);
    end
    %% ComPort Enabled
    if comportEnabled
        %% Lidar Distance
        fclose(serialInterface2);
        fopen(serialInterface2);
        lidarDist = str2num(fscanf(serialInterface2))/100;
        while isempty(lidarDist)
            disp('Range Sensor value not read. Restart');
            lidarDist = str2num(fscanf(serialInterface2))/100;
        end

        %% Movement 5 steps
        fprintf(serialInterface1,'8');
        pause(1.5);
        fprintf(serialInterface1,'0');
        pause(3);
        if (distCounter == 14) || (distCounter == 22) || (distCounter == 36)
            fprintf(serialInterface1,'0');
            pause(3);
            fprintf(serialInterface1,'6');
            pause(4);
            fprintf(serialInterface1,'0');
            pause
        end
        distCounter = distCounter + 1;
        lidarData(k) = lidarDist;
    end
end
fprintf(serialInterface1,'0');

%% Concatinating LidarData
mapLidarData1 = lidarData(1:15)';
mapLidarData2 = lidarData(15:22)' + 6.48;
mapLidarData3 = lidarData(23:37)'+ 5.7;
mapLidarData4 = lidarData(38:44)' + 0.4;

mapLidarData1(:,2) = [linspace(1.5,7,length(mapLidarData1))];
mapLidarData2(:,2) = [linspace(2,6.2,length(mapLidarData2))];
mapLidarData3(:,2) = [linspace(1.5,7,length(mapLidarData3))];
mapLidarData4(:,2) = [linspace(2,6.2,length(mapLidarData4))];

%% Find deviation in robot motion
% robotDev1 = 1+mapLidarData1(:,1);
% robotDev2 = 7.48-mapLidarData2(:,1);
% robotDev3 = 6.4-mapLidarData3(:,1);
% robotDev4 = 1+mapLidarData4(:,1);
% 
% robotDev1(:,2) = [linspace(1.5,7,length(mapLidarData1))];
% robotDev2(:,2) = [linspace(1.5,7,length(mapLidarData2))];
% robotDev3(:,2) = 6.4-mapLidarData3(:,1);
% robotDev4(:,2) = 6.4-mapLidarData3(:,1);

%% Concatinating Acoustic Data
mapNlsDistData1 = nlsDistData(1:15)';
mapNlsDistData2 = nlsDistData(15:22)' + 6.48;
mapNlsDistData3 = nlsDistData(23:37)'+ 5.6;
mapNlsDistData4 = nlsDistData(38:44)' + 0.2;

mapNlsDistData1(:,2) = [linspace(1.5,7,length(mapNlsDistData1))];
mapNlsDistData2(:,2) = [linspace(2,6.2,length(mapNlsDistData2))];
mapNlsDistData3(:,2) = [linspace(1.5,7,length(mapNlsDistData3))];
mapNlsDistData4(:,2) = [linspace(2,6.2,length(mapNlsDistData4))];

%% Save data
measurementData.NLS = nlsDistData;
measurementData.toa = toaData;
measurementData.RIRData = rirData;
measurementData.ObservedSignals = receivedData;
measurementData.probedSignal = probedData;
measurementData.hardware_delay = hardwareDelay;
measurementData.LIDAR = lidarData;
measurementData.LidarWallData1 = mapLidarData1;
measurementData.LidarWallData2 = mapLidarData2;
measurementData.LidarWallData3 = mapLidarData3;
measurementData.LidarWallData4 = mapLidarData4;
measurementData.nlsDistWallData1 = mapNlsDistData1;
measurementData.nlsDistWallData2 = mapNlsDistData2;
measurementData.nlsDistWallData3 = mapNlsDistData3;
measurementData.nlsDistWallData4 = mapNlsDistData4;


%% Superimposing acoustic distance
figure(450);
rectangle('Position',[1 1 5.4 6.48], 'LineStyle','--')
axis([0 8 0 8])
xlabel('X-Axis Distance [m]')
ylabel('Y-Axis Distance [m]')
% title('Mapping using Range Sensor and Sound (Lab Interior)')
grid on
hold on
plot(measurementData.LidarWallData1(:,1),measurementData.LidarWallData1(:,2), 'bx',...
    'DisplayName', 'LiDAR')
plot(measurementData.LidarWallData2(:,2),measurementData.LidarWallData2(:,1), 'bx', ...
    'HandleVisibility', 'off')
plot(measurementData.LidarWallData3(:,1),measurementData.LidarWallData3(:,2), 'bx', ...
    'HandleVisibility', 'off')
plot(measurementData.LidarWallData4(:,2),measurementData.LidarWallData4(:,1), 'bx', ...
    'HandleVisibility', 'off')
plot(measurementData.LidarWallData1(:,1),measurementData.LidarWallData1(:,2), 'b-', ...
    'HandleVisibility', 'off')
plot(measurementData.LidarWallData2(:,2),measurementData.LidarWallData2(:,1), 'b-', ...
    'HandleVisibility', 'off')
plot(measurementData.LidarWallData3(:,1),measurementData.LidarWallData3(:,2), 'b-', ...
    'HandleVisibility', 'off')
plot(measurementData.LidarWallData4(:,2),measurementData.LidarWallData4(:,1), 'b-', ...
    'HandleVisibility', 'off')
% hold off
% hold on
plot(measurementData.nlsDistWallData1(:,1),measurementData.nlsDistWallData1(:,2), 'ro', ...
    'DisplayName', 'Proposed Method')
plot(measurementData.nlsDistWallData2(:,2),measurementData.nlsDistWallData2(:,1), 'ro', ...
    'HandleVisibility', 'off')
plot(measurementData.nlsDistWallData3(:,1),measurementData.nlsDistWallData3(:,2), 'ro', ...
    'HandleVisibility', 'off')
plot(measurementData.nlsDistWallData4(:,2),measurementData.nlsDistWallData4(:,1), 'ro', ...
    'HandleVisibility', 'off')
plot(measurementData.nlsDistWallData1(:,1),measurementData.nlsDistWallData1(:,2), 'r-', ...
    'HandleVisibility', 'off')
plot(measurementData.nlsDistWallData2(:,2),measurementData.nlsDistWallData2(:,1), 'r-', ...
    'HandleVisibility', 'off')
plot(measurementData.nlsDistWallData3(:,1),measurementData.nlsDistWallData3(:,2), 'r-', ...
    'HandleVisibility', 'off')
plot(measurementData.nlsDistWallData4(:,2),measurementData.nlsDistWallData4(:,1), 'r-', ...
    'HandleVisibility', 'off')
rectangle('Position',[2 2 3.5 4.5], 'LineStyle','-.', 'tag', 'Robot Trajectory','EdgeColor',[0 0 1])

box on
legend
hline = line(NaN,NaN,'LineWidth',2,'LineStyle','-.','Color',[0 0 1]);
legend('LiDAR','Proposed Method','Robot Trajectory')
hold off

%%
if saveData
    save([mfilename,'dataMaplab','_',datestr(now,30),'.mat']);
end

%% Superimposing acoustic distance
figure(450);
close all
hold on
rectangle('Position',[1 1 5.4 6.48], 'LineStyle','--')
rectangle('Position',[2 2 3.5 4.5], 'LineStyle','-.', 'tag', 'Robot Trajectory','EdgeColor',[0 0 1])
axis([0 8 0 8])
xlabel('X-Axis Distance [m]')
ylabel('Y-Axis Distance [m]')
plot(measurementData.LidarWallData1(4,1),measurementData.LidarWallData1(4,2), 'bx',...
    'DisplayName', 'LiDAR')
plot(measurementData.nlsDistWallData1(4,1),measurementData.nlsDistWallData1(4,2), 'ro', ...
    'DisplayName', 'Proposed Method')
box on
hline = line(NaN,NaN,'LineWidth',2,'LineStyle','-.','Color',[0 0 1]);
legend('LiDAR','Proposed Method','Robot Trajectory')
% title('Mapping using Range Sensor and Sound (Lab Interior)')
grid on
pause 
for qq = 4:2:length(measurementData.LidarWallData1)
plot(measurementData.LidarWallData1(qq,1),measurementData.LidarWallData1(qq,2), 'bx',...
    'HandleVisibility', 'off')

% hold off
% hold on
plot(measurementData.nlsDistWallData1(qq,1),measurementData.nlsDistWallData1(qq,2), 'ro', ...
    'HandleVisibility', 'off')
% pause(6) %Used for video
end
% pause(6) %Used for video
for nn = 1:1:length(measurementData.LidarWallData2)
plot(measurementData.LidarWallData2(nn,2),measurementData.LidarWallData2(nn,1), 'bx',...
    'HandleVisibility', 'off')

% hold off
% hold on
plot(measurementData.nlsDistWallData2(nn,2),measurementData.nlsDistWallData2(nn,1), 'ro', ...
    'HandleVisibility', 'off')
pause(5) %Used for video
end
hold off
