

%% Mapping using Range Sensor for ground truth

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
% srcFactor = 0.054;
srcFactor = 1;
distSteps = 14;

%% Generate signals used for the experiment

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

%% Initialize Source Signal
soundPlayback=[winSignal;...
              zeros(setup.signal.lengthSignal-setup.signal.lengthBurst,1)];

%% Initializing COM port for motor control
if comportEnabled
    [serialInterface1, serialInterface2]=initializeSerialInterface();
end

%% Initialize PortAudio
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

%% Initial position X using Range sensor
fclose(serialInterface2);
fopen(serialInterface2);
initPosRobotX = str2num(fscanf(serialInterface2))/100;
if isempty(initPosRobotX)
%     fprintf(serialInterface1,'0')
    disp('Range Sensor value not read. Restart');
    initPosRobotX = str2num(fscanf(serialInterface2))/100;
end
initPosY = 1;
initPosWallX = initPosRobotX;
% initPosX = initPosX+initPosWallX;
%% Loop experiment for different distances
for k = 1:distSteps
    if comportEnabled
        fclose(serialInterface2);
        fopen(serialInterface2);
        lidarDist = str2num(fscanf(serialInterface2))/100
%         fprintf(serialInterface1,'8');
%         pause(3);
%         fprintf(serialInterface1,'0');
        while isempty(lidarDist)
%             fprintf(serialInterface1,'0')
            disp('Range Sensor value not read. Restart');
            lidarDist = str2num(fscanf(serialInterface2))/100;
        end
        %%
                %%
        disp('Playing and recording sound...');
        recordPage=playrec('playrec',srcFactor*soundPlayback*ones(1,length(outputChannel)),outputChannel,recordTime*sampleRate,inputChannel);

        while playrec('isFinished',recordPage)<1
        end
        disp('Done recording!');
        recordData=playrec('getRec',recordPage);

        
        [pks, locs]=findpeaks(recordData(:,2),'MinPeakProminence',0.001);
        rirEstFft = fft(recordData(locs(1):end,1), setup.signal.nfft)./fft(soundPlayback, setup.signal.nfft);
        rirEst = ifft(rirEstFft);

%              filtered loudspeaker response
        %     soundPlayback = filter(rirEst(81:200),1, soundPlayback);

        signals.observ = recordData(locs(1)+offset:end,1);
        signals.clean = soundPlayback;

%         %% resampling probed signal and observation to 28kHz
%         signals.observ = resample(double(signals.observ), 1, sampleFactor);
%         signals.clean = resample(double(signals.clean), 1, sampleFactor);


%          Estimate RIR from received and observed signals
        signals.signalObservFft = fft(signals.observ, setup.signal.nfft); % include direct path comp
        signals.signalCleanFft = fft(signals.clean, setup.signal.nfft);

        % spectrum_analyser(deviceID,inputChannel);
        % Estimator


        % NLS single channel
        [estimates, costFunction]=rNlsEst(signals,setup);
        toaEstimates.rNlsEst=estimates.rNlsEst.toa;
        
        nlsDist = toaEstimates.rNlsEst/setup.signal.sampFreq*setup.room.soundSpeed/2
        nlsDist1(k) = nlsDist;
        lidarDist1(k) = lidarDist;
        fclose(serialInterface2);
        fopen(serialInterface2);
        currentRobotPosX = str2num(fscanf(serialInterface2))/100;
        if isempty(currentRobotPosX)
        %     fprintf(serialInterface1,'0')
            disp('Range Sensor value not read. Restart');
            currentRobotPosX = str2num(fscanf(serialInterface2))/100;
        end
        currentRobotPosX = nlsDist;
        %% PLot
%         figure(360);
%         plot(rirEst)
%         vline(toaEstimates.rNlsEst)
%         legend('RIR')
%         title('NLS Est')

% %% check if NLS est is similar to Range sensor
%         if(nlsDist>0.5) || (nlsDist < lidarDist)
%             pause(2);
%             [estimates, costFunction]=rNlsEst(signals,setup);
%             toaEstimates.rNlsEst=estimates.rNlsEst.toa;
%             disp('NLS re-estimate');
%             nlsDist = toaEstimates.rNlsEst/setup.signal.sampFreq*setup.room.soundSpeed/2
%         else
%             continue;
%         end


        %%
        if plotEnabled
            figure(222);
            rectangle('Position',[0 0 initPosRobotX 12])
            axis([0 12 0 12])
            xlabel('X-Axis Distance [m]')
            ylabel('Y-Axis Distance [m]')
            title('Mapping using Range Sensor and Sound (Est. Glass Wall)')
            hold on
            grid on
%             scatter(initPosRobotX+abs(initPosRobotX-currentRobotPosX),initPosY, 'b');
%             legend('Robot Position','Range Sensor - Ground truth','NLS Est.');
            if abs(initPosRobotX-currentRobotPosX)<1
                scatter(initPosWallX+(lidarDist-initPosRobotX),initPosY,'k');
                scatter(initPosWallX+(nlsDist-initPosRobotX),initPosY,'r');
            else
                scatter(initPosWallX+(lidarDist),initPosY,'k');
                scatter(initPosWallX+(nlsDist),initPosY,'r');
            end
                initPosY = initPosY+0.5;
            hold off
        %     legend("SNR = "+ snrdB+ " dB");
 
        %     hold on;
        end
        %% Momentarily stop
        fprintf(serialInterface1,'8');
        pause(2)
        fprintf(serialInterface1,'0');
    end
    if saveData
        save(fullfile("D:\AudioAnalysisLab\OneDrive - Aalborg Universitet\Aalborg University\OneDrive - Aalborg Universitet\Aalborg University\AudioAnalysisLab\soundFile\"+"GlassWall"+"_"+"Step"+k+"_"+datestr(now,30)))
    end
end
fprintf(serialInterface1,'0');
legend('Range Sensor','NLS Est.');

if saveData
    save("dataMapGlassWall_"+datestr(now,30)+'.mat');
    saveas(gcf, "mapGlassWall"+datestr(now,30)+".fig")
    saveas(gcf, "mapGlassWall"+datestr(now,30)+".jpg")
    saveas(gcf, "mapGlassWall"+datestr(now,30)+".eps")   
end


% figure(222);
% rectangle('Position',[0 0 2 12])
% axis([0 5 0 12])
% xlabel('X-Axis Distance [m]')
% ylabel('Y-Axis Distance [m]')
% scatter(initPosWallX-abs(currentRobotPosX-lidarDist1),initPosY,'k', 'fill');
% hold on
% scatter(initPosWallX-abs(currentRobotPosX-nlsDist1),initPosY,'r', 'fill');
% legend('Range Sensor - Ground truth','NLS Est.');
% title('Mapping using Range Sensor and Sound (Robot)')
% fprintf(serialInterface1,'6');
% pause(6);
% fprintf(serialInterface1,'0');


%%
figure(222);
rectangle('Position',[0.8 0 initPosRobotX 15], 'LineStyle','--')
axis([0 7 0 16])
xlabel('X-Axis Distance [m]')
ylabel('Y-Axis Distance [m]')
% title('Mapping using Range Sensor and Sound (Est. Glass Wall)')
hold on
grid on
plot(initPosWallX+(lidarWall(:,1)),lidarWall(:,2),'bx','DisplayName', 'Lidar data');
plot(initPosWallX+(nlsWallEst(:,1)),nlsWallEst(:,2),'ro','DisplayName', 'Proposed Method');
plot(initPosWallX+(lidarWall(:,1)),lidarWall(:,2),'b-','HandleVisibility', 'off');
plot(initPosWallX+(nlsWallEst(:,1)),nlsWallEst(:,2),'r-','HandleVisibility', 'off');
line([1.2 1.2], [0 14], 'LineStyle','-.', 'tag','robot Movement')
hold off
box on
legend
%% Used for video
figure(666);
close all;
rectangle('Position',[0.8 0 initPosRobotX 15], 'LineStyle','--')
axis([0 7 0 16])
xlabel('X-Axis Distance [m]')
ylabel('Y-Axis Distance [m]')
box on
% title('Mapping using Range Sensor and Sound (Est. Glass Wall)')
grid on
hold on
% pause
plot(initPosWallX+(lidarWall(1,1)),lidarWall(1,2),'bx','DisplayName', 'Lidar data');
plot(initPosWallX+(nlsWallEst(1,1)),nlsWallEst(1,2),'ro','DisplayName', 'Proposed Method');
line([1.2 1.2], [0 14], 'LineStyle','-.', 'tag','robot Movement')
legend('LiDAR','Proposed Method','Robot Movement')

for qq = 1:length(nlsWallEst)
plot(initPosWallX+(lidarWall(qq,1)),lidarWall(qq,2),'bx','HandleVisibility', 'off');
plot(initPosWallX+(nlsWallEst(qq,1)),nlsWallEst(qq,2),'ro','HandleVisibility', 'off');
plot(initPosWallX+(lidarWall(qq,1)),lidarWall(qq,2),'b-','HandleVisibility', 'off');
plot(initPosWallX+(nlsWallEst(qq,1)),nlsWallEst(qq,2),'r-','HandleVisibility', 'off');
% pause(4);
end
hold off
