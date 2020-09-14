%% Evaluating NLS, Em-Sc Vs SNR on real Data
%

% to plot graph in the desired SNR = [30 20 15 10 5 0]

clc;
clear all;
close all;

%% Add Libraries
addpath(genpath('m_files'));
addpath(genpath('srcSignal'));
addpath(genpath('lib'));
addpath(genpath('MAT_files\'));
dataPath = dir('MAT_files\');

%% Plot enable?
showData = true;
savedData = false;
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
tolerance = 10;
snrList = [30 20 15 10 5 1];
snrList=snrList(1);
distToWallSteps = 10;
actualDistDATA = [0.8300    1.1500    1.5100    2.0100    2.500];
distances = [0.8 1.0 1.5 2.0 2.5];
% actualDist = [2.5 2.01 1.5 0.83 1.0];
setup.EM.nIter = 40;
sigFactor = [1];
actualDistDATA = actualDistDATA';
setup.EM.minimumDistance=0.8;
setup.EM.maximumDistance=3;

toaInterval=round((2*[setup.EM.minimumDistance;setup.EM.maximumDistance]...
    -setup.array.micRadius)/setup.room.soundSpeed*setup.signal.sampFreq);

windowNdx=toaInterval(1):(toaInterval(2)+setup.signal.lengthBurst);
lengthWindow=length(windowNdx);

grids.toa=toaInterval(1):toaInterval(2);

%% Generate signals used for the experiment

% Length of the sound burst
setup.signal.lengthBurst = 1500;
% length of the signal
setup.signal.lengthSignal = 5000;
% sampling frequency
sampleRate = 48000;
% resample signal by factor
sampleFactor = 1;

setup.signal.sampFreq = sampleRate;

%% Applying window to the source signal
nwin = 440;
winbkman = blackman(nwin);
winbkman = winbkman(nwin/2+1:end);
rng default
% [bhp,ahp]=butter(4,4/24,'high');

winSignal = randn(setup.signal.lengthBurst,1);
winSignal(end-length(winbkman)+1:end) = winSignal(end-length(winbkman)+1:end).*winbkman;
winSignal = 0.9*winSignal/max(abs(winSignal));
soundPlayback=[winSignal;...
                  zeros(setup.signal.lengthSignal-setup.signal.lengthBurst,1)];

%% evaluation
% kk = 1;
ll = 1;
hh = 1;
disp("Processing...")
tic
for kk = 13:17%size(dataPath,1)
    load(dataPath(kk).name)
    for ii=1:100
           % Generate probe signal

           soundPlayback = measurementData.probedSignal(:,ii);

           recordData = measurementData.audioData(:,1,ii);
    %        recordData = recordData.tmpAudioData;

           rirEstFft = fft(recordData, setup.signal.nfft) ./ fft(soundPlayback, setup.signal.nfft);
           rirEst = ifft(rirEstFft);
%            plot(rirEst)

           signals.observ = recordData;
           signals.clean = soundPlayback;
    %        signals.observ = signals.observ(1:43000,1);

           % NLS Sc
           [estimates, costFunction]=rNlsEst(signals,setup);
           toaEstimates.rNlsEst=estimates.rNlsEst.toa;       

           %Single channel EM
           estimates=emSingleChan(signals,setup);
           toaEstimates.emOneCh=estimates.emOneCh.toa;

           if showData
               figure(200);
               subplot(211)
               plot(rirEst)
               vline(toaEstimates.rNlsEst)
               legend('RIR')
               title('NLS Est')
               subplot(212)
               plot(rirEst)
               vline(toaEstimates.emOneCh)
               title('One Channel EM')
               legend('RIR')
           end

           nlsDistDATA(ii,ll) = toaEstimates.rNlsEst/setup.signal.sampFreq*setup.room.soundSpeed/2;

           emDistDATA(ii,ll) = toaEstimates.emOneCh/setup.signal.sampFreq*setup.room.soundSpeed/2;

          
           if (nlsDistDATA(ii,ll)<=(((100+tolerance)/100)*actualDistDATA(ll)) && nlsDistDATA(ii,ll)>=(((100-tolerance)/100)*actualDistDATA(ll)))
               distErrorNls(ii,ll) = 0;
           else
               distErrorNls(ii,ll) = 1;
           end

           if (emDistDATA(ii,ll)<=(((100+tolerance)/100)*actualDistDATA(ll)) && emDistDATA(ii,ll)>=(((100-tolerance)/100)*actualDistDATA(ll)))
               distErrorEmSc(ii,ll) = 0;
           else
               distErrorEmSc(ii,ll) = 1;
           end
           hh= hh+1;
    end
%     meanValue(ll,hh) = mean(nlsDistDATA(:,ll))';
%     varianceValue(ll,hh) = var(nlsDistDATA(:,ll))';    
    ll = ll+1;
    hh = 1;
    
end
toc
disp("Completed!")
%% 95% Confidence interval
[nn, nx] = size(nlsDistDATA);
nn = nn-1;
% 95% confidence interval
% A table can be found from, for example,
% https://en.wikipedia.org/wiki/Student%27s_t-distribution
talpha = tinv(1-0.025,nn);

mval = mean(nlsDistDATA);
sval = (talpha*std(nlsDistDATA,0,1)/sqrt(nn));

figure(221)
errorbar(actualDistDATA, ones(1,nx).*mval, ones(1,nx).*sval, 'DisplayName', 'whatever');
xlabel('Distance [m]')
ylabel('mean error[m^2]')
grid on

%% RMSE

for ll = 1: size(nlsDistDATA,2)
   rmseDATA(ll) = calculateRmse(nlsDistDATA(:,ll), actualDistDATA(ll));
end
   figure(666);
   errorbar(distances, (rmseDATA), std(nlsDistDATA));
   grid on
   xlabel('Distance [m]');
   ylabel('RMSE [m]');
   
%% make table to show data
meanValue = mean(nlsDistDATA)';
standardDev = sqrt(var(nlsDistDATA))';
rmseValue = rmseDATA';
realData = table(actualDistDATA, meanValue, standardDev, rmseValue)
%% Plot Percentage Curve NLS
figure(222);
distPercError = (1-sum(distErrorNls)/length(distErrorNls));
% plot([0.5:0.5:2.5],(distPercError)*100, 'bo');
% hold on
plot(actualDistDATA,(distPercError)*100, 'b-');
xlabel('Distance [m]')
ylabel('Percentage Accuracy [%]')
title('Performance measure of NLS against distance (Wall)')
% legend("SNR = "+ snrList(1)+ " dB");
%         legend('SNR = 30 dB', 'SNR = 20 dB', 'SNR = 15 dB', 'SNR = 10 dB', 'SNR = 5 dB');
% fname = sprintf("percAccuracyDistSnr%d.mat",SNR);
% save(fname);
grid on
if savedData
        saveas(gcf, "percVsSdnr_NlsEst"+"_"+snrList+"_"+datestr(now,30)+".fig")
        saveas(gcf, "percVs_Sdnr_NlsEst"+"_"+snrList(1)+"_"+datestr(now,30)+".jpg")
        saveas(gcf, "percVs_Sdnr_NlsEst"+"_"+snrList(1)+"_"+datestr(now,30)+".eps")
end
%% Plot Percentage Curve EM
figure(223);
distPercErrorEmSC = (1-sum(distErrorEmSc)/length(distErrorEmSc));
plot(actualDistDATA,(distPercErrorEmSC)*100, 'rx');
hold on
plot(actualDistDATA,(distPercErrorEmSC)*100, 'r-');
xlabel('Distance [m]')
ylabel('Percentage Accuracy [%]')
title('Performance measure of EM against distance (Wall)')
% legend("SNR = "+ snrList(1)+ " dB");
%         legend('SNR = 30 dB', 'SNR = 20 dB', 'SNR = 15 dB', 'SNR = 10 dB', 'SNR = 5 dB', 'SNR = 1 dB');
% fname = sprintf("percAccuracyDistSnr%d.mat",SNR);
% save(fname);
grid on
if savedData
        saveas(gcf, "percVsSdnr_EmSc"+"_"+snrList+"_"+datestr(now,30)+".fig")
        saveas(gcf, "percVs_Sdnr_EmSc"+"_"+snrList(1)+"_"+datestr(now,30)+".jpg")
        saveas(gcf, "percVs_Sdnr_EmSc"+"_"+snrList(1)+"_"+datestr(now,30)+".eps")
        save([mfilename,'_RealData_SNR',num2str(measurementData.snr),'_',datestr(now,30),'.mat']);
end