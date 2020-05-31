% Script for generating ConScan test data with non-zero latency.
% File format ('ConScanData.csv') is yaw, pitch, RSSI, GPSmsec, Mode. 
% Mode = 0 (no overdrive); 1 (Yaw overdrive); 2 (Pitch overdrive)
%
twopi   = 2*pi;
Latency = 1; % This is in integer measurement intervals (10 msec each)
BiasYaw     = 0.15; % degrees
BiasPitch   = 0.3; % degrees
NominalRssi = 2; % At -60 dB, RSSI voltage = 2 volts
OutputFID = fopen('ConScanData.csv','w');
%
% Build yaw data record (8 seconds)
ClockAngles = twopi/67:twopi/67:12*twopi; %12 cycles
YawAngles = 0.5 * sin(ClockAngles); % w.r.t. Beamwidths, about 1 deg
% Note: 12 sinusoidal oscillations in deg, measured from
% boresight.  This spans half of the 3 dB beamwidth of 1 deg.
% Yaw angles can be scaled for a sin(x)/x radiation pattern.
OffPointingLoss = 10*log10((sin((BiasYaw+YawAngles)*pi)./(BiasYaw+YawAngles)/pi).^2);
VoltageDelta = OffPointingLoss/20; % 20 dB/volt; negative values
% Note: BW outputs RSSI as 4 volt for peak signal, 1V for min
RssiVolts = max((NominalRssi + VoltageDelta), 1);
% Now add measurement noise Gaussian on +/- 2 dB (0.1 volts)
RssiVolts =  RssiVolts + (0.1*randn(1,length(YawAngles)));
%
% Build array of output data prior to latency shift
DataCount = length(YawAngles);
OutputArray = [YawAngles' zeros(DataCount, 1) RssiVolts' 10*(1:DataCount)' ones(DataCount, 1)];
%
% Add one second (100 records) of nominal non-oscillation (Mode = 0)
Quiescent = [zeros(100,1) zeros(100,1) NominalRssi*ones(100,1) 10*((DataCount+1):(DataCount+100))' zeros(100,1)];
OutputArray = [OutputArray; Quiescent];
%
% Add four seconds of pitch oscillation.  
ClockAngles = twopi/67:twopi/67:6*twopi; %Start with 6 cycles, then clip
PitchAngles = 0.5 * sin(ClockAngles); % w.r.t. Beamwidths 
OffPointingLoss = 10*log10((sin((BiasPitch+PitchAngles)*pi)./(BiasPitch+PitchAngles)/pi).^2);
VoltageDelta = OffPointingLoss/20; % 20 dB/volt; negative values
% Note: BW outputs RSSI as 4 volt for peak signal, 1V for min
RssiVolts = max((NominalRssi + VoltageDelta), 1);
% Now add measurement noise Gaussian on +/- 2 dB (0.1 volts)
RssiVolts =  RssiVolts + (0.1*randn(1,length(PitchAngles)));
PitchData = [zeros(400,1) PitchAngles(1:400)' RssiVolts(1:400)' 10*((DataCount+101):(DataCount+500))' 2*ones(400,1)];
OutputArray = [OutputArray; PitchData];
%
% Shift RSSI data to account for assumed latency
ArrayLength = length(OutputArray);
OutputArray((Latency+1):ArrayLength,3) = OutputArray(1:(ArrayLength-Latency),3);
OutputArray((1:Latency),3) = NominalRssi;
%
% Write file
HeaderRow = {'YawError','PitchError','RSSI','GPSmsec','Mode'};
fprintf(OutputFID,'%s,%s,%s,%s,%s\n',HeaderRow{:});
for Record = 1:length(OutputArray)
    fprintf(OutputFID,'%f,%f,%f,%d,%d\n',OutputArray(Record,:));
end
fclose(OutputFID);
return

