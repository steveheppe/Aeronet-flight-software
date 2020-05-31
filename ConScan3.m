% Compute calibration offsets between GPS/IMU and antenna boresight, based
% on ConScan results generated separately for yaw and pitch, and written
% into the file 'ConScanData.csv'. 
%
% This script also writes a timestamped archive file to disk, and generates
% some visualization plots to see computed offsets in real time.
%
% After validation, this script will also generate new boresight offsets.
close all
global KEY_IS_PRESSED KeyCmd BeamWidth
KEY_IS_PRESSED = 0;
%
% Open log file for test data capture
CurrentTimeString = char(datetime('now'));
filenamestr1 = ['ConScanTest_' CurrentTimeString(1:11) '_' ...
    CurrentTimeString(13:14) '_' CurrentTimeString(16:17) '.txt'];
LOGfid = fopen(filenamestr1,'w');
fprintf(LOGfid,'GPSmsec\tLatency(msec)\tYawOffset(deg)\tPitchOffset(deg)\tLatencyError\tYawError\tPitchError\n');
%
% Define filename for ConScan data on Pi (shared z: drive)
filenamestr4 = 'ConScanData.csv'; % BUG: Need to include path to z: drive
CycleTime  = 10; % nominal measurement cycle time in msec
ReceiveFreq= 71.5;   % GHz
AntDiameter= 0.3;    % meters
RcvLambda  = 3e8/ReceiveFreq/1e9; % meters
BeamWidth  = 70*RcvLambda/AntDiameter; % degrees
%
% Setup realtime status display window which also allows for keyboard
% interrupts using the Matlab figure attribute 'KeyPressFcn' associated
% with this realtime status display window, which must be active for the 
% keyboard interrupts to function properly as currently programmed.
StatusFigure = figure('Name','Current Status','Position',[10 400 500 400]);
set(StatusFigure, 'KeyPressFcn', @myKeyPressFcn)
plot(0,0,'rs');
axis([-5 5 -5 5]);
hold on
title('Computed Boresight Offsets');
xlabel('Degrees');
ylabel('Degrees');
TimeStr    = ['GPS TOW (sec)       = ' num2str(0)];
LatencyStr = ['Est. Latency (msec) = ' num2str(0) '; Error = ' num2str(0)];
HeadingStr = ['Current Yaw Bias    = ' num2str(0) '; Error = ' num2str(0)];
PitchStr   = ['CurrentPitch Bias   = ' num2str(0) '; Error = ' num2str(0)];
text(-4.8,4.5,TimeStr,'Clipping','on','VerticalAlignment','top');
text(-4.8,4.0,LatencyStr,'Clipping','on','VerticalAlignment','top');
text(-4.8,3.5,HeadingStr,'Clipping','on','VerticalAlignment','top');
text(-4.8,3.0,PitchStr,'Clipping','on','VerticalAlignment','top');

%
% Setup other figure windows which will be used for iterative data
RawFigure = figure('Name','Raw ConScan Data','Position',[550 400 500 400]);
YawFigure = figure('Name','Yaw Offset','Position',[700 200 500 400]);
PitchFigure = figure('Name','Pitch Offset','Position',[800 100 500 400]);
figure(StatusFigure);
%
% Main loop will check for new data roughly every 5 seconds
StartTime  = 0;
YawAdjust  = 0;
PitchAdjust= 0;
Running    = true;
Iteration  = 0;
TimerOne = tic;
while Running
    pause(5); 
    %
    % Every 10 seconds (roughly), generate new simulated ConScan data
    if mod(Iteration,2) == 0
        CurrentTime = toc(TimerOne);
        [SimLatency, SimYawBias, SimPitchBias] = SimConScan(CurrentTime);
    end
    Iteration = Iteration + 1;
    if KEY_IS_PRESSED  % Service keyboard commands
        switch KeyCmd
            case 'x' % Terminate execution; close files
                fprintf(1,'\n**ConScan script terminated manually\n');
                KEY_IS_PRESSED = 0;
                Running = false;
                KeyCmd = '';
            
            case 'a' % Apply results of last ConScan operation (save file)
                OffsetsFid = fopen('ConScanOffsets.txt','w');
                fprintf(OffsetsFid,'Realtime Yaw and Pitch ConScan offsets (deg) to adjust antenna boresight\n');
                fprintf(OffsetsFid,'%f\t%f\n',BestYawOffset,PitchOffset);
                fclose(OffsetsFid);
                KEY_IS_PRESSED = 0;
                KeyCmd = '';
        end
    end
    % Read data from current ConScan file; check to see if it is new data.
    % Data is assumed to be written as yaw, pitch, RSSI, GPSmsec, Mode
    % Mode = 0 (no overdrive); 1 (Yaw overdrive); 2 (Pitch overdrive)
    ConScanFid = fopen(filenamestr4,'rt');
    [RssiTable] = textscan(ConScanFid,'%f %f %f %f %f',...
        'headerline',1,...
        'delimiter',',',...
        'EmptyValue',NaN);
    RssiData = cell2mat(RssiTable);
    CurrentStartTime = RssiData(1,4);
    if CurrentStartTime == StartTime % Close file and wait another 5 sec.
        fclose(ConScanFid);
        continue
    end
    %
    % We have new data.  Update start time and get rid of NaNs (if any)
    StartTime = CurrentStartTime;
    NanSieve = any(isnan(RssiData)');
    RssiData = RssiData(~NanSieve,:);
    RssiData = RssiData(150:length(RssiData),:);
    [DataPoints, ~] = size(RssiData);
    %
    % Generate initial visualization of measurement data
    figure(RawFigure);
    hold off
    Time = RssiData(:,4) - StartTime;
    plot(Time,RssiData(:,1),'r');
    hold on
    plot(Time,RssiData(:,2),'b');
    plot(Time,RssiData(:,3),'k');
    xlabel('Time (msec)');
    ylabel('Yaw (deg), Pitch (deg), and RSSI (volts)');
    legend('Yaw offsets','Pitch offsets','RSSI data');
    figure(StatusFigure);
    %
    % Identify and delete "outliers" that differ by more than 10 dB from their
    % adjacent data points.  Note: 10 dB is equivalent to 0.5 volts
    BigDeltas = abs(diff(RssiData(:,3))) > 0.5;
    TwoSidedSieve = BigDeltas & [0; BigDeltas(1:(DataPoints-2),1)];
    RssiData = RssiData(~TwoSidedSieve,:);
    PeakVoltage = max(RssiData(:,3));
    %
    % Extract yaw oscillation data, perform additional data cleanup as
    % required, and compute estimated latency and real-time bias offset
    YawSieve = RssiData(:,5) == 1;
    YawData  = RssiData(YawSieve,1:4);
    %
    % Repair possible gaps in record by inserting filler lines, assuming 
    % minimum cycle time is 10 msec.
    TimeDeltas = diff(YawData(:,4));
    LostLineIndices = find(TimeDeltas > CycleTime);
    if isempty(LostLineIndices)
        text(0.1,4.8,'No detected data loss in Yaw.');
    else % repair data gaps, working from end of YawData to the  beginning
        text(0.1,4.8,'** Data loss detected in Yaw.');
        LostLineCount   = length(LostLineIndices);
        for Index = LostLineCount:-1:1
            LostRecords = (TimeDeltas(LostLineIndices(Index))/10) - 1;
            InsertPoint = LostLineIndices(Index); % filler goes after this line
            InsertBlock = [zeros(LostRecords,1) zeros(LostRecords,1) zeros(LostRecords,1) zeros(LostRecords,1)];
            [RowMax, ~] = size(YawData);
            YawData = [YawData(1:InsertPoint,:); InsertBlock; YawData((InsertPoint+1):RowMax,:)];
        end
    end
    %
    % Update array size and form logical vector marking "good" lines
    [RowMax, ~] = size(YawData);
    GoodLineSieve = (YawData(:,3) ~= 0);
    %
    % Parameter estimation algorithm:
    %   a) Try latency hypotheses +/- 10 measurement cycles (+/- 0.1 secs)
    %   b) Fit latency-shifted data to parabolic curve (approx. sin(x)/x)
    %   c) Select latency hypothesis with smallest MSE
    %   d) Cross-check with test of end-points and MSE vs. sin(x)/x
    YawMSE    = zeros(1,21);
    YawOffset = zeros(1,21);
    SavedPolyParams = zeros(3,21);
    IndexMSE = 0;
    for Latency = -10:10 % Positive values imply RSSI data is delayed
        IndexMSE = IndexMSE + 1;
        DoubledSieve = GoodLineSieve(11:(RowMax-10)) & GoodLineSieve((11+Latency):(RowMax+Latency-10));
        FullSieve    = [ones(10,1); DoubledSieve; ones(10,1)];
        SiftedYaw = YawData(logical(FullSieve),1);
        SiftedRSSI= YawData(logical(FullSieve),3);
        [RowMaxTemp, ~] = size(SiftedYaw);
        PolyParams = polyfit(SiftedYaw(11:(RowMaxTemp-10),1),SiftedRSSI((11+Latency):(RowMaxTemp+Latency-10),1),2);
        SavedPolyParams(1,IndexMSE) = PolyParams(1);
        SavedPolyParams(2,IndexMSE) = PolyParams(2);
        SavedPolyParams(3,IndexMSE) = PolyParams(3);
        % Compute offset which is the axis of symmetry, and MSE 
        YawOffset(IndexMSE) = -0.5 * PolyParams(2)/PolyParams(1);
        ComputedRssiValues  = polyval(PolyParams, SiftedYaw(11:(RowMaxTemp-10),1));
        RssiErrors = ComputedRssiValues - SiftedRSSI((11+Latency):(RowMaxTemp+Latency-10),1);
        YawMSE(IndexMSE) = mean(RssiErrors.^2);
    end
    [ParabolicMMSE, IndexMMSE] = min(YawMSE);
    Latency = IndexMMSE - 11;
    BestYawOffset = YawOffset(IndexMMSE);
    %
    % Recover best-case data for subsequent processing
    DoubledSieve = GoodLineSieve(11:(RowMax-10)) & GoodLineSieve((11+Latency):(RowMax+Latency-10));
    FullSieve    = [ones(10,1); DoubledSieve; ones(10,1)];
    SiftedYaw = YawData(logical(FullSieve),1);
    SiftedRSSI= YawData(logical(FullSieve),3);
    [RowMaxTemp, ~] = size(SiftedYaw);
    %
    % Cross-check by testing sin(x)/x hypotheses out to 1 deg error bound.
    % Use best-case observed RSSI (minimum RssiVoltage) as NominalRssi
    NominalRssi = min(SiftedRSSI);
    Hypotheses = sign(BestYawOffset)*(0.2:0.05:1);
    Hcount = length(Hypotheses);
    MeanErrors = zeros(1,Hcount);
    RssiMSE    = zeros(1,Hcount);
    for Hindex = 1:Hcount
        TestOffset = Hypotheses(Hindex);
        AdjustedYaws = SiftedYaw(11:(RowMaxTemp-10),1) - TestOffset;
        ExpectedGainLoss = 10*log10((sin(AdjustedYaws*pi/BeamWidth)./(AdjustedYaws*pi/BeamWidth)).^2);
        VoltageDelta = ExpectedGainLoss/20; % 20 dB/volt
        EstRssiVolts = max((NominalRssi + VoltageDelta), 1);
        RssiErrors= EstRssiVolts - SiftedRSSI((11+Latency):(RowMaxTemp+Latency-10),1);
        MeanErrors(Hindex)= mean(RssiErrors);
        RssiMSE(Hindex) = mean((RssiErrors - MeanErrors(Hindex)).^2);
    end
    [SincMMSE, SincIndex] = min(RssiMSE);
    if SincMMSE < ParabolicMMSE
        BestYawOffset = Hypotheses(SincIndex);
    end
    figure(YawFigure);
    hold off
    plot(SiftedYaw(11:(RowMaxTemp-10),1),SiftedRSSI((11+Latency):(RowMaxTemp+Latency-10),1),'r.');
    title('RSSI voltage data for Yaw Scan');
    ylabel('RSSI voltage');
    xlabel('Yaw Offpointing (deg)');
    EvalPoints = min(SiftedYaw):0.01:max(SiftedYaw);
    hold on
    if SincMMSE > ParabolicMMSE % plot a parabolic curve
        BestFit = polyval(SavedPolyParams(:,IndexMMSE), EvalPoints);
        plot(EvalPoints, BestFit, 'b');
    else % plot sin(x)/x curve
        ExpectedGainLoss = 10*log10((sin((EvalPoints-BestYawOffset)*pi/BeamWidth)./((EvalPoints-BestYawOffset)*pi/BeamWidth)).^2);
        VoltageDelta = ExpectedGainLoss/20; % 20 dB/volt
        EstRssiVolts = max((NominalRssi - MeanErrors(SincIndex) + VoltageDelta), 1);
        plot(EvalPoints, EstRssiVolts, 'b');
    end
    %
    % Repeat procedure for Pitch axis, but assume latency is the same
    PitchSieve = RssiData(:,5) == 2;
    PitchData  = RssiData(PitchSieve,1:4);
    %
    % Repair possible gaps in record by inserting filler lines, assuming 
    % minimum cycle time is 10 msec.
    TimeDeltas = diff(PitchData(:,4));
    LostLineIndices = find(TimeDeltas > CycleTime);
    figure(StatusFigure);
    if isempty(LostLineIndices)
        text(0.1,4.3,'No detected data loss in Pitch.');
    else % repair data gaps, working from end of YawData to the  beginning
        text(0.1,4.3,'** Data loss detected in Pitch.');
        LostLineCount   = length(LostLineIndices);
        for Index = LostLineCount:-1:1
            LostRecords = (TimeDeltas(LostLineIndices(Index))/10) - 1;
            InsertPoint = LostLineIndices(Index); % filler goes after this line
            InsertBlock = [zeros(LostRecords,1) zeros(LostRecords,1) zeros(LostRecords,1) zeros(LostRecords,1)];
            [RowMax, ~] = size(PitchData);
            PitchData = [PitchData(1:InsertPoint,:); InsertBlock; PitchData((InsertPoint+1):RowMax,:)];
        end
    end
    %
    % Update array size and form logical vector marking "good" lines
    [RowMax, ~] = size(PitchData);
    GoodLineSieve = (PitchData(:,3) ~= 0);
    %
    % Parameter estimation:
    DoubledSieve = GoodLineSieve(11:(RowMax-10)) & GoodLineSieve((11+Latency):(RowMax+Latency-10));
    FullSieve    = [ones(10,1); DoubledSieve; ones(10,1)];
    SiftedPitch  = PitchData(logical(FullSieve),2);
    SiftedRSSI   = PitchData(logical(FullSieve),3);
    [RowMax, ~] = size(SiftedPitch);
    PolyParams = polyfit(SiftedPitch(11:(RowMax-10),1),SiftedRSSI((11+Latency):(RowMax+Latency-10),1),2);
    % Compute offset which is the axis of symmetry, and MSE 
    PitchOffset  = -0.5 * PolyParams(2)/PolyParams(1);
    ComputedRssiValues  = polyval(PolyParams, SiftedPitch(11:(RowMax-10),1));
    RssiErrors = ComputedRssiValues - SiftedRSSI((11+Latency):(RowMax+Latency-10),1);
    ParabolicMSE = mean(RssiErrors.^2);
    %
    % Cross-check by testing sin(x)/x hypotheses out to 1 deg error bound.
    % Use best-case observed RSSI (minimum RssiVoltage) as NominalRssi
    NominalRssi = min(SiftedRSSI);
    Hypotheses = sign(PitchOffset)*(0.2:0.05:1);
    Hcount = length(Hypotheses);
    MeanErrors = zeros(1,Hcount);
    RssiMSE    = zeros(1,Hcount);
    for Hindex = 1:Hcount
        TestOffset = Hypotheses(Hindex);
        AdjustedPitches = SiftedPitch(11:(RowMax-10),1) - TestOffset;
        ExpectedGainLoss = 10*log10((sin(AdjustedPitches*pi/BeamWidth)./(AdjustedPitches*pi/BeamWidth)).^2);
        VoltageDelta = ExpectedGainLoss/20; % 20 dB/volt
        EstRssiVolts = max((NominalRssi + VoltageDelta), 1);
        RssiErrors= EstRssiVolts - SiftedRSSI((11+Latency):(RowMax+Latency-10),1);
        MeanErrors(Hindex)= mean(RssiErrors);
        RssiMSE(Hindex) = mean((RssiErrors - MeanErrors(Hindex)).^2);
    end
    [SincMMSE, SincIndex] = min(RssiMSE);
    if SincMMSE < ParabolicMSE
        PitchOffset = Hypotheses(SincIndex);
    end
    figure(PitchFigure);
    hold off
    plot(SiftedPitch(11:(RowMax-10),1),SiftedRSSI((11+Latency):(RowMax+Latency-10),1),'r.');
    title('RSSI voltage data for Pitch Scan');
    ylabel('RSSI voltage');
    xlabel('Pitch Offpointing (deg)');
    hold on
    EvalPoints = min(SiftedPitch):0.01:max(SiftedPitch);
    if SincMMSE > ParabolicMSE % plot a parabolic curve
        BestFit = polyval(PolyParams, EvalPoints);
        plot(EvalPoints, BestFit, 'b');
    else % plot sin(x)/x curve
        ExpectedGainLoss = 10*log10((sin((EvalPoints-PitchOffset)*pi/BeamWidth)./((EvalPoints-PitchOffset)*pi/BeamWidth)).^2);
        VoltageDelta = ExpectedGainLoss/20; % 20 dB/volt
        EstRssiVolts = max((NominalRssi - MeanErrors(SincIndex) + VoltageDelta), 1);
        plot(EvalPoints, EstRssiVolts, 'b');
    end
    %
    % Erase text status messages in StatusFigure and overwrite
    % Note: Latency is 10 msec per measurement cycle; offsets are the
    % negative of the simulated biases, so we add instead of subtract in
    % order to compute the current error values.
    figure(StatusFigure);
    text(-4.8,4.5,TimeStr,'Clipping','on','VerticalAlignment','top','Color','white');
    text(-4.8,4.0,LatencyStr,'Clipping','on','VerticalAlignment','top','Color','white');
    text(-4.8,3.5,HeadingStr,'Clipping','on','VerticalAlignment','top','Color','white');
    text(-4.8,3.0,PitchStr,'Clipping','on','VerticalAlignment','top','Color','white');
    TimeStr    = ['GPS TOW (sec)       = ' num2str(fix(StartTime/1000))];
    LatencyStr = ['Est. Latency (msec) = ' num2str(10*Latency) '; Error = ' num2str(10*(Latency-SimLatency))];
    HeadingStr = ['Current Yaw Bias    = ' num2str(BestYawOffset) '; Error = ' num2str(BestYawOffset+SimYawBias)];
    PitchStr   = ['CurrentPitch Bias   = ' num2str(PitchOffset) '; Error = ' num2str(PitchOffset+SimPitchBias)];
    text(-4.8,4.5,TimeStr,'Clipping','on','VerticalAlignment','top');
    text(-4.8,4.0,LatencyStr,'Clipping','on','VerticalAlignment','top');
    text(-4.8,3.5,HeadingStr,'Clipping','on','VerticalAlignment','top');
    text(-4.8,3.0,PitchStr,'Clipping','on','VerticalAlignment','top');
    plot(BestYawOffset,PitchOffset,'b.');
    %
    % Update log file
    fprintf(LOGfid,'%d\t\t%d\t\t\t\t%f\t\t%f\t\t%d\t\t\t\t%f\t%f\n',...
        StartTime,10*Latency,BestYawOffset,PitchOffset,...
        (10*(Latency-SimLatency)),(BestYawOffset+SimYawBias),(PitchOffset+SimPitchBias));
end
fclose('all');

function myKeyPressFcn(~, event)
% Capture keyboard single-stroke command and return it to the main routine.
global KEY_IS_PRESSED KeyCmd 
KEY_IS_PRESSED  = 1;
KeyCmd = event.Character;
switch event.Character
    case 'h'
        disp('List of single-character keyboard commands:')
        disp('x = close all files, plot results, and terminate script')
        disp('a = apply real-time results of last ConScan operation')
        KeyCmd = '';
        KEY_IS_PRESSED  = 0;
    case 'x'
    case 'a'
    otherwise
        disp('Unrecognized keyboard command. Type "h" for help')
        KeyCmd = '';
        KEY_IS_PRESSED  = 0;
end        
end

function [Latency,BiasYaw,BiasPitch] = SimConScan(StartTime)
% Script for generating simulated ConScan test data with non-zero latency.
% File format ('ConScanData.csv') is yaw, pitch, RSSI, GPSmsec, Mode. 
% Mode = 0 (no overdrive); 1 (Yaw overdrive); 2 (Pitch overdrive)
%
global BeamWidth
twopi   = 2*pi;
Latency = floor(10*rand - 5); % This is in integer measurement intervals 
BiasYaw     = rand - 0.5; % degrees
BiasPitch   = rand - 0.5; % degrees
NominalRssi = 2; % At -60 dB, RSSI voltage = 2 volts
OutputFID = fopen('ConScanData.csv','w');
GPShsec   = fix(StartTime*100);
%
% Build yaw data record (8 seconds)
ClockAngles = twopi/67:twopi/67:12*twopi; %12 cycles
YawAngles = 0.5 * sin(ClockAngles); % w.r.t. Beamwidths, about 1 deg
% Note: 12 sinusoidal oscillations in deg, measured from
% boresight.  This spans half of the 3 dB beamwidth of 1 deg.
% Yaw angles can be scaled for a sin(x)/x radiation pattern.
OffPointingLoss = 10*log10((sin((BiasYaw+YawAngles)*pi/BeamWidth)./((BiasYaw+YawAngles)*pi/BeamWidth)).^2);
VoltageDelta = OffPointingLoss/20; % 20 dB/volt
% Note: BW outputs RSSI as 4 volt for peak signal, 1V for min
RssiVolts = max((NominalRssi + VoltageDelta), 1);
% Now add measurement noise Gaussian on +/- 2 dB (0.1 volts)
RssiVolts =  RssiVolts + (0.1*randn(1,length(YawAngles)));
%
% Build array of output data prior to latency shift
DataCount = length(YawAngles);
OutputArray = [YawAngles' zeros(DataCount, 1) RssiVolts' 10*(GPShsec:(GPShsec+DataCount-1))' ones(DataCount, 1)];
%
% Add one second (100 records) of nominal non-oscillation (Mode = 0)
Quiescent = [zeros(100,1) zeros(100,1) NominalRssi*ones(100,1) 10*((GPShsec+DataCount):(GPShsec+DataCount+99))' zeros(100,1)];
OutputArray = [OutputArray; Quiescent];
%
% Add four seconds of pitch oscillation.  
ClockAngles = twopi/67:twopi/67:6*twopi; %Start with 6 cycles, then clip
PitchAngles = 0.5 * sin(ClockAngles); % w.r.t. Beamwidths 
OffPointingLoss = 10*log10((sin((BiasPitch+PitchAngles)*pi)./(BiasPitch+PitchAngles)/pi).^2);
VoltageDelta = OffPointingLoss/20; % 20 dB/volt
% Note: BW outputs RSSI as 4 volt for peak signal, 1V for min
RssiVolts = max((NominalRssi + VoltageDelta), 1);
% Now add measurement noise Gaussian on +/- 2 dB (0.1 volts)
RssiVolts =  RssiVolts + (0.1*randn(1,length(PitchAngles)));
PitchData = [zeros(400,1) PitchAngles(1:400)' RssiVolts(1:400)' 10*((GPShsec+DataCount+100):(GPShsec+DataCount+499))' 2*ones(400,1)];
OutputArray = [OutputArray; PitchData];
%
% Shift RSSI data to account for assumed latency
ArrayLength = length(OutputArray);
if Latency > 0
    OutputArray((Latency+1):ArrayLength,3) = OutputArray(1:(ArrayLength-Latency),3);
    OutputArray((1:Latency),3) = NominalRssi;
elseif Latency < 0
    OutputArray(1:(ArrayLength+Latency),3) = OutputArray((1-Latency):ArrayLength,3);
    OutputArray(((ArrayLength+Latency+1):ArrayLength),3) = NominalRssi;
end    
%
% Write file
HeaderRow = {'YawError','PitchError','RSSI','GPSmsec','Mode'};
fprintf(OutputFID,'%s,%s,%s,%s,%s\n',HeaderRow{:});
for Record = 1:length(OutputArray)
    fprintf(OutputFID,'%f,%f,%f,%d,%d\n',OutputArray(Record,:));
end
fclose(OutputFID);
end

