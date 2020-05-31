function [Latency,BestYawOffset,PitchOffset] = ConScanEmbedded(ConScanData)
    % Compute calibration offsets between GPS/IMU and antenna boresight, based
    % on current ConScan results assumed to be in an Nx5 array "ConScanData"
    % where columns are: 1) Yaw offset; 2) Pitch offset; 3) RSSI (volts); 4)
    % GPS msec; and 5) Mode (1 = yaw osc; 2 = pitch osc; 0 = no osc).
    %
    % This routine should be called when a full ConScan procedure has finished
    % and the calling routine has ensured there is sufficient data for analysis
    %
    CycleTime  = 10; % nominal measurement cycle time in msec
    ReceiveFreq= 71.5;   % GHz    ***LATENT BUG -- SHOULD BE INPUT PARAM
    AntDiameter= 0.3;    % meters ***LATENT BUG -- SHOULD BE INPUT PARAM
    RcvLambda  = 3e8/ReceiveFreq/1e9; % meters
    BeamWidth  = 70*RcvLambda/AntDiameter; % degrees
    %
    % Read data from ConScan array; perform data cleanup as needed.
    StartTime = ConScanData(1,4);
    NanSieve = any(isnan(ConScanData)');
    ConScanData = ConScanData(~NanSieve,:);
    [DataPoints, ~] = size(ConScanData);
    %
    % Identify and delete "outliers" that differ by more than 10 dB from their
    % adjacent data points.  Note: 10 dB is equivalent to 0.5 volts
    BigDeltas = abs(diff(ConScanData(:,3))) > 0.5;
    TwoSidedSieve = BigDeltas & [0; BigDeltas(1:(DataPoints-2),1)];
    ConScanData = ConScanData(~TwoSidedSieve,:);
    PeakVoltage = max(ConScanData(:,3));
    %
    % Extract yaw oscillation data, perform additional data cleanup as
    % required, and compute estimated latency and real-time bias offset
    YawSieve = ConScanData(:,5) == 1;
    YawData  = ConScanData(YawSieve,1:4);
    %
    % Repair possible gaps in record by inserting filler lines, assuming 
    % minimum cycle time is 10 msec.
    TimeDeltas = diff(YawData(:,4));
    LostLineIndices = find(TimeDeltas > CycleTime);
    if ~isempty(LostLineIndices)
        % repair data gaps, working from end of YawData to the  beginning
        fprintf(LOGfid,'\n** Data loss detected in Yaw.'); % Print to log file
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
    %
    % Repeat procedure for Pitch axis, but assume latency is the same
    PitchSieve = ConScanData(:,5) == 2;
    PitchData  = ConScanData(PitchSieve,1:4);
    %
    % Repair possible gaps in record by inserting filler lines, assuming 
    % minimum cycle time is 10 msec.
    TimeDeltas = diff(PitchData(:,4));
    LostLineIndices = find(TimeDeltas > CycleTime);
    if ~isempty(LostLineIndices)
        % repair data gaps, working from end of YawData to the  beginning
        fprintf(LOGfid,'\n** Data loss detected in Pitch.'); 
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
    %
    % Update log file with latency (msec) and incremental offset adjustments
    fprintf(LOGfid,'\n%d\t%d\t%f\t%f',StartTime,10*Latency,BestYawOffset,PitchOffset);
end
