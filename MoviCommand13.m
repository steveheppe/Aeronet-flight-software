% SCRIPT TO SEND COMMANDS TO MOVI PRO (AND READ GPS AND POINT)
% Note: This version has InertialLabs and Movi I/O as in-line instead of as
% functions, since the function calls were consuming too much time.  Need
% to compile these. TBD
%
% There is an assumption that the INS-DL has been previously configured to
% generate binary data (the OPVT2A sentence) upon startup at 460,800 bps. 
%
% Global declarations might be useful for separate function calls.  TBD.
%global InsPort
%global s BytesRead GimbalStatus1 GimbalStatus2 GimbalRX GimbalRY GimbalRZ
clear all
close ALL
global KEY_IS_PRESSED KeyCmd TargetID BeamWidth s LOGfid
global RslSimHeadingOffset RslSimPitchOffset RslSimLatency RslBuffer
global RslBufferPointer;
KEY_IS_PRESSED = 0;
%
% IMPORTANT: Gimbal Control Mode (GimbalCntl) is set near line 453
TargetID = 3; % 1 = WPB/Tiara; 2 = Freeport; 3 = Mt. Hood
Duration = 60;  % Duration of test in seconds
SensorID = 1;   % 0 = VectorNav; 1 = INS-DL
RslSimState  = 1;% 0 = read real RSSI values; 1 = simulated values
RslSimLatency= 4;% RSSI latency in measurement cycles (in simulation)
RslBuffer = zeros((RslSimLatency+1),1);
RslBufferPointer = 1;
ConScanRowMax= 1000; % RSSI data points for real-time GPS drift analysis
ConScanArray = zeros(ConScanRowMax,3);
ConScanPointer = 1;
EstimatedLatency = RslSimLatency;
GimbalCoastTime = 5; % Max time (sec) for coasting on gimbal telemetry
GimbalCoastTimer = tic;
AlphaThreshold   = 5; % Heading jump (degrees) that results in hysteresis
GoodINSdata = 0;
TorqueMode = 1; % System starts with torque applied to motors.
HeadingAvgErr = 0;
HeadingStd = 0; % Will hold STD of est. heading error over last N seconds
PitchAvgErr = 0;
PitchStd = 0;
RollStd = 0;
DeltaRX = 0; % Will hold angle deltas (between last two reports) in MoviTLM
DeltaRY = 0;
DeltaRZ = 0;
RecentErrors = zeros(ConScanRowMax,3);
UnwrapState = 0; % 0 = not unwrapping; 1 = unwrapping
UnwrapValue = 360; % Test value for amount of unwrap to perform
RslSimHeadingOffset = 0.2; % Misalignment (deg) between GPS/IMU and antenna
RslSimPitchOffset   = 0.1;
TestConScan  = 1;% If 1, will review RSSI data every 10 seconds to check 
                 % for possible drift in GPS heading guidance
RandomShifts=rand(1,2);
WideScanning = 0;% State variable= 1 if currently in wide RSSI scan
WideScanObj  = 0;% =1 iff WideScanning file and figure are open.
WideScanSpan = 5;% Half-width of wide yaw/pitch RSSI scan span in degrees
Scanning     = 0;% State variable= 1 if currently in "plus scan" opperation
ScanSpan     = 0.25; % Half-width of narrow yaw/pitch scan in degrees 
UseGimbalYaw = 0;% This control switch would exercise a "fixed-based" unit 
                 % that had been surveyd-in (eg, Tiara rooftop)
% Open a new or existing file to see if previously-stored offsets 
% have been generated for this radio node.
OffsetsFid = fopen('BoresightOffsets.txt','a+');
frewind(OffsetsFid);
HeaderLine = fgetl(OffsetsFid);
if HeaderLine == -1
    %We've opened an empty file.  Save with "factory offsets" for next use
    fprintf(OffsetsFid,'Yaw and Pitch offsets (deg) for antenna boresight\n');
    HeadingAdjBase= 0;   % Boresight relative to GPS/IMU sensor axis
    TiltAdjBase   = 0;   
    fprintf(OffsetsFid,'%f\t%f\n',HeadingAdjBase,TiltAdjBase);
    fclose(OffsetsFid);
else
    %Offsets were previously defined.  Extract them (2nd line) for use now.
    FileValues = fscanf(OffsetsFid,'%g %g');
    HeadingAdjBase = FileValues(1); 
    TiltAdjBase = FileValues(2);
    fclose(OffsetsFid);
end
TiltLimit = 20; % Worst +/- tilt allowed; disable gimbal torque if exceeded
RollLimit = 20; % Worst +/- roll allowed; disable gimbal torque if exceeded
SoftLimitExceeded = 0; % State variable
HeadingAdjust = HeadingAdjBase;% adjust IMU output to correct for boresight misalignment
TiltAdjust = TiltAdjBase;      
ReceiveFreq= 71.5;   % GHz
AntDiameter= 0.3;    % meters
RcvLambda  = 3e8/ReceiveFreq/1e9; % meters
BeamWidth  = 70*RcvLambda/AntDiameter; % degrees
EarthRadius = 6371;  %km
halfpi = pi/2;
twopi  = 2*pi;
Movi2deg = 180/32767; % Conversion factor for Movi shaft encoder to degrees
MoviRC   = 180; % Full-scale rate in deg/sec (+/- 32767).  Set in Movi app.
HeadingError = 0;
PitchError = 0;
RollError = 0;
%
% Target locations 
MtHoodLat = 45.373611;
MtHoodLon = -121.695833;
MtHoodAlt = 3429; % meters
HoodRiverLat = 45.709;
HoodRiverLon = -121.511;
HoodRiverAlt = 500;
TiaraLat = 26.786774;
TiaraLon = -80.033367;
TiaraAlt = 135;
FreeportLat = 26.517238;
FreeportLon = -78.773512;
FreeportAlt = 60;
%
% Set initial target location based on hard-coded value (can be adjusted
% with keyboard command)
switch TargetID
    case 1
        TargetLat = TiaraLat;
        TargetLon = TiaraLon;
        TargetAlt = TiaraAlt;
    case 2
        TargetLat = FreeportLat;
        TargetLon = FreeportLon;
        TargetAlt = FreeportAlt;
    case 3
        TargetLat = MtHoodLat;
        TargetLon = MtHoodLon;
        TargetAlt = MtHoodAlt;
end

% Setup realtime status display window which also allows for keyboard
% interrupts using the Matlab figure attribute 'KeyPressFcn' associated
% with this realtime status display window, which must be active for the 
% keyboard interrupts to function properly as currently programmed.
StatusFigure = figure;
set(StatusFigure, 'KeyPressFcn', @myKeyPressFcn)
plot(TargetLon,TargetLat,'rs');
switch TargetID
    case 1
        axis([-81 -78 26 27]);
        UpperLeftLat = 27;
        UpperLeftLon = -81;
    case 2
        axis([-81 -78 26 27]);
        UpperLeftLat = 27;
        UpperLeftLon = -81;
    case 3
        axis([-122 -121 45 46]);
        UpperLeftLat = 46;
        UpperLeftLon = -122;
end
hold on
title('Current Pointing Scenario and Historical Track');
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');

HeadingRate = 0;
PitchRate = 0;
RollRate = 0;
BadGPScount = 0;

if SensorID == 0 % We have selected the VectorNav as the GPS/INS source
    % Open serial port to receive data from VectorNav
    InsPort = serial('COM5');
    InsPort.Baudrate = 115200;
    InsPort.StopBits = 1;
    InsPort.Parity = 'none';
    InsPort.FlowControl = 'none';
    fopen(InsPort);
    InsPort.Status
    %InsPort.ReadAsyncMode = 'manual';
   
else
    % Open serial port to receive data from InertialLabs GPS/INS
    LastReadTime = tic;
    InsPort = serial('COM13');
    InsPort.Baudrate = 460800;
    InsPort.StopBits = 1;
    InsPort.Parity = 'none';
    InsPort.FlowControl = 'none';
    InsPort.InputBufferSize = 256;
    fopen(InsPort);
    InsPort.Status
    BytesToRead = 109; % Initialize read requirement to ensure at least 1 msg
    %InsPort.ReadAsyncMode = 'manual';
end
%
% Open log file for test data capture
CurrentTimeString = char(datetime('now'));
filenamestr1 = ['GimbalTest_' CurrentTimeString(1:11) '_' ...
    CurrentTimeString(13:14) '_' CurrentTimeString(16:17) '.txt'];
LOGfid = fopen(filenamestr1,'w');
%
% Open log file that will only contain reported angles from GPS & gimbal
filenamestr2 = ['Angles_' CurrentTimeString(1:11) '_' ...
    CurrentTimeString(13:14) '_' CurrentTimeString(16:17) '.txt'];
ANGLEfid = fopen(filenamestr2,'w');
fprintf(ANGLEfid,'GPS angles (deg) for roll, pitch and yaw axes (XYZ), Gimbal angles for roll, pitch and yaw, MoviTlmBytes, time, and desired pitch and yaw.\n');
%
% Open log file that will only contain log of INS-DL data from gimbal
filenamestr3 = ['INSDL_' CurrentTimeString(1:11) '_' ...
    CurrentTimeString(13:14) '_' CurrentTimeString(16:17) '.txt'];
INSDLfid = fopen(filenamestr3,'w');
fprintf(INSDLfid,'INS-DL log data: Time, Adj.Heading, Adj.Pitch, Roll, Lat, Lon, Alt, Sol_type, SVcount\n');
fprintf(INSDLfid,'Note: Adj.Heading and Adj.Pitch account for ant. boresight offsets from GPS/IMU');
%
% Open log file for "scavanged" conical scan results
filenamestr5 = ['ConScanFile_' CurrentTimeString(1:11) '_' ...
    CurrentTimeString(13:14) '_' CurrentTimeString(16:17) '.txt'];
ConScanFID = fopen(filenamestr5,'w');
fprintf(ConScanFID,'Estimated GPS drift based on uncontrolled offpointing\n');
%
% Get nominal attitude from GPS/INS (assumes Movi is stable). 
if SensorID == 0 % Get attitude from VectorNav
    % this block of code should be pulled out and compiled as a separate fn
    INSdata   = fscanf(InsPort);
    StrLength = length(INSdata);
    ParamArray = strsplit(INSdata,',');
    %PreambleTest = sum(cell2mat(ParamArray(1,1)) == '$VNINS');
    % Preamble should be 6 char ('$VNINS') and length should be 142 characters
    if ( (StrLength ~= 142) || (length(cell2mat(ParamArray(1,1))) ~= 6 ) )
        fprintf(1,'\n ***Bad data on GPS/INS port; line 57.  StrLength = %4i \n', StrLength)
        OwnHeading = 0;
        OwnPitch = 0;
        OwnRoll = 0;
        OwnLat = 0;
        OwnLon = 0;
        OwnAlt = 0;
    else
        % Attitude and position parameters are in cells 5 through 10
        OwnHeading = str2double(cell2mat(ParamArray(1,5)));
        OwnPitch = str2double(cell2mat(ParamArray(1,6)));
        OwnRoll = str2double(cell2mat(ParamArray(1,7)));
        OwnLat = str2double(cell2mat(ParamArray(1,8)));
        OwnLon = str2double(cell2mat(ParamArray(1,9)));
        OwnAlt = str2double(cell2mat(ParamArray(1,10)));
    end
else % Get nominal attitude from INS-DL.  Note: if more than
    % one message could have been lost due to buffer overflow, dump the
    % contents of input buffer (which may be old) and collect new data.  If
    % at most one message may have been lost, try to use current buffer.
    % Data accumulates at the rate of 109 * 100 Hz = 10900 bytes/sec.
    %if (toc(LastReadTime)*10900 > (InsPort.InputBufferSize + 109)) && ...
    %        (InsPort.BytesAvailable > 0)
    %    DumpIO = fread(InsPort,InsPort.BytesAvailable,'uint8');
    %    BytesToRead = 218; % After overflow, gather 2 msg sets to ensure 1.
    %end
    ParseSuccess = 0;
    while ParseSuccess == 0
        if InsPort.BytesAvailable > 0
            DumpIO = fread(InsPort,InsPort.BytesAvailable,'uint8');
        end
        while InsPort.BytesAvailable < BytesToRead; continue; end
        INSinput = fread(InsPort,BytesToRead,'uint8');
        LastReadTime = tic;
        BytesRead = length(INSinput);
        % Find indices of bytes corresponding to first byte of header (0xAA).
        % Good header candidates cannot be too close to the end of the input
        % string.  The message is 109 bytes long.  thus, considering BytesRead,
        % a header candidate cannot be at any position > BytesRead - 108.
        HeaderIndices = find(INSinput(1:(BytesRead-108)) == 170);
        ParseSuccess = 0;
        for AAindex = length(HeaderIndices):-1:1
            if ParseSuccess == 1; break; end
            % There are three tests for checking proper message structure: 1)
            % second byte of header should have value 85 (0x55); 2) the next
            % four bytes should have decimal values = [1 87 107 0]; and 3) the
            % checksum should match the sum of bytes (AAindex+2):(AAindex+106)
            Byte0 = HeaderIndices(AAindex);
            if (INSinput(Byte0+1) == 85) && (INSinput(Byte0+2) == 1) && ...
                    (INSinput(Byte0+3)==87) && (INSinput(Byte0+4)==107) && ...
                    (INSinput(Byte0+5)==0) && ...
                    sum(uint16(INSinput((Byte0+2):(Byte0+106)))) == ...
                    typecast(uint8(INSinput((Byte0+107):(Byte0+108))),'uint16')
                ParseSuccess = 1;
                HeadingDeg = double(typecast(uint8(INSinput((Byte0+6):(Byte0+7)))','uint16'))*0.01;
                PitchDeg = double(typecast(uint8(INSinput((Byte0+8):(Byte0+9)))','int16'))*0.01;
                RollDeg  = double(typecast(uint8(INSinput((Byte0+10):(Byte0+11)))','int16'))*0.01;
                VDCinput = double(typecast(uint8(INSinput((Byte0+32):(Byte0+33)))','int16'))*0.01;
                TempDegC = double(typecast(uint8(INSinput((Byte0+34):(Byte0+35)))','int16'))*0.1;
                LatDeg = double(typecast(uint8(INSinput((Byte0+36):(Byte0+39)))','int32'))*0.0000001;
                LonDeg = double(typecast(uint8(INSinput((Byte0+40):(Byte0+43)))','int32'))*0.0000001;
                AltMeters = double(typecast(uint8(INSinput((Byte0+44):(Byte0+47)))','int32'))*0.01;
                GPSmsec = typecast(uint8(INSinput((Byte0+82):(Byte0+85)))','int32');
                GPSseconds = double(GPSmsec)*0.001; % Sec since midnight Sunday
                GNSS_info1 = dec2base(INSinput(Byte0+86),2,8);
                GNSS_info2 = dec2base(INSinput(Byte0+87),2,8);
                AngleSolType = mod(uint8(INSinput(Byte0+91)),100);
                GNSS_heading = double(typecast(uint8(INSinput((Byte0+92):(Byte0+93)))','uint16'))*0.01;
                SVcount = uint8(INSinput(Byte0+88));
                %
                % For May 2019 Movi implementation, INS-DL is mounted sideways to
                % conserve space.  So pitch/roll/yaw reports must be transformed.
                % See INS-DL ICD page 17 (Figure 1.5).
                OwnHeading = mod( (90 + HeadingDeg), 360);
                OwnPitch = -RollDeg;
                OwnRoll = PitchDeg;
                OwnLat = LatDeg;
                OwnLon = LonDeg;
                OwnAlt = AltMeters;
            end
            if ParseSuccess == 0; BytesToRead = 218; end
        end % End read of INS-DL
    end
end % End initial reading of nominal attitude from either sensor
%
% Plot current location and setup variables to update display later on
plot(OwnLon,OwnLat,'r.');
PreviousLon = OwnLon;
PreviousLat = OwnLat;
HeadingStr = ['Commanded Heading = ' num2str(0) '. Actual Heading = ' num2str(0)];
PitchStr   = ['Commanded Pitch = ' num2str(0) '. Actual Pitch = ' num2str(0)];
TrkErrStr  = ['Est. tracking error (1 sigma) is ' num2str(0) ' deg in yaw; ' num2str(0) ' deg in pitch'];
InsStateStr= ['SVcount = ' num2str(0) '; AngleSolType = ' num2str(0) '.'];
%
LastHeading = OwnHeading; % Setup of testing for heading "outliers"
LastPitch   = OwnPitch;
% Open serial port to write data to Movi.  See Freefly API v. 1.0.pdf 
s = serial('COM8');
s.Baudrate = 111111;
s.StopBits = 1;
s.Parity = 'none';
s.FlowControl = 'none';
fopen(s);
s.Status
%s.ReadAsyncMode = 'continuous'; % Default mode is 'continuous'

% Build unchanging CMD message and payload preamble bytes
Header = 'QX';
Length = uint8(26);
AttributeByte1 = uint8(149);    % 0x95
AttributeByte2 = uint8(2);      % 0x02
Options = uint8(66);            % 0x42 for API communication
TRID = uint8(0);                % Transmit Request ID (reserved)
RRID = uint8(0);                % Reply Request ID (reserved)
PayloadBinding = [uint8(0) uint8(0) uint8(0)]; % reserved
FrontMatter = [Header Length AttributeByte1 AttributeByte2 Options TRID RRID PayloadBinding];

% Build byte with gimbal control flags for euler angles and rate control
% Bits 7:6 = 00 implies euler angles and active control. Next three pairs
% of bits indicate control mode in x, y, z axes respectively.  0b00 means
% that control is deferred to another input source.  0b01 implies "rate"
% commands.  0b10 is "absolute" commands.  0b11 is "absolute/majestic".
% This last mode adds a user-configurable window and smoothing setting.

GimbalCntl = uint8(21); % Active Euler angles with rate control
%GimbalCntl = uint8(42); % Active Euler angles with abs. control on all axes
%GimbalCntl = uint8(0);  % Euler angles -- deferred control on all axes
%GimbalCntl = uint8(63); % Majestic control (absolute) on all axes
% Build 9 bytes of lens control flags (unused for this application)
LensCntl_1 = uint8(0);
LensCntl_2 = uint8(0);
LensCntl_3 = uint8(0);
LensCntl_4 = uint8(0);
LensCntl_5 = uint8(0);
LensCntl_6 = uint8(0);
LensCntl_7 = uint8(0);
LensCntl_8 = uint8(0);
LensCntl_9 = uint8(0);

% Initialize gimbal with "defer" cmd (GimbalCntl = 0) to avoid movement.  
% For absolute angles, gimbal angles are signed int16 values scaled to
% +/- 180 degrees.  The scale factor is 182.04 "counts" per degree.
% For rate cmds, full-scale is set in the Movi app (currently +/- 180
% deg/sec corresponding to +/- 32767), and specified in default parameters
% at start of script (above).  IMPORTANT: encoded number format appears to
% be twos-complement, so "-1" is encoded as 0xFFFF, not 0x8001.    
%
% The x, y and z axes are roll, tilt and pan. Positive values are clockwise
% Current code relies on rate commands with these values zeroed-out. 
GimbalRX = int16(0); % RX is the roll axis
GimbalRY = int16(0); % RY is the tilt axis
GimbalRZ = int16(0); % RZ is the pan axis
GimbalRR = int16(0); % Used for quaternion cmd mode (not used here)

% Send initialization command to wake up Movi... perform 10x or until a
% response is received (indicated by s.BytesAvailable > 0).  
KickCount = 0;
while s.BytesAvailable < 37  && KickCount < 10
    XmtMoviCmd(s,FrontMatter,AttributeByte1,AttributeByte2,...
        Options,GimbalCntl,GimbalRX,GimbalRY,GimbalRZ); % Kick the Movi
    fprintf(1,'\n Sent Movi initialization command: ');
    KickCount = KickCount + 1;
    pause(0.05);
end
if KickCount == 10
    fprintf(1,'\n** Could not wake up Movi; Check power and comm path.\n');
    fclose(InsPort);
    delete(InsPort);
    clear InsPort
    fclose(s);
    delete(s);
    clear s
    fclose(LOGfid);
    fclose(ANGLEfid);
    fclose(ConScanFID);
    return
end
%
% Parse Movi TLM response to get initial axis positions, and offsets w.r.t.
% GPS.  These can be used to drive the gimbal based on its own TLM.
MoviTLM = fread(s,s.BytesAvailable,'uint8'); 
BytesRead = length(MoviTLM);
% Check validity of response w.r.t. 'QX' preamble and checksum
% (stub) **BUG.  Maybe expand this input checking
if MoviTLM(1,1) ~= 81
    fprintf(1,'\nInitial Movi TLM corrupted during kickstart. No GPS offsets.\n');
else
    %
    % Parse TLM response
    Batt_A = 10.0 + MoviTLM(12,1)*0.1; %volts
    Batt_B = 10.0 + MoviTLM(13,1)*0.1; %volts
    GimbalStatus1 = dec2base(MoviTLM(14,1),2,8);
    GimbalStatus2 = dec2base(MoviTLM(15,1),2,8);
    % Form unsigned 16-bit position values; convert to signed degrees
    RXtlm = bin2dec([dec2bin(MoviTLM(18,1)) dec2bin(MoviTLM(19,1),8)]);
    GimbalRXdeg = RXtlm*Movi2deg;  % Correct answer if MSB == 0
    if (RXtlm > 32767); GimbalRXdeg = -(65536-RXtlm)*Movi2deg; end
    RYtlm = bin2dec([dec2bin(MoviTLM(20,1)) dec2bin(MoviTLM(21,1),8)]);
    GimbalRYdeg = RYtlm*Movi2deg;
    if (RYtlm > 32767); GimbalRYdeg = -(65536-RYtlm)*Movi2deg; end
    RZtlm = bin2dec([dec2bin(MoviTLM(22,1)) dec2bin(MoviTLM(23,1),8)]);
    GimbalRZdeg = RZtlm*Movi2deg;
    if (RZtlm > 32767); GimbalRZdeg = -(65536-RZtlm)*Movi2deg; end
    InitialRXdeg = GimbalRXdeg; LastGimbalRXdeg = InitialRXdeg;
    InitialRYdeg = GimbalRYdeg; LastGimbalRYdeg = InitialRYdeg;
    InitialRZdeg = GimbalRZdeg; LastGimbalRZdeg = InitialRZdeg;
end
        
% Start loop to send messages at 100 Hz.  
% Collect data sent from Movi Pro and GPS/INS, and then send a new command.
AlignedState = 0; % AlignedState = 1 when GPS and gimbal yaw delta is known
TimerOne = tic;
for LoopCount = 1:(Duration*100)
    pause(0.001); % This is needed to sense keypress (system bug??)
    if KEY_IS_PRESSED  % Service keyboard command
        switch KeyCmd
            case 'x' % Terminate execution; close files; plot results
                fprintf(1,'\n**Movi Command script terminated manually\n');
                KEY_IS_PRESSED = 0;
                KeyCmd = '';
                break;
                
            case 't'
                TorqueMode = 1;
                KEY_IS_PRESSED = 0;
                KeyCmd = '';
                
            case 'q'
                TorqueMode = 0; % Quiet mode; no torque
                KEY_IS_PRESSED = 0;
                KeyCmd = '';
                
            case 's' % Initiate wide RSSI scan and report/plot results
                % Cleanup from last scan, if needed
                if WideScanObj 
                    close(WideScanFigure);
                    fclose(WideScanFile);
                    WideScanObj = 0;
                end
                % Setup state machine for wide RSSI raster scan (+/- X deg)
                % Open data file for data.  Display figure is opened later.
                WideScanning = 1;
                CornerSeek = 1;
                LineID     = 0;
                CurrentYawDeltaTarget = -WideScanSpan;
                CurrentTiltDeltaTarget= WideScanSpan;
                CurrentTimeString = char(datetime('now'));
                filenamestr4 = ['WideScan_' CurrentTimeString(1:11) '_' ...
                    CurrentTimeString(13:14) '_' CurrentTimeString(16:17) '.txt'];
                WideScanFile = fopen(filenamestr4,'w');
                fprintf(WideScanFile,'Columns represent YawOffset, TiltOffset, RSSI, GPS msec\n');
                fprintf(1,'\nStarting wide RSSI scan.\n');
                WideScanFigure = figure; % Create figure for raw RSSI plot
                figure(StatusFigure); % Switch active figure for key cmds
                WideScanObj = 1;
                WideScanTimer = tic;
                % Note: the offsets are w.r.t. the GPS/IMU sensor frame
                KEY_IS_PRESSED = 0;
                KeyCmd = '';
                
            case 'a' % Apply results of last wide RSSI scan (and close fig)
                if (WideScanning == 1)
                    fprintf(1,'\nWarning: Wide Scan is not complete; adjustment values are not available.');
                    fprintf(1,'\n  You can wait for the scan to finish, or...');
                    fprintf(1,'\n  Type "c" to end scan and continue normal operation, or');
                    fprintf(1,'\n  Type "x" to kill the entire operating script.');
                    fprintf(1,'\n');
                else
                    % Set base offsets for current and future operations
                    OffsetsFid = fopen('BoresightOffsets.txt','a+');
                    frewind(OffsetsFid);
                    HeaderLine = fgetl(OffsetsFid);
                    if HeaderLine == -1
                        %We've opened an empty file.  Save with "factory offsets" for next use
                        fprintf(OffsetsFid,'Yaw and Pitch offsets (deg) for antenna boresight\n');
                        HeadingAdjBase= 0;   % degrees of clockwise yaw that must be negated, as 
                                             % determined by the last accepted RSSI scan (or user)
                        TiltAdjBase   = 0;   % degrees of upward tilt that must be negated, as 
                                             % determined by the last accepted RSSI scan (or user)
                        fprintf(OffsetsFid,'%f\t%f\n',HeadingAdjBase,TiltAdjBase);
                        fclose(OffsetsFid);
                    else
                        %Offsets were externally defined.  Extract them (2nd line) for use now.
                        [HeadingAdjBase, TiltAdjBase] = fscanf(OffsetsFid,'%g %g');
                        fclose(OffsetsFid);
                    end
                    %
                    % Also update "real-time" vernier adjustments to
                    % reflect this most recent WideScan.  Note: these
                    % vernier adjustments can be updated in "real-time" by
                    % the plus scan procedure, to track short-term GPS var.
                    HeadingAdjust = -HeadingAdjBase;
                    TiltAdjust = -TiltAdjBase;
                    if WideScanObj
                        close(WideScanFigure); 
                        WideScanObj = 0;
                    end
                    KEY_IS_PRESSED = 0;
                    KeyCmd = '';
                end
                
            case 'c' % Continue without applying results (and close fig)
                if WideScanObj
                    close(WideScanFigure); 
                    WideScanObj = 0;
                end
                WideScanning = 0;
                CornerSeek = 0;
                LineID = 0;
                KEY_IS_PRESSED = 0;
                KeyCmd = '';
                
            case 'k' % Kill Movi torque temporarily
                % Set GimbalCntl = 0b01010101 (decimal 85) to kill torque
                % with all axes still in rate mode.  Pause for 0.25 sec.
                fprintf(1,'\n**Torque killed for 0.25 seconds\n');
                KEY_IS_PRESSED = 0;
                KeyCmd = '';
                GimbalCntl = uint8(85);
                GimbalRZ = int16(0.); % Heading
                GimbalRY = int16(0.); % Pitch
                GimbalRX = int16(0.); % Roll
                XmtMoviCmd(s,FrontMatter,AttributeByte1,AttributeByte2,...
                    Options,GimbalCntl,GimbalRX,GimbalRY,GimbalRZ);
                pause(0.25);
                            
            case 'u' % Setup conditions for unwrap operation
                fprintf(1,'\nsetting up for unwrap operation.');
                UnwrapState = 1;
                RequiredUnwrap = UnwrapValue; % Test value (hardcoded)
                CurrentYaw = LastGimbalRZdeg;
                AccumUnwrap= 0;
                KEY_IS_PRESSED = 0;
                KeyCmd = '';
                                
            case '1'
                TargetLat = TiaraLat;
                TargetLon = TiaraLon;
                TargetAlt = TiaraAlt;
                axis([-81 -78 26 27]);
                UpperLeftLat = 27;
                UpperLeftLon = -81;
                KEY_IS_PRESSED = 0;
                KeyCmd = '';
            case '2'
                TargetLat = FreeportLat;
                TargetLon = FreeportLon;
                TargetAlt = FreeportAlt;
                axis([-81 -78 26 27]);
                UpperLeftLat = 27;
                UpperLeftLon = -81;
                KEY_IS_PRESSED = 0;
                KeyCmd = '';
            case '3'
                TargetLat = MtHoodLat;
                TargetLon = MtHoodLon;
                TargetAlt = MtHoodAlt;
                axis([-122 -121 45 46]);
                UpperLeftLat = 46;
                UpperLeftLon = -122;
                KEY_IS_PRESSED = 0;
                KeyCmd = '';
        end
    end
    % Every second (or so), update data on real-time display window if the
    % RSSI scan window is not open/present in the system.
    % Also, generate a temp (overwritten) status file for remote monitoring
    if fix(LoopCount/100) == LoopCount/100 && ~WideScanObj
        figure(StatusFigure);
        plot(PreviousLon,PreviousLat,'b.');
        plot(OwnLon,OwnLat,'r.');
        PreviousLon = OwnLon;
        PreviousLat = OwnLat;
        PrevHeadingStd = HeadingStd;
        PrevPitchStd = PitchStd;
        PrevRollStd = RollStd;
        HeadingAvgErr = mean(RecentErrors(:,1));
        HeadingStd  = std(RecentErrors(:,1));
        PitchAvgErr = mean(RecentErrors(:,2));
        PitchStd  = std(RecentErrors(:,2));
        RollStd  = std(RecentErrors(:,3));
        %
        % Overwrite old text in white, and replace with new values
        text(UpperLeftLon,(UpperLeftLat),HeadingStr,'Clipping','on','VerticalAlignment','top','Color','white');
        text(UpperLeftLon,(UpperLeftLat-0.05),PitchStr,'Clipping','on','VerticalAlignment','top','Color','white');
        text(UpperLeftLon,(UpperLeftLat-0.1),TrkErrStr,'Clipping','on','VerticalAlignment','top','Color','white');
        HeadingStr = ['Commanded Heading = ' num2str(fix(100*HeadingDeg)/100) '. Actual Heading = ' num2str(fix(100*WorkingHeading)/100)];
        PitchStr   = ['Commanded Pitch = ' num2str(fix(100*PitchDeg)/100) '. Actual Pitch = ' num2str(fix(100*(OwnPitch))/100)];
        TrkErrStr  = ['Est. tracking error (1 sigma) is ' num2str(fix(100*HeadingStd)/100) ' deg in yaw; ' num2str(fix(100*PitchStd)/100) ' deg in pitch'];
        text(UpperLeftLon,(UpperLeftLat),HeadingStr,'Clipping','on','VerticalAlignment','top');
        text(UpperLeftLon,(UpperLeftLat-0.05),PitchStr,'Clipping','on','VerticalAlignment','top');
        text(UpperLeftLon,(UpperLeftLat-0.1),TrkErrStr,'Clipping','on','VerticalAlignment','top');
        if SensorID == 1 % Add key status for INS-DL
            text(UpperLeftLon,(UpperLeftLat-0.15),InsStateStr,'Clipping','on','VerticalAlignment','top','Color','white');
            InsStateStr= ['SVcount = ' num2str(SVcount) '; AngleSolType = ' num2str(AngleSolType) '.'];
            text(UpperLeftLon,(UpperLeftLat-0.15),InsStateStr,'Clipping','on','VerticalAlignment','top');
        end
        StatusFid = fopen('PlatformStatusFile.txt','w');
        fprintf(StatusFid,'Incremental Platform Status Update\n');
        fprintf(StatusFid,'\nLast recorded GPS TimeOfWeek (sec) = %i',uint32(GPSseconds));
        fprintf(StatusFid,'\n%s',HeadingStr);
        fprintf(StatusFid,'\n%s',PitchStr);
        fprintf(StatusFid,'\n%s',TrkErrStr);
        if SensorID == 1; fprintf(StatusFid,'\n%s',InsStateStr); end
        fclose(StatusFid);
    end
    if s.BytesAvailable <= 36 % Missing or incomplete Movi TLM message
        %fprintf(1,'\nMovi comm issue at LoopCount = %i. BytesAvail = %i',LoopCount,s.BytesAvailable);
        Batt_A = 999;
        Batt_B = 999;
        GimbalRXdeg = 999;
        GimbalRYdeg = 999;
        GimbalRZdeg = 999;
        MoviTlmBytes = s.BytesAvailable;
    else % at least one TLM message was received.
        % Try to find a complete message by looking for a header where
        % there are enough additional data bytes for a full message.
        % Note: AT BEST, this is an ack and TLM from cmd issued last cycle.
        % Depending on timing of gimbal, comm, and this loop, the ack and
        % TLM may relate to a command issued in an even earlier cycle.
        MoviTLM = fread(s,s.BytesAvailable,'uint8'); 
        BytesRead = length(MoviTLM);
        MoviTlmBytes = BytesRead;
        HeaderIndices = find(MoviTLM(1:(BytesRead-36)) == 81);
        if isempty(HeaderIndices)
            % Insert flags in data fields to indicate no TLM, and go anyway
            Batt_A = 999;
            Batt_B = 999;
            GimbalRXdeg = 999;
            GimbalRYdeg = 999;
            GimbalRZdeg = 999;
        else
            %
            % Parse last apparent TLM response starting with header (81)
            % ***BUG.  We should also check parity check, and if no good,
            % examine other candidate message start points, if any.
            HeaderByte = HeaderIndices(length(HeaderIndices));
            MoviTLM = MoviTLM(HeaderByte:(HeaderByte+36),1);
            %fprintf(1,'\n LoopCount = %i.  Movi TLM successfully read.',LoopCount);
            %fprintf(1,'\n\t InsPort Status = %s; Bytes Avail = %i.',InsPort.Status,InsPort.BytesAvailable);
            Batt_A = 10.0 + MoviTLM(12,1)*0.1; %volts
            Batt_B = 10.0 + MoviTLM(13,1)*0.1; %volts
            GimbalStatus1 = dec2base(MoviTLM(14,1),2,8);
            GimbalStatus2 = dec2base(MoviTLM(15,1),2,8);
            if GimbalRXdeg<999 % This flag works for all axes
                LastGimbalRXdeg = GimbalRXdeg; 
                LastGimbalRYdeg = GimbalRYdeg;
                LastGimbalRZdeg = GimbalRZdeg;
            end
            % Form unsigned 16-bit position values; convert to signed degrees
            RXtlm = bin2dec([dec2bin(MoviTLM(18,1)) dec2bin(MoviTLM(19,1),8)]);
            GimbalRXdeg = RXtlm*Movi2deg;  % Correct answer if MSB == 0
            if (RXtlm > 32767); GimbalRXdeg = -(65536-RXtlm)*Movi2deg; end
            if (GimbalRXdeg<999) 
                DeltaRX = GimbalRXdeg - LastGimbalRXdeg; % Incremental roll
            end
            
            RYtlm = bin2dec([dec2bin(MoviTLM(20,1)) dec2bin(MoviTLM(21,1),8)]);
            GimbalRYdeg = RYtlm*Movi2deg;
            if (RYtlm > 32767); GimbalRYdeg = -(65536-RYtlm)*Movi2deg; end
            if (GimbalRYdeg<999) 
                DeltaRY = GimbalRYdeg - LastGimbalRYdeg; % Incremental pitch
            end
            
            RZtlm = bin2dec([dec2bin(MoviTLM(22,1)) dec2bin(MoviTLM(23,1),8)]);
            GimbalRZdeg = RZtlm*Movi2deg;
            if (RZtlm > 32767); GimbalRZdeg = -(65536-RZtlm)*Movi2deg; end
            if (GimbalRZdeg<999) 
                if sign(GimbalRZdeg) == sign(LastGimbalRZdeg)
                    DeltaRZ = GimbalRZdeg - LastGimbalRZdeg; % Incremental yaw
                else % We have "wrapped around" +/- 180 degrees on yaw axis
                    DeltaRZ = sign(LastGimbalRZdeg)*(360 - abs(GimbalRZdeg) - abs(LastGimbalRZdeg));
                end
            end
        end
    end
%
% Read data from GPS/INS. Note: this block of code should be pulled out and
% compiled as a separate function.  Also, we set a watchdog timer which
% will break out of the entire script and raise an alarm if no data is read
% for 10 seconds (failure of GPS/INS)
CommOutageTimer = tic;
    if SensorID == 0 % Read data from VectorNav
        while InsPort.BytesAvailable < 142
            if toc(CommOutageTimer) > 10; break; end
        end
        % We either have at least one msg, or the GPS/INS sensor has failed
        % In the event of sensor failure, exit the main loop and terminate.
        if toc(CommOutageTimer) > 10
            fprintf(1,'\nALARM: GPS/INS appears to have failed -- no comm\n'); 
            break; 
        end
        % Read successive INS reports until buffer is (nearly) empty, and use
        % the last full report
        INSdata = fgetl(InsPort); 
        StrLength = length(INSdata);
        ParamArray = strsplit(INSdata,',');
        if (length(cell2mat(ParamArray(1,1))) ~= 6)
            PreambleTest = 0;
        else
            PreambleTest = sum(cell2mat(ParamArray(1,1)) == '$VNINS');
        end
        % Preamble should be 6 char ('$VNINS'); we need first 11 parameters
        if ( (length(ParamArray) < 11) || (PreambleTest ~= 6 ) )
            fprintf(1,'\n ***Bad data on GPS/INS port (line 380); coasting.  StrLength = %4i PreambleTest = %4i\n', StrLength, PreambleTest)
            GoodINSdata = 0;
            BadGPScount = BadGPScount + 1;
        else
            % Attitude and position parameters are in cells 5 through 10
            OwnHeading = str2double(cell2mat(ParamArray(1,5)));
            OwnPitch = str2double(cell2mat(ParamArray(1,6)));
            OwnRoll = str2double(cell2mat(ParamArray(1,7)));
            OwnLat = str2double(cell2mat(ParamArray(1,8)));
            OwnLon = str2double(cell2mat(ParamArray(1,9)));
            OwnAlt = str2double(cell2mat(ParamArray(1,10)));
            if any(isnan([OwnHeading OwnPitch OwnRoll OwnLat OwnLon OwnAlt]))
                fprintf(1,'\n ***Bad data on GPS/INS port (line 392); coasting.  NaN detected\n')
                GoodINSdata = 0;
                BadGPScount = BadGPScount + 1;
            else% VN data appears to be good
                % Even if INS data is apparently good, there might be a large jump
                % in heading if GPS virtual compass was not properly aligned.  Look
                % for such a condition, and apply hysteresis to reject outliers but
                % allow for learning in the event prior measurements were wrong.
                if (abs(OwnHeading-LastHeading) < AlphaThreshold) || ...
                        ((360 - abs(OwnHeading-LastHeading)) < AlphaThreshold)
                    % Data appears to be good; also, no big jump in heading.
                    % Use the current value of OwnHeading and update hysteresis
                    % parameters to prepare for next iteration
                    GoodINSdata = 1;
                    DeltaHeading= OwnHeading-LastHeading;
                    if abs(DeltaHeading) > 180
                        DeltaHeading = (360-abs(DeltaHeading)) * (-sign(DeltaHeading));
                    end
                    DeltaPitch  = OwnPitch - LastPitch;
                    LastHeading = OwnHeading;
                    AlphaBetaOut= OwnHeading;
                else% Heading jump.  Setup alpha-beta tracker and use Movi
                    % TLM or LastHeading until alpha-beta tracker converges
                    % to currently-reported value (then jump to new value).
                    GoodINSdata = 0;
                    fprintf(1,'\n***Heading jump. Implementing hysteresis.');
                    fprintf(1,'\nLastHeading= %f; OwnHeading= %f; AlphaBeta= %f',LastHeading, OwnHeading, AlphaBetaOut);
                    AlphaBetaOut = 0.25*AlphaBetaOut + 0.75*OwnHeading;
                    if (abs(OwnHeading-AlphaBetaOut) < AlphaThreshold) || ...
                            ((360 - abs(OwnHeading-AlphaBetaOut)) < AlphaThreshold)
                        % Alpha-beta tracker has converged.  Assume jump
                        % in heading is real.  Update variables.
                        LastHeading = OwnHeading;
                        AlphaBetaOut= OwnHeading;
                        GoodINSdata = 1;
                    elseif AlignedState == 1 && GimbalRZdeg ~= 999
                        % Use Movi TLM short-term until GPS recovers
                        OwnHeading = mod((360 + GimbalRZdeg + YawDelta),360);
                    else
                        OwnHeading = LastHeading; % This is the "pre-jump" heading
                    end
                end % This ends the strategy for an observed heading jump with VN.
            end % End parsing of successfully-read VectorNav data
        end % End test to see if there is VN data to parse
    %
    else % Read data from INS-DL
        HeaderIndices = [];
        while isempty(HeaderIndices)
            if toc(CommOutageTimer) > 10; break; end
            while InsPort.BytesAvailable < BytesToRead; continue; end
            INSinput = fread(InsPort,BytesToRead,'uint8');
            LastReadTime = tic;
            BytesRead = length(INSinput);
            % Find indices of bytes corresponding to first byte of header (0xAA).
            % Good header candidates cannot be too close to the end of the input
            % string.  The message is 109 bytes long.  thus, considering BytesRead,
            % a header candidate cannot be at any position > BytesRead - 108.
            % If there are no good header candidates within the early part of
            % the input string, look for a later candidate and collect more data 
            HeaderIndices = find(INSinput == 170);
            if isempty(HeaderIndices); continue; end
            Sieve1 = (HeaderIndices < (BytesRead-107));
            Sieve2 = (HeaderIndices > (BytesRead-108));
            if any(Sieve1) % We should be able to parse a full INS-DL msg
                HeaderIndices = HeaderIndices(Sieve1);
                break;
            elseif any(Sieve2) % Candidate header, but we need more bytes
                BytesToRead = 109-(BytesRead-max(HeaderIndices(Sieve2))+1);
                while InsPort.BytesAvailable < BytesToRead; continue; end
                INSinput = [INSinput; fread(InsPort,BytesToRead,'uint8')];
                BytesRead = length(INSinput);
            end
        end
        % We either have some GPS data, or the GPS/INS sensor has failed
        % In the event of sensor failure, exit the main loop and terminate.
        if toc(CommOutageTimer) > 10
            fprintf(1,'\nALARM: GPS/INS appears to have failed -- no comm\n'); 
            break; 
        end
        ParseSuccess = 0;
        GoodINSdata = 0;
        for AAindex = length(HeaderIndices):-1:1
            if ParseSuccess == 1; break; end
            % There are three tests for checking proper message structure: 1)
            % second byte of header should have value 85 (0x55); 2) the next
            % four bytes should have decimal values = [1 87 107 0]; and 3) the
            % checksum should match the sum of bytes (AAindex+2):(AAindex+106)
            Byte0 = HeaderIndices(AAindex);
            if (INSinput(Byte0+1) == 85) && (INSinput(Byte0+2) == 1) && ...
                    (INSinput(Byte0+3)==87) && (INSinput(Byte0+4)==107) && ...
                    (INSinput(Byte0+5)==0) && ...
                    sum(uint16(INSinput((Byte0+2):(Byte0+106)))) == ...
                    typecast(uint8(INSinput((Byte0+107):(Byte0+108))),'uint16')
                ParseSuccess = 1;
                HeadingDeg = double(typecast(uint8(INSinput((Byte0+6):(Byte0+7)))','uint16'))*0.01;
                PitchDeg = double(typecast(uint8(INSinput((Byte0+8):(Byte0+9)))','int16'))*0.01;
                RollDeg  = double(typecast(uint8(INSinput((Byte0+10):(Byte0+11)))','int16'))*0.01;
                VDCinput = double(typecast(uint8(INSinput((Byte0+32):(Byte0+33)))','int16'))*0.01;
                TempDegC = double(typecast(uint8(INSinput((Byte0+34):(Byte0+35)))','int16'))*0.1;
                LatDeg = double(typecast(uint8(INSinput((Byte0+36):(Byte0+39)))','int32'))*0.0000001;
                LonDeg = double(typecast(uint8(INSinput((Byte0+40):(Byte0+43)))','int32'))*0.0000001;
                AltMeters = double(typecast(uint8(INSinput((Byte0+44):(Byte0+47)))','int32'))*0.01;
                GPSmsec = typecast(uint8(INSinput((Byte0+82):(Byte0+85)))','int32');
                GPSseconds = double(GPSmsec)*0.001; % Sec since midnight Sunday
                GNSS_info1 = dec2base(INSinput(Byte0+86),2,8);
                GNSS_info2 = dec2base(INSinput(Byte0+87),2,8);
                AngleSolType = mod(uint8(INSinput(Byte0+91)),100);
                GNSS_heading = double(typecast(uint8(INSinput((Byte0+92):(Byte0+93)))','uint16'))*0.01;
                SVcount = uint8(INSinput(Byte0+88));
                %
                % For May 2019 Movi implementation, INS-DL is mounted sideways to
                % conserve space.  So pitch/roll/yaw reports must be transformed.
                % See INS-DL ICD page 17 (Figure 1.5).
                OwnHeading = mod( (90 + HeadingDeg), 360);
                OwnPitch = -RollDeg;
                OwnRoll = PitchDeg;
                OwnLat = LatDeg;
                OwnLon = LonDeg;
                OwnAlt = AltMeters;
                TimeStamp = GPSseconds;
                if any(isnan([OwnHeading OwnPitch OwnRoll OwnLat OwnLon OwnAlt]))
                    fprintf(1,'\n ***Bad data on GPS/INS port (line 430); coasting.  NaN detected\n')
                    GoodINSdata = 0;
                    BadGPScount = BadGPScount + 1;
                else
                    %fprintf(1,'\n LoopCount = %i.  INS-DL successfully parsed.',LoopCount);
                    fprintf(INSDLfid,'%15.3f\t%8.2f\t%8.2f\t%8.2f\t%12.5f\t%12.5f\t%8.2f\t%i\t%i\n',...
                        TimeStamp,OwnHeading,OwnPitch,OwnRoll,OwnLat,OwnLon,OwnAlt,AngleSolType,SVcount);
                    %
                    % Even if INS data is apparently good, there might be a large jump
                    % in heading if GPS virtual compass was not properly aligned.  Look
                    % for such a condition, and apply hysteresis to reject outliers but
                    % allow for learning in the event prior measurements were wrong.
                    if AngleSolType == 50 || (abs(OwnHeading-LastHeading) < AlphaThreshold) || ...
                            ((360 - abs(OwnHeading-LastHeading)) < AlphaThreshold)
                        % Data appears to be good; also, no big jump in heading.
                        % Use the current value of OwnHeading and update hysteresis
                        % parameters to prepare for next iteration
                        GoodINSdata = 1;
                        DeltaHeading= OwnHeading-LastHeading;
                        if abs(DeltaHeading) > 180
                            DeltaHeading = (360-abs(DeltaHeading)) * (-sign(DeltaHeading));
                        end
                        DeltaPitch  = OwnPitch - LastPitch;
                        LastHeading = OwnHeading;
                        AlphaBetaOut= OwnHeading;
                    else% Use old heading or Movi TLM (if previously 
                        % aligned) until the output of an alpha-beta 
                        % tracking filter comes close to the current reading
                        GoodINSdata = 0;
                        fprintf(1,'\n***Heading jump. Implementing hysteresis.');
                        fprintf(1,'\nLastHeading= %f; OwnHeading= %f; AlphaBeta= %f',LastHeading, OwnHeading, AlphaBetaOut);
                        AlphaBetaOut = 0.25*AlphaBetaOut + 0.75*OwnHeading;
                        if (abs(OwnHeading-AlphaBetaOut) < AlphaThreshold) || ...
                                ((360 - abs(OwnHeading-AlphaBetaOut)) < AlphaThreshold)
                            % Alpha-beta tracker has converged.  Exit
                            % hysteresis behavior and update variables
                            LastHeading = OwnHeading;
                            AlphaBetaOut= OwnHeading;
                            GoodINSdata = 1;
                        elseif AlignedState == 1 && GimbalRZdeg ~= 999
                            % Use Movi TLM short-term until GPS recovers
                            OwnHeading = mod((360 + GimbalRZdeg + YawDelta),360);
                        else
                            OwnHeading = LastHeading; % This is the "pre-jump" heading
                        end
                    end % This ends the strategy for an observed heading jump with INS-DL.  
                end
                %
                % Compute BytesToRead at next read operation
                if BytesRead < InsPort.InputBufferSize % Buffer didn't overflow
                    StragglerBytes = mod((BytesRead-Byte0+1),109); % in buffer
                    LostBytes = mod((109-StragglerBytes),109); % not yet rcv'd
                    BytesToRead = 109 + LostBytes;
                else % Buffer overflowed on last read; must collect 218 bytes.
                    BytesToRead = 218;
                end
            end
            %if ParseSuccess == 0; BytesToRead = 218; end
        end % End read of data from INS-DL (parsing from header byte)
    end % End read of data from GPS/IMU sensor, of either type
    %
    % Check for exceedance of soft limits on pitch and roll.  If either
    % is exceeded by more than 1 deg, set state variable to kill torque
    if (abs(OwnPitch) > (TiltLimit+1)) || (abs(OwnRoll) > (RollLimit+1))
        SoftLimitExceeded = 1;
    else
        SoftLimitExceeded = 0;
    end

    if GoodINSdata && ~WideScanning  % Adjust reported heading and pitch by current offsets
        OwnHeading = mod((360 + OwnHeading + HeadingAdjust),360);
        OwnPitch   = OwnPitch + TiltAdjust;
    end
    %
    % Check alignment between GPS/IMU and gimbal.  If possible, align
    % such that GimbalRZdeg + YawDelta = OwnHeading from GPS/IMU.  This
    % will allow very short-term gimbal control based on gimbal telemetry, 
    % but only if we also have AIS heading 
    if GoodINSdata == 1 && GimbalRZdeg ~= 999
        YawDelta = mod(OwnHeading - mod((GimbalRZdeg+360),360),360);
        % Determine orientation of YawDelta (CW or CCW). If YawDelta is
        % greater than 180 deg, we need to go CCW.  
        if YawDelta > 180
            YawDelta = YawDelta - 360;
        end
        PitchDelta = OwnPitch - GimbalRYdeg;
        RollDelta  = OwnRoll - GimbalRXdeg;
        AlignedState = 1;
        GimbalCoastTimer = tic;
    elseif toc(GimbalCoastTimer) > GimbalCoastTime
        AlignedState = 0;
    end

% Calculate heading and pitch needed to point at target
    TargetColat = pi*(90-TargetLat)/180; %radians
    OwnColat = pi*(90-OwnLat)/180;
    DeltaLon = pi*abs(TargetLon - OwnLon)/180;
    %
    % Use Law of Cosines for Spherical Triangles to get Great Circle arc
    GCarc = acos( cos(TargetColat)*cos(OwnColat) + ...
        sin(TargetColat)*sin(OwnColat)*cos(DeltaLon) ); %radians
    Range = GCarc * EarthRadius * 1000; %meters
    %
    % Use Law of Sines for Spherical Triangles to get heading on 0 -> pi/2
    Heading = asin( sin(DeltaLon)*sin(TargetColat)/sin(GCarc) ); %radians
    if TargetColat > OwnColat % Target is "south" of us
        Heading = pi - Heading;
    end
    if TargetLon < OwnLon % Target is west of us.
        Heading = twopi - Heading;
    end
    HeadingDeg = 180*Heading/pi;
    %
    % Use Law of Cosines for plane triangles to get LOS distance
    TargetRadius = EarthRadius*1000 + TargetAlt;
    OwnRadius = EarthRadius*1000 + OwnAlt;
    LOSrange = sqrt(TargetRadius^2 + OwnRadius^2 - 2*TargetRadius*OwnRadius*cos(GCarc));
    % Use Law of Sines to get pitch angle w.r.t. horizontal plane (radians)
    Pitch = halfpi - asin( sin(GCarc) * TargetRadius / LOSrange); %inverted
    if ( (TargetRadius^2) < (LOSrange^2 + OwnRadius^2) ) %Need to look down
        Pitch = -Pitch;
    end
    PitchDeg = 180*Pitch/pi;
    %
    % Calculate current offpointing relative to current target, and
    % preserve magnitude of old error to see if error is growing or not
    OldHeadingError = HeadingError;
    OldPitchError = PitchError;
    OldRollError = RollError;
    WorkingHeading = OwnHeading;
    HeadingError = mod((360 + HeadingDeg - WorkingHeading), 360);
    PitchError = OwnPitch - PitchDeg;
    RollError = OwnRoll;
    if UseGimbalYaw == 1  % Overwrite errors with Gimbal attitude data
        GimbalRZtarget = InitialRZdeg + HeadingError;
        GimbalRYtarget = InitialRYdeg + PitchError;
        GimbalRXtarget = InitialRXdeg + RollError;
        HeadingError = -mod((360 + GimbalRZtarget - GimbalRZdeg), 360);
        PitchError = -(GimbalRYtarget - GimbalRYdeg);
        RollError = -(GimbaxlRXtarget - GimbalRXdeg);
    end
    if HeadingError > 180
        HeadingError = 360 - HeadingError;
    end
    % Determine orientation of heading error (CW or CCW).  If sum of
    % current (desired) HeadingDeg + HeadingError = OwnHeading (converted
    % to a positive number modulo 360 deg), the heading error is "positive"
    % and we should go CCW with a negative yaw rate command.  On the other
    % hand, if there is no alignment, we should go CW.
    if abs(mod((HeadingDeg+HeadingError),360) - mod((WorkingHeading+360),360)) > 0.05
        HeadingError = -HeadingError;
    end
    %
    % Read RSSI and log running data in ConScanArray and RecentErrors for 
    % real-time monitoring of GPS drift
    if WideScanning % Reset small-scale RSSI data gathering to start fresh
        ConScanPointer = 1;
    elseif ConScanPointer ~= ConScanRowMax % Populate real-time arrays
        RSSI = ReadRSSI(RslSimState,HeadingError,PitchError);
        ConScanArray(ConScanPointer,:) = [HeadingError PitchError RSSI];
        RecentErrors(ConScanPointer,:) = [HeadingError PitchError RollError];
        ConScanPointer = ConScanPointer + 1;
    else% We are not WideScanning, and we have filled the RSSI array
        % Compute apparent drift in GPS; write yaw/pitch results to file
        [RowMax, ColMax] = size(ConScanArray);
        ConScanPointer = 1;        
        % Align yaw/pitch data with RSSI data by deleting EstimatedLatency 
        % data points from the beginning of the RSSI data, and the same 
        % number from the end of the yaw and pitch data
        ConScanArray2 = [ConScanArray(1:(RowMax-EstimatedLatency),1) ...
            ConScanArray(1:(RowMax-EstimatedLatency),2) ...
            ConScanArray((1+EstimatedLatency):RowMax,3)];
            RowMax = RowMax - EstimatedLatency;
            MinX = min(ConScanArray2(:,1)); MaxX = max(ConScanArray2(:,1));
            MinY = min(ConScanArray2(:,2)); MaxY = max(ConScanArray2(:,2));
            %
            % The null hypothesis is that the existing boresight offsets are still
            % good.  Evaluate the MSE of RSSI values against the null hypothesis, to
            % use as a comparative metric later
            RelativeOffsets = sqrt( (ConScanArray2(:,1)).^2 + (ConScanArray2(:,2)).^2 );
            NormalizedOffsets = RelativeOffsets/BeamWidth;
            OffpointLossRatios= (sin(pi*NormalizedOffsets)./(pi*NormalizedOffsets)).^2;
            OffpointLossesDB = max(-60, 10*log10(OffpointLossRatios)); % negative values.
            ScaledLossesVolts= 4 + OffpointLossesDB/20;
            MSEnull = mean((ScaledLossesVolts - ConScanArray2(:,3)).^2);
            %
            % Fit a second-order surface to these data points (mostly inside main lobe)
            SurfCoeffs = fit2dPolySVD(ConScanArray2(:,1),ConScanArray2(:,2),ConScanArray2(:,3),2);
            % Evaluate the surface on a grid with 0.01 degree spacing; find peak value
            BoresightOffsets = zeros(1,2);
            Yvalues  = MaxY:(-0.01):MinY;
            Xvalues = MinX:(0.01):MaxX;
            RowMax = length(Yvalues); 
            ColMax = length(Xvalues);
            MaxValue = 0;
            for ColumnID = 1:ColMax
                HeadingOffset = Xvalues(ColumnID);
                Xcolumn = HeadingOffset * ones(RowMax,1);
                SurfaceValues = eval2dPoly(Xcolumn,Yvalues',SurfCoeffs);
                [MaxValueInCol, MaxValueIndex] = max(SurfaceValues);
                if MaxValueInCol > MaxValue
                    MaxValue = MaxValueInCol;
                    BoresightOffsets(1,:) = [HeadingOffset Yvalues(MaxValueIndex)];
                else % Once we've passed the peak, no need to check remainder of surf.
                    break
                end
            end
            %
            % Assess MSE for the hypothesis with adjusted offsets, and compare to the
            % MSEnull associated with the existing offsets
            RelativeOffsets = sqrt( (ConScanArray2(:,1)-BoresightOffsets(1,1)).^2 + ...
                (ConScanArray2(:,2) - BoresightOffsets(1,2)).^2 );
            NormalizedOffsets = RelativeOffsets/BeamWidth;
            OffpointLossRatios= (sin(pi*NormalizedOffsets)./(pi*NormalizedOffsets)).^2;
            OffpointLossesDB = max(-60, 10*log10(OffpointLossRatios)); % negative values.
            ScaledLossesVolts= 4 + OffpointLossesDB/20;
            MSEprime = mean((ScaledLossesVolts - ConScanArray2(:,3)).^2);
            if MSEprime >= MSEnull 
                % New offsets appear to be no better than current/older offets.  Set
                % calculated BoresightOffsets to 0 to keep them unchanged.
                BoresightOffsets(1,:) = [0 0];
            end
            % For the real-time operating system in the Pi, "BoresightOffsets" is all
            % we need.  These values can be used to adjust the current values for
            % HeadingAdjust and TiltAdjust (subtract the BoresightOffsets values).
            % HOWEVER, AT THIS TIME, WE MERELY LOG THE RESULTS FOR LATER
            % ANALYSIS.
        fprintf(ConScanFID,'\nAt LoopCount= %i, Best Yaw/Pitch Offsets = %f deg; %f deg',...
            LoopCount,BoresightOffsets(1,1),BoresightOffsets(1,2));
    end
        
    %
    % If we are in WideScanning mode, adjust target pointing angles
    % according to current detailed status of the raster scan.  There are N
    % scan lines where N = 2*WideScanSpan/ScanSpan. We first seek the upper
    % left corner of the raster scan FOV. The first, and all odd-numbered,
    % lines shift yaw target CCW by WideScanSpan; even numbered lines
    % toggle CW by WideScanSpan.  Each line toggles pitch by -ScanSpan.
    % Precise line following is not needed... we gather data and fit a
    % nominal antenna pattern to the available data.
    if WideScanning
        if CornerSeek
            HeadingOffset= HeadingError; %Target offset from GPS/IMU axis
            PitchOffset  = PitchError;
            RSSI = ReadRSSI(RslSimState,HeadingOffset,PitchOffset);
            HeadingError = WideScanSpan + HeadingOffset;%Error to corner
            PitchError = -WideScanSpan + PitchOffset;
            if ((abs(HeadingError)<0.5) && (abs(PitchError)<0.5)) || ...
                    (abs(HeadingError)<1 && abs(PitchError)<1 && ...
                    abs(DeltaRY) < 0.1 && abs(DeltaRZ) < 0.1)
                % We are very close to the upper left corner, or we are
                % close, but "in the doldrums". So start raster scan
                CornerSeek = 0;
                LineTimer  = tic;
                LineID = LineID + 1;
            end
        else % We are somewhere in the midst of the raster scan process
            HeadingOffset= HeadingError; %Real target offset from GPS/IMU axis
            PitchOffset  = PitchError;
            if (fix(LineID/2) == LineID/2) % Even-numbered row scans CCW
                HeadingError = HeadingOffset + WideScanSpan; 
            else  % Odd-numbered row scans CW
                HeadingError = HeadingOffset - WideScanSpan; 
            end
            PitchError = PitchError - WideScanSpan + LineID*ScanSpan;
            RSSI = ReadRSSI(RslSimState,HeadingOffset,PitchOffset);
            %
            % Throw away every fifth reading to simulate RPi com issues
            if fix(LoopCount/5) ~= (LoopCount/5)
                fprintf(WideScanFile,'\n%f\t%f\t%f\t%i',HeadingOffset,PitchOffset,RSSI,GPSmsec);
            end
            if ((abs(HeadingError)<0.5) && (abs(PitchError)<0.5)) || ...
                    (abs(HeadingError)<1 && abs(PitchError)<1 && ...
                    abs(DeltaRY) < 0.1 && abs(DeltaRZ) < 0.1) || ...
                    toc(LineTimer) > 5
                % We are close to the end of a line; go to next line or end
                LineID = LineID + 1;
                LineTimer = tic;
                if LineID > 2*WideScanSpan/ScanSpan 
                    % We're done. 
                    % Close WideScanFile and reopen for read access only.
                    % Generate raw visualization and print data to allow 
                    % operator to compute antenna boresight offsets 
                    % externally, and pass offset data back into this
                    % script if desired.
                    fclose(WideScanFile);
                    WideScanFile = fopen(filenamestr4,'r');
                    HeaderLine = fgetl(WideScanFile);
                    RssiData = fscanf(WideScanFile,'%g %g %g %g',[4 inf])';
                    [RowMax, ~] = size(RssiData);
                    figure(WideScanFigure);
                    plot3(RssiData(:,1),RssiData(:,2),RssiData(:,3),'b.');
                    hold on
                    xlabel('Heading Offset (deg)');
                    ylabel('PitchOffset (deg)');
                    zlabel('RSSI voltage');
                    title('Wide Scan Calibration Results (Raw Data)');
                    TextStr1 = 'WideScan Complete';
                    TextStr2 = 'Raw data in latest WideScan file';
                    text(-5, 5, 4, TextStr1);
                    text(-5, 5, 3.75, TextStr2);
                    WideScanning = 0;
                    fclose(WideScanFile);
                    fprintf(1,'\nWide Raster Scan consumed %f seconds.\n',toc(WideScanTimer));
                    fprintf(1,'Filename for WideScan data is: %s\n',filenamestr4);
                    
                end
            else
                % We're still heading to the end of a line.  Keep going.
            end
            % Done with the line, and possibly the entire scan
        end
    end
    %
    % If we are coasting on missed GPS/INS data, issue "coast" cmd.
    % If we are unwrapping the cable, slew fast until complete.
    % Otherwise, generate new commands in degrees or degrees/second.  For
    % Majestic coast, cmds should be set to zero (they will be interpreted
    % as absolute offset commands).
    if (GoodINSdata == 0) 
        GimbalCntl = uint8(21); % 21 = rate hold; 63 = position hold
        GimbalRZ = int16(0.); % Heading
        GimbalRY = int16(0.); % Pitch
        GimbalRX = int16(0.); % Roll
    elseif (UnwrapState == 1)
        AccumUnwrap = AccumUnwrap + DeltaHeading; % Use GPS/INS data
        if abs(AccumUnwrap) < abs(RequiredUnwrap)
            GimbalCntl = uint8(21); 
            HeadingRate = 30 * sign(UnwrapValue);
            PitchRate = PcontrolFnY(PitchError, OldPitchError);
            RollRate = PcontrolFnX(RollError,OldRollError);
            GimbalRZ = int16(HeadingRate*32767/MoviRC);
            %GimbalRY = int16(PitchRate*32767/MoviRC); 
            %GimbalRX = int16(RollRate*32767/MoviRC);
            %GimbalRZ = int16((CurrentYaw + UnwrapValue)*32767/180);
            GimbalRY = int16(0);
            GimbalRX = int16(0);
        else
            UnwrapState = 0;
        end
    else
        GimbalCntl = uint8(21);
        % GimbalCntl should be one of these values for rate or offset cmds
        % unless soft tilt/roll limits are exceeded (when it should be 85).
        % 21 = rate control on all axes
        % 31 = rate control on roll; majestic hold on others
        % 42 = absolute angle offset commands on all axes
        % 55 = rate control on pitch; majestic hold on others
        % 61 = rate control on yaw; majestic hold on others
        % 63 = absolute, majestic offset commands on all axes
        %
        % Note: for absolute (majestic) offset commands, which are selected
        % using GimbalCntl values of 42 or 63, this code applies a maximum
        % commanded offset of 10 degrees (for any single command) and a
        % deadband of +/- 0.1 degrees.
        %
        % Initially, set all cmd params to zero to simplify following code.
        HeadingRate = 0; HeadingOffset = 0; GimbalRZ = int16(0);
        PitchRate = 0; PitchOffset = 0; GimbalRY = int16(0);
        RollRate = 0; RollOffset = 0; GimbalRX = int16(0);
        %
        % Test logic for scavenging small errors and doing random jitter.
        % If recent estimated errors in either heading or yaw are less
        % than 0.2 degrees, add a random 0.5 degree perturbation
        % at 2 Hz to provide some data for estimating GPS drift.
        if TestConScan && ~WideScanObj
            % Twice per second (and only then) set random perturbations
            if(fix(LoopCount/50)==(LoopCount/50))
                RandomShifts=rand(1,2);
                fprintf(ConScanFID,'\nAt LoopCount = %i, RandomShifts = %i, %i',...
                    LoopCount, sign(RandomShifts(1,1)-0.5), sign(RandomShifts(1,2)-0.5));
            end
            % If recent variations have been small, and errors centered on
            % zero, increase errors a little for the next 0.5 seconds, 
            % in the random directions generated.  Alternatively, if
            % variations are small but errors are skewed one way or the
            % other, push the system back to get some errors straddling the
            % nominal boresight
            if (HeadingStd < 0.2) && (abs(HeadingAvgErr) < 0.2)
                HeadingError = HeadingError + 0.5*sign(RandomShifts(1,1) - 0.5);
            elseif (HeadingStd < 0.2) && (abs(HeadingAvgErr) >= 0.2)
                HeadingError = HeadingError + 0.5*sign(HeadingAvgErr);
            end
            if (PitchStd < 0.2) && (abs(PitchAvgErr) < 0.2)
                PitchError = PitchError + 0.5*sign(RandomShifts(1,2) - 0.5);
            elseif (HeadingStd < 0.2) && (abs(PitchAvgErr) >= 0.2)
                PitchError = PitchError + 0.5*sign(PitchAvgErr);
            end
        end
        %
        % Try not to exceed soft limits on tilt and roll.  If a limit is
        % exceeded by more than one degree, kill torque on all axes.
        if (abs(OwnPitch) > TiltLimit) && (sign(PitchError) ~= sign(OwnPitch))
            PitchError = 0;
        end
        if (abs(OwnRoll) > RollLimit) && (sign(RollError) ~= sign(OwnRoll))
            RollError = 0;
        end
        if SoftLimitExceeded == 1 % Kill torque on all axes if over limit
            GimbalCntl = uint8(85);
        end
        if TorqueMode == 0 % User has requested no torque on motors
            GimbalCntl = uint8(85);
        end            
        
        switch GimbalCntl
            case 21
                HeadingRate = PcontrolFnZ(HeadingError,OldHeadingError);
                PitchRate = PcontrolFnY(PitchError,OldPitchError);
                RollRate = PcontrolFnX(RollError,OldRollError);
                GimbalRZ = int16(HeadingRate*32767/MoviRC);
                GimbalRY = int16(PitchRate*32767/MoviRC); 
                GimbalRX = int16(RollRate*32767/MoviRC);
            case 31
                RollRate = PcontrolFnX(RollError,OldRollError);
                GimbalRX = int16(RollRate*32767/MoviRC);
            case 42
                % ***BUG Absolute offset commands not working yet
                if (abs(HeadingError) < 0.1); HeadingError = 0; end
                if (abs(HeadingError) > 10); HeadingError = 10*sign(HeadingError); end
                HeadingOffset = HeadingError/2;
                %GimbalRZ = int16(HeadingOffset*32767/180);
                GimbalRZcmd = OwnHeading - HeadingOffset; % incr. goal
                GimbalRZcmd =  - HeadingOffset; % incr. goal
                if abs(GimbalRZcmd) > 180
                    GimbalRZcmd= -sign(GimbalRZcmd)*(360-abs(GimbalRZcmd));
                end
                GimbalRZ = int16((GimbalRZcmd)*32767/180);
                if (abs(PitchError) < 0.1); PitchError = 0; end
                if (abs(PitchError) > 10); PitchError = 10*sign(PitchError); end
                PitchOffset = PitchError/2;
                GimbalRY = int16(OwnPitch - PitchOffset*32767/180);
                if (abs(RollError) < 0.1); RollError = 0; end
                if (abs(RollError) > 10); RollError = 10*sign(RollError); end
                RollError = RollError/2;
                GimbalRX = int16(-RollError*32767/180);
                GimbalRX = int16(0); % Zero out roll for initial test
                %GimbalRY = int16(0); % Zero out pitch for initial test
            case 55
                PitchRate = PcontrolFnY(PitchError,OldPitchError);
                GimbalRY = int16(PitchRate*32767/MoviRC/10); % Positive error (low)
                if PitchRate < 0; GimbalRY = int16(PitchRate*32767/MoviRC); end
            case 61
                HeadingRate = PcontrolFnZ(HeadingError,OldHeadingError);
                GimbalRZ = int16(HeadingRate*32767/MoviRC);
            case 63
                % ***BUG Majestic offset commands not working yet
                if (abs(HeadingError) < 0.1); HeadingError = 0; end
                %if (abs(HeadingError) > 10); HeadingError = 10*sign(HeadingError); end
                HeadingOffset = HeadingError/2;
                %GimbalRZ = int16(HeadingOffset*32767/180);
                GimbalRZcmd = OwnHeading - HeadingOffset; % incr. goal
                if abs(GimbalRZcmd) > 180
                    GimbalRZcmd= -sign(GimbalRZcmd)*(360-abs(GimbalRZcmd));
                end
                GimbalRZ = int16((GimbalRZcmd)*32767/180);
                if (abs(PitchError) < 0.1); PitchError = 0; end
                if (abs(PitchError) > 10); PitchError = 10*sign(PitchError); end
                PitchOffset = PitchError/2;
                GimbalRY = int16(-PitchOffset*32767/180);
                if (abs(RollError) < 0.1); RollError = 0; end
                if (abs(RollError) > 10); RollError = 10*sign(RollError); end
                RollError = RollError/2;
                GimbalRX = int16(-RollError*32767/180);
                GimbalRX = int16(0);
                GimbalRY = int16(0);
            case 85
                GimbalRZ = int16(0.); % Heading
                GimbalRY = int16(0.); % Pitch
                GimbalRX = int16(0.); % Roll
        end
    end
    
    % Issue a standard command to the Movi using selected GimbalCntl mode
    % Note: this may include adjustments for WideScanning
    XmtMoviCmd(s,FrontMatter,AttributeByte1,AttributeByte2,...
        Options,GimbalCntl,GimbalRX,GimbalRY,GimbalRZ);
        
    
        
    fprintf(LOGfid,'\nGPS data:  \t%6.2f \t%6.2f \t%6.2f',OwnRoll,OwnPitch,OwnHeading);
    fprintf(LOGfid,'\nMovi data:  \t%6.2f \t%6.2f \t%6.2f',GimbalRXdeg,GimbalRYdeg,GimbalRZdeg);
    fprintf(LOGfid,'\nErrors are: \t%6.2f \t%6.2f \t%6.2f',RollError,PitchError,HeadingError);
    fprintf(LOGfid,'\nGimbalCntl parameter = %i',GimbalCntl);
    fprintf(LOGfid,'\nRates are:  \t%6.2f \t%6.2f \t%6.2f',RollRate,PitchRate,HeadingRate);
    %fprintf(LOGfid,'\nOffsets are:  \t%6.2f \t%6.2f \t%6.2f',RollOffset,PitchOffset,HeadingOffset);
    fprintf(LOGfid,'\nCMD values: %10i %7i %7i\n',GimbalRX, GimbalRY,GimbalRZ);
    fprintf(ANGLEfid,'%8.2f\t%8.2f\t%8.2f\t',OwnRoll,OwnPitch,OwnHeading);
    fprintf(ANGLEfid,'%8.2f\t%8.2f\t%8.2f\t%i\t%8.3f\t%8.2f\t%8.2f\n',...
            GimbalRXdeg,GimbalRYdeg,GimbalRZdeg,MoviTlmBytes,toc(TimerOne),PitchDeg,HeadingDeg);
    
end
ElapsedTime = toc(TimerOne)
fclose(InsPort);
delete(InsPort);
clear InsPort
fclose(s);
delete(s);
clear s
fclose(LOGfid);
fclose(ANGLEfid);
fclose(INSDLfid);
fclose(ConScanFID);

%
% Reopen 'Angles.txt' and generate comparative plot of GPS and Gimbal data
DataFID = fopen(filenamestr2,'r');
if (DataFID == -1); return; end  % File could not be opened
HeaderLine = fgetl(DataFID);
AngleData = fscanf(DataFID,'%g %g %g %g %g %g %g %g %g %g %g',[10 inf])';
[RowMax, ColMax] = size(AngleData);

MaxTime = AngleData(RowMax,8);
figure
subplot(3,1,1)
plot([0 MaxTime], [0 0],'k'); 
hold on
plot(AngleData(:,8), AngleData(:,5),'r.');
plot(AngleData(:,8), AngleData(:,1),'b');
axis([0 MaxTime -10 10]);
title('GPS and Gimbal Stationary Performance');
ylabel('Roll Error (deg)');
subplot(3,1,2)
plot([0 MaxTime], [0 0],'k');
hold on
plot(AngleData(:,8), AngleData(:,5)-AngleData(:,9),'r.');
plot(AngleData(:,8), AngleData(:,2)-AngleData(:,9),'b');
%plot(AngleData(:,8), AngleData(:,3)-AngleData(:,9),'m');
axis([0 MaxTime -10 10]);
ylabel('Pitch Error (deg)');
subplot(3,1,3)
plot([0 MaxTime], [0 0],'k'); 
hold on
plot(AngleData(:,8), (AngleData(:,6)+YawDelta)-AngleData(:,10),'m.');
plot(AngleData(:,8), AngleData(:,3)-AngleData(:,10),'b');
axis([0 MaxTime -10 10]);
ylabel('Heading Error (deg)');
xlabel('Elapsed Time (sec)');
figure
plot(AngleData(:,8),AngleData(:,7));
hold on
title('Movi TLM process');
ylabel('Bytes Read');
Sieve = find(AngleData(:,6) == 999);
plot(AngleData(Sieve,8),AngleData(Sieve,6)/33,'r.');
ylabel('Parse Failures');
xlabel('Elapsed Time (sec)');
figure
plot(AngleData(:,8), AngleData(:,2)-AngleData(:,9),'b');
hold on
plot(AngleData(:,8), AngleData(:,3)-AngleData(:,10),'m');
plot([0 MaxTime], [0 0],'k');
plot(AngleData((100:100:RowMax),8),zeros(1,length(100:100:RowMax)),'k+');
axis([0 MaxTime -1 1]);
title('Detailed Pointing Offsets');
ylabel('Pointing Offset (deg)');
xlabel('Elapsed Time (sec)');
legend('Pitch','Yaw');

fclose(DataFID);

function RateCmd = PcontrolFnX(offset,OldOffset)
% Bang-bang controller with +/- 0.1 deg dead band; two mid-range gains.
% This version implements a "hard-over" control signal in the backlobe, and
% a "supergain" in mid-range to yank back movment in the wrong direction.
% Output is in degrees/sec; must be scaled to Movi native format 

%if abs(offset) > 139.5    % "back side" gain to avoid bad jerk
%    RateCmd = offset - sign(offset) * 180;
%end
if abs(offset) < 0.05 % Close-in tracking range with pseudo dead band
    RateCmd = 0.;
    %RateCmd = -10 * offset * abs(offset);
    %RateCmd = -40 * offset;
end
if (offset >= 0.05) && (offset < 2) % Mid-range offsets (positive)
    RateCmd = -3.;
    if (offset > OldOffset); RateCmd = 2*RateCmd; end
    %RateCmd = (1/(offset - 0.7))-40.5;
end

%if (offset >= 2) && (offset < 139.5) % Mid-range offsets (positive)
if (offset >= 2)  % Mid-range offsets (positive)
    RateCmd = -20.;
    %RateCmd = (1/(offset - 0.7))-40.5;
end
if (offset <= -0.05) && (offset > -2)  % Mid-range offsets (negative)
    RateCmd = 3.;
    if (offset < OldOffset); RateCmd = 2*RateCmd; end
    %RateCmd = (1/(0.7 + offset))+40.5;
end
%if (offset <= -2) && (offset > -139.5)  % Mid-range offsets (negative)
if (offset <= -2)  % Mid-range offsets (negative)
    RateCmd = 20.;
    %RateCmd = (1/(0.7 + offset))+40.5;
end
end

function RateCmd = PcontrolFnY(offset,OldOffset)
% Bang-bang controller with +/- 0.1 deg dead band; two mid-range gains.
% This version implements a "hard-over" control signal in the backlobe, and
% a "supergain" in mid-range to yank back movment in the wrong direction.
% Output is in degrees/sec; must be scaled to Movi native format 

%if abs(offset) > 139.5    % "back side" gain to avoid bad jerk
%    RateCmd = offset - sign(offset) * 180;
%end
if abs(offset) < 0.05 % Close-in tracking range with pseudo dead band
    RateCmd = 0.;
    %RateCmd = -10 * offset * abs(offset);
    %RateCmd = -40 * offset;
end
if (offset >= 0.05) && (offset < 2) % Mid-range offsets (positive)
    RateCmd = -2.8;
    if (offset > OldOffset); RateCmd = 2*RateCmd; end
    %RateCmd = (1/(offset - 0.7))-40.5;
end

%if (offset >= 2) && (offset < 139.5) % Mid-range offsets (positive)
if (offset >= 2)  % Mid-range offsets (positive)
    RateCmd = -10.;
    %RateCmd = (1/(offset - 0.7))-40.5;
end
if (offset <= -0.05) && (offset > -2)  % Mid-range offsets (negative)
    RateCmd = 2.8;
    if (offset < OldOffset); RateCmd = 2*RateCmd; end
    %RateCmd = (1/(0.7 + offset))+40.5;
end
%if (offset <= -2) && (offset > -139.5)  % Mid-range offsets (negative)
if (offset <= -2)  % Mid-range offsets (negative)
    RateCmd = 10.;
    %RateCmd = (1/(0.7 + offset))+40.5;
end
end

function RateCmd = PcontrolFnZ(offset,OldOffset)
% Bang-bang controller with +/- 0.1 deg dead band; two mid-range gains.
% This version implements a "hard-over" control signal in the backlobe, and
% a "supergain" in mid-range to yank back movment in the wrong direction.
% Output is in degrees/sec; must be scaled to Movi native format 

%if abs(offset) > 139.5    % "back side" gain to avoid bad jerk
%    RateCmd = offset - sign(offset) * 180;
%end
if abs(offset) < 0.05 % Close-in tracking range with pseudo dead band
    RateCmd = 0.;
    %RateCmd = -10 * offset * abs(offset);
    %RateCmd = -40 * offset;
end
if (offset >= 0.05) && (offset < 4) % Mid-range offsets (positive)
    RateCmd = -3.5;
    if (offset > OldOffset); RateCmd = 2*RateCmd; end
    %RateCmd = (1/(offset - 0.7))-40.5;
end

%if (offset >= 2) && (offset < 139.5) % Mid-range offsets (positive)
if (offset >= 4)  % Mid-range offsets (positive)
    RateCmd = -20.;
    %RateCmd = (1/(offset - 0.7))-40.5;
end
if (offset <= -0.05) && (offset > -4)  % Mid-range offsets (negative)
    RateCmd = 3.5;
    if (offset < OldOffset); RateCmd = 2*RateCmd; end
    %RateCmd = (1/(0.7 + offset))+40.5;
end
%if (offset <= -2) && (offset > -139.5)  % Mid-range offsets (negative)
if (offset <= -4)  % Mid-range offsets (negative)
    RateCmd = 20.;
    %RateCmd = (1/(0.7 + offset))+40.5;
end
end

function [ValuesOut] = ReadMovi(~, TimerOne)
% Read Movi COM port and fill global variables with data
global s BytesRead GimbalStatus1 GimbalStatus2 GimbalRX GimbalRY GimbalRZ
MoviTLM = fread(s,s.BytesAvailable,'uint8');
BytesRead = length(MoviTLM);
%
% Check validity of response w.r.t. 'QX' preamble and checksum
% (stub) **BUG.  Should verify good message received

%
% Parse TLM response
Batt_A = 10.0 + MoviTLM(12,1)*0.1; %volts
Batt_B = 10.0 + MoviTLM(13,1)*0.1; %volts
GimbalStatus1 = dec2base(MoviTLM(14,1),2,8);
GimbalStatus2 = dec2base(MoviTLM(15,1),2,8);
GimbalRX= bin2dec([dec2bin(MoviTLM(18,1)) dec2bin(MoviTLM(19,1))]);
if GimbalRX > 32767 % Gimbal angular states are signed (int16_t)
    GimbalRXdeg = -(GimbalRX-32768)*180/32767;
else
    GimbalRXdeg = GimbalRX*180/32767;
end
GimbalRY= bin2dec([dec2bin(MoviTLM(20,1)) dec2bin(MoviTLM(21,1))]);
if GimbalRY > 32767 % Gimbal states are signed (int16_t)
    GimbalRYdeg = -(GimbalRY-32768)*180/32767;
else
    GimbalRYdeg = GimbalRY*180/32767;
end
GimbalRZ= bin2dec([dec2bin(MoviTLM(22,1)) dec2bin(MoviTLM(23,1))]);
if GimbalRZ > 32767 % Gimbal states are signed (int16_t)
    GimbalRZdeg = -(GimbalRZ-32768)*180/32767;
else
    GimbalRZdeg = GimbalRZ*180/32767;
end
ElapsedTime = toc(TimerOne);
%fprintf(ANGLEfid,'%s',[dec2bin(MoviTLM(18,1)) dec2bin(MoviTLM(19,1))]);
%fprintf(ANGLEfid,'\t');
%fprintf(ANGLEfid,'%s',[dec2bin(MoviTLM(20,1)) dec2bin(MoviTLM(21,1))]);
%fprintf(ANGLEfid,'\t');
%fprintf(ANGLEfid,'%s',[dec2bin(MoviTLM(22,1)) dec2bin(MoviTLM(23,1))]);
%fprintf(ANGLEfid,'\n%f\t%f\t%f\n',GimbalRX,GimbalRY,GimbalRZ);
%fprintf(ANGLEfid,'%8.3f, %8.3f, %8.3f, %8.4f\n',GimbalRXdeg,GimbalRYdeg,GimbalRZdeg,ElapsedTime);
%fprintf(LOGfid,'%8.3f, %8.3f, %8.3f, %8.4f\n',GimbalRXdeg,GimbalRYdeg,GimbalRZdeg,ElapsedTime);
ValuesOut = [Batt_A; Batt_B; GimbalRXdeg; GimbalRYdeg; GimbalRZdeg];
end

function [ValuesOut] = ReadVectorNav
% Read COM port for VectorNav and get Heading, Pitch, Roll, Lat, Lon, Alt
global InsPort
INSdata   = fscanf(InsPort);
StrLength = length(INSdata);
ParamArray = strsplit(INSdata,',');
%PreambleTest = sum(cell2mat(ParamArray(1,1)) == '$VNINS');
% Preamble should be 6 char ('$VNINS') and length should be 142 characters
if ( (StrLength ~= 142) || (length(cell2mat(ParamArray(1,1))) ~= 6 ) )
    fprintf(1,'\n ***Bad data on GPS/INS port.  StrLength = %4i \n', StrLength)
    ValuesOut = [0; 0; 0; 0; 0; 0];
    return
end
% Attitude and position parameters are in cells 5 through 10
OwnHeading = str2double(cell2mat(ParamArray(1,5)));
OwnPitch = str2double(cell2mat(ParamArray(1,6)));
OwnRoll = str2double(cell2mat(ParamArray(1,7)));
OwnLat = str2double(cell2mat(ParamArray(1,8)));
OwnLon = str2double(cell2mat(ParamArray(1,9)));
OwnAlt = str2double(cell2mat(ParamArray(1,10)));
ValuesOut = [OwnHeading; OwnPitch; OwnRoll; OwnLat; OwnLon; OwnAlt];
end

function XmtMoviCmd(MoviPort,FrontMatter,AttributeByte1,AttributeByte2,Options,...
    GimbalCntl,GimbalRX,GimbalRY,GimbalRZ)
% Generates and transmits a command to the Movi
% Generate high and low bytes for Movi axis commands in command msg
global s LOGfid
MSBvalue = 128*(GimbalRZ < 0); % MSB == 128 if GimbalRZ < 0.
if MSBvalue>0
    GimbalRZhigh= uint8(MSBvalue + bitshift(int16(32768+GimbalRZ),-8));
    GimbalRZlow = uint8(mod((32768+GimbalRZ),256));
else
    GimbalRZhigh= uint8(bitshift(int16(GimbalRZ),-8));
    GimbalRZlow = uint8(mod(GimbalRZ,256));
end
MSBvalue = 128*(GimbalRY < 0); % MSB == 128 if GimbalRY < 0.
if MSBvalue>0
    GimbalRYhigh= uint8(MSBvalue + bitshift(int16(32768+GimbalRY),-8));
    GimbalRYlow = uint8(mod((32768+GimbalRY),256));
else
    GimbalRYhigh= uint8(bitshift(int16(GimbalRY),-8));
    GimbalRYlow = uint8(mod(GimbalRY,256));
end
MSBvalue = 128*(GimbalRX < 0); % MSB == 128 if GimbalRX < 0.
if MSBvalue>0
    GimbalRXhigh= uint8(MSBvalue + bitshift(int16(32768+GimbalRX),-8));
    GimbalRXlow = uint8(mod((32768+GimbalRX),256));
else
    GimbalRXhigh= uint8(bitshift(int16(GimbalRX),-8));
    GimbalRXlow = uint8(mod(GimbalRX,256));
end
Checksum = uint8(255 - mod((uint16(AttributeByte1) + uint16(AttributeByte2) + ...
        uint16(Options) + uint16(GimbalCntl) + uint16(GimbalRXlow) + ...
        uint16(GimbalRXhigh) + uint16(GimbalRYlow) + ...
        uint16(GimbalRYhigh) + uint16(GimbalRZlow) + uint16(GimbalRZhigh)),256));
CmdStr = [FrontMatter GimbalCntl GimbalRXhigh GimbalRXlow GimbalRYhigh ...
    GimbalRYlow GimbalRZhigh GimbalRZlow uint8(0) uint8(0) ...
    uint8(0) uint8(0) uint8(0) uint8(0) uint8(0) uint8(0) uint8(0)...
    uint8(0) uint8(0) Checksum];
fprintf(s, CmdStr);
fprintf(LOGfid,'Six bytes for RX, RY, RZ, in order, are: %i %i %i %i %i %i ',GimbalRXhigh, GimbalRXlow, GimbalRYhigh, ...
    GimbalRYlow, GimbalRZhigh, GimbalRZlow);
fprintf(LOGfid,'\n');
end

function myKeyPressFcn(hObject, event)
% Cature keyboard single-stroke command and return it to the main routine.
global KEY_IS_PRESSED KeyCmd TargetID
KEY_IS_PRESSED  = 1;
KeyCmd = event.Character;
switch event.Character
    case 'h'
        disp('List of single-character keyboard commands:')
        disp('x = close all files, plot results, and terminate script')
        disp('p = toggle Movi power on (1 sec intermittent servo command)')
        disp('o = toggle Movi power off (3 sec intermittent servo command)')
        disp('q = quiet mode (no torque until Motor Active command received')
        disp('t = motor active -- torque applied to all motors as needed')
        disp('k = temporarily kill torque on Movi to stop oscillation')
        disp('s = perform RSSI scan and report results')
        disp('a = apply results of last RSSI scan')
        disp('c = continue without applying results of last RSSI scan')
        disp('u = perform cable unwrap operation')
        disp('1 = select Tiara WPB as new geopointing target')
        disp('2 = select Freeport as new geopointing target')
        disp('3 = select Mt. Hood as new geopointing target.')
        KeyCmd = '';
        KEY_IS_PRESSED  = 0;
    case 'x'
    case 'p'
        % STUB -- issue servo command for one second
        KeyCmd = '';
        KEY_IS_PRESSED  = 0;
    case 'o'
        % STUB -- issue servo command for three seconds
        KeyCmd = '';
        KEY_IS_PRESSED  = 0;
    case 'q'
    case 't'
    case 'k'
    case 's'
    case 'a'
    case 'c'
    case 'u'
    case '1'
        TargetID = 1;
    case '2'
        TargetID = 2;
    case '3'
        TargetID = 3;
    otherwise
        disp('Unrecognized keyboard command. Type "h" for help')
        KeyCmd = '';
        KEY_IS_PRESSED  = 0;
        
end
end

function OutputVoltage = ReadRSSI(RslSimState,HeadingOffset,PitchOffset)
% Can read RSSI from Bridgewave, or simulate based on offpointing values
global BeamWidth
global RslSimHeadingOffset RslSimPitchOffset RslSimLatency RslBuffer
global RslBufferPointer % Pointer into circular buffer of RSSI values
if RslSimState % If RslSimState = 1, simulate RSL value w.r.t. peak of 4v
    % Note: 4v = -20 dBm; 1v = -80 dBm. Thus, on boresight, output = 4v. 
    % For each 20 dB loss, subtract 1v down to a floor of 1v
    FullOffset = sqrt((HeadingOffset-RslSimHeadingOffset)^2 + ...
        (PitchOffset-RslSimPitchOffset)^2); %degrees
    NormalizedOffset = FullOffset/BeamWidth; % in Beamwidths
    if NormalizedOffset == 0
        OffpointLossRatio = 1;
    else
        OffpointLossRatio = (sin(pi*NormalizedOffset)/(pi*NormalizedOffset))^2;
    end
    OffpointLossDB = max(-60, 10*log10(OffpointLossRatio)); % negative val.
    % Add noise based on uniform distribution over +/- 2 dB
    OffpointLossDB = OffpointLossDB + 4*rand - 2;
    RslBuffer(RslBufferPointer) = 4 + OffpointLossDB/20; % Voltage scaling is 20 dB/volt
    if RslBufferPointer == (RslSimLatency+1)
        RslBufferPointer = 1;
        OutputVoltage = RslBuffer(1);
    else
        RslBufferPointer = RslBufferPointer + 1;
        OutputVoltage = RslBuffer(RslBufferPointer);
    end
else % Read an actual RSSI value from the Bridgewave
    %STUB
end

end



function coeffs = fit2dPolySVD( x, y, z, order )
% Fit a polynomial f(x,y) so that it provides a best fit to the data z.
% Version 1.4.0.0 (July 2011) created by Richard Whitehead. 
%
% Uses SVD which is robust even if the data is degenerate.  Will always
% produce a least-squares best fit to the data even if the data is
% overspecified or underspecified.
% x, y, z are column vectors specifying the points to be fitted.
% The three vectors must be the same length.
% Order is the order of the polynomial to fit.
% Coeffs returns the coefficients of the polynomial.  These are in
% increasing power of y for each increasing power of x, e.g. for order 2:
% zbar = coeffs(1) + coeffs(2).*y + coeffs(3).*y^2 + coeffs(4).*x +
% coeffs(5).*x.*y + coeffs(6).*x^2
% Use eval2dPoly (available on MATLAB Exchange) to evaluate the polynomial.

[sizexR, sizexC] = size(x);
[sizeyR, sizeyC] = size(y);
[sizezR, sizezC] = size(z);

if (sizexC ~= 1) || (sizeyC ~= 1) || (sizezC ~= 1)
    fprintf( 'Inputs of fit2dPolySVD must be column vectors' );
    return;
end

if (sizeyR ~= sizexR) || (sizezR ~= sizexR)
    fprintf( 'Inputs vectors of fit2dPolySVD must be the same length' );
    return;
end


numVals = sizexR;

% scale to prevent precision problems
scalex = 1.0/max(abs(x));
scaley = 1.0/max(abs(y));
scalez = 1.0/max(abs(z));
xs = x .* scalex;
ys = y .* scaley;
zs = z .* scalez;


% number of combinations of coefficients in resulting polynomial
numCoeffs = (order+2)*(order+1)/2;

% Form array to process with SVD
A = zeros(numVals, numCoeffs);

column = 1;
for xpower = 0:order
    for ypower = 0:(order-xpower)
        A(:,column) = xs.^xpower .* ys.^ypower;
        column = column + 1;
    end
end

% Perform SVD
[u, s, v] = svd(A);

% pseudo-inverse of diagonal matrix s
sigma = eps^(1/order); % minimum value considered non-zero
qqs = diag(s);
qqs(abs(qqs)>=sigma) = 1./qqs(abs(qqs)>=sigma);
qqs(abs(qqs)<sigma)=0;
qqs = diag(qqs);
if numVals > numCoeffs
    qqs(numVals,1)=0; % add empty rows
end

% calculate solution
coeffs = v*qqs'*u'*zs; 


% scale the coefficients so they are correct for the unscaled data
column = 1;
for xpower = 0:order
    for ypower = 0:(order-xpower)
        coeffs(column) = coeffs(column) * scalex^xpower * scaley^ypower / scalez;
        column = column + 1;
    end
end

end

function zbar = eval2dPoly( x, y, coeffs )
% Given the coefficients of a polynomial as returned by fit2dPolySVD,
% calculates the values z for input values (x,y).
% Version 1.4.0.0 (July 2011) created by Richard Whitehead. 
%
% x, y are column vectors specifying the points to be calculated.
% The vectors must be the same length.
% Coeffs is the coefficients array returned by fit2dPolySVD.


[sizexR, sizexC] = size(x);
[sizeyR, sizeyC] = size(y);

if (sizexC ~= 1) || (sizeyC ~= 1)
    fprintf( 'Inputs of eval2dPoly must be column vectors' );
    return;
end

if (sizeyR ~= sizexR)
    fprintf( 'Inputs vectors of eval2dPoly must be the same length' );
    return;
end


numVals = sizexR;

order = 0.5 * (sqrt(8*length(coeffs)+1) - 3);

zbar = zeros(numVals,1);
column = 1;
for xpower = 0:order
    for ypower = 0:(order-xpower)
        zbar = zbar + (coeffs(column) .* x.^xpower .* y.^ypower);
        column = column + 1;
    end
end

end
