% Script for parsing and analyzing roll/pitch/yaw from Movi telemetry log
%
% Assume grep has been used to extract all log file lines with Movi TLM,
% and that the results have been imported as 2 columns of data
%
close all
[RowMax, ~] = size(AttitudeMay27);
Telemetry = zeros(RowMax, 2); % Columns will represent pitch, yaw
for row = 2:RowMax
    Telemetry(row, 1) = AttitudeMay27.VarName4(row); %  pitch
    Telemetry(row, 2) = AttitudeMay27.VarName5(row); %  yaw
end
Over180 = logical(Telemetry > 180);
Telemetry(Over180) = Telemetry(Over180) - 360;
std(Telemetry(30000:90000,1))
std(Telemetry(30000:90000,2))
%
% Adjust row range as needed to capture valid data (100*seconds)
figure
plot((1:RowMax)/100, -Telemetry(:,1));
hold on
plot((1:RowMax)/100, Telemetry(:,2));
axis([0 RowMax/100 -1 1]);
title('Pitch and Yaw errors');
ylabel('Reported error (deg)');
xlabel('Elapsed Time (sec)');
legend('Pitch', 'Yaw');
