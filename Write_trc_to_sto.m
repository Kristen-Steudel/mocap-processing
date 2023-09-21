% Kristen Steudel September 13, 2023
% Write trc to sto
addpath("common\")
addpath("S001\MarkerData")
[header, data, dataArray] = TRCload('rotated_Sprint_04500001.trc');
writeSTO(data,header,"C:\Users\15086\Documents\GitHub\mocap-processing\S001\MarkerData",'rotated_Sprint_04500001');
disp('STO file saved properly')