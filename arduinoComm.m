function arduinoReceived = arduinoComm(arduino, tank, desWL, desM1, desM2, desM3, desM4, desM5, desM6, desM7, desM8, desM9)
% to be used in musselHeater (app designer)
% updated 04jun2019 by WD
%   1. added missing, leading zeros if for some reason setpoint below 10
%   2. confirmed that get correct return string from Arduino
% take in values to be sent to Arduino from desired data (imported file)
% arduino returns string - should we parse it here?
    T = tank;
    M1 = sprintf('%0.2f', desM1);
    if size(M1,2)<5
        addzero=5-size(M1,2);
        M1=strcat(mat2str(zeros(1,addzero)),M1);
    end
    M2 = sprintf('%0.2f', desM2);
    if size(M2,2)<5
        addzero=5-size(M2,2);
        M2=strcat(mat2str(zeros(1,addzero)),M2);
    end
    M3 = sprintf('%0.2f', desM3);
    if size(M3,2)<5
        addzero=5-size(M3,2);
        M3=strcat(mat2str(zeros(1,addzero)),M3);
    end
    M4 = sprintf('%0.2f', desM4);
    if size(M4,2)<5
        addzero=5-size(M4,2);
        M4=strcat(mat2str(zeros(1,addzero)),M4);
    end
    M5 = sprintf('%0.2f', desM5);
    if size(M5,2)<5
        addzero=5-size(M5,2);
        M5=strcat(mat2str(zeros(1,addzero)),M5);
    end
    M6 = sprintf('%0.2f', desM6);
    if size(M6,2)<5
        addzero=5-size(M6,2);
        M6=strcat(mat2str(zeros(1,addzero)),M6);
    end
    M7 = sprintf('%0.2f', desM7);
    if size(M7,2)<5
        addzero=5-size(M7,2);
        M7=strcat(mat2str(zeros(1,addzero)),M7);
    end
    M8 = sprintf('%0.2f', desM8);
    if size(M8,2)<5
        addzero=5-size(M8,2);
        M8=strcat(mat2str(zeros(1,addzero)),M8);
    end
    M9 = sprintf('%0.2f', desM9);
    if size(M9,2)<5
        addzero=5-size(M9,2);
        M9=strcat(mat2str(zeros(1,addzero)),M9);
    end
    WL = sprintf('%0.2f', desWL);
    if size(WL,2)<4
        addzero=5-size(WL,2);
        WL=strcat(mat2str(zeros(1,addzero)),WL);
    end
    %output is new, it used to be fprintf(arduino, [T M1 M2 M3 M4 M5 M6 M7
    %M8 M9 WL]); only (edit 5.31.19)
    output = strcat(T, M1, M2, M3, M4, M5, M6, M7, M8, M9, WL);
    fprintf(arduino, output);
    
    
    arduinoReceived = fscanf(arduino);
end