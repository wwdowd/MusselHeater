function [arduino, flag] = setupSerial(comPort)
% Initialize the serial port communication between Arduino and MATLAB.
% We ensure that the arduino is also communicating with MATLAB at this
% time. A predefined code on the arduino acknowledges this.
% If setup is complete then the value of the setup is returned as 1, else
% 0.
    
    flag = 1;
    clear arduino a;
    %added this code to remove comPort if already exists WD 04jun2019
    ardname=strcat('Serial-',comPort);
    props={'Name'};vals={ardname};
    obj=instrfindall(props,vals);
    delete(obj);
    
    %added this code to remove comPort if already exists WD 04jun2019
    
    arduino = serial(comPort,'DataBits',8,'StopBits',...
        1,'BaudRate',115200,'Parity','none','TimeOut',20)
    
    %set(arduino, 'DataBits', 8);
    %set(arduino, 'StopBits', 1);
    %set(arduino, 'BaudRate', 115200);
    %set(arduino, 'Parity', 'none');
    %RT added Terminator designation 7.31.18
    %set(arduino, 'Terminator', 'LF');
    
    fopen(arduino);
    pause(5)
    %wait for Arduino to wake up 5s
%     T = timer('TimerFcn',@(~,~)disp('Communicating with Arduino...'),'StartDelay',5);
%     start(T)
    fprintf(arduino, '%s', 'a');  %changed from %c, %s includes NL at end
    a=fread(arduino,4,'char');
    %disp('The value returned from Arduino is %',a)
    if a(2) == 97
        disp('Arduino returned an "a"... Woohoo')
        %little a to Arduino triggers void loop
        %%%%%fprintf(arduino, '%c', 'a');
    else 
     while (a(2)~=97) %(a(1) ~= 'a') %(a ~= 97)
         fprintf(arduino, '%s', 'a');
         a = fread(arduino,4,'char');%fread(arduino, 3, 'uchar'); %changed from 3 to 1 %changed from uchar to char %fscanf(arduino,'%c');
     end
     %%%%%fprintf(arduino, '%c', 'a');
    end
    
    %mbox = msgbox('Serial Communication Connection Successful.');
    %uiwait(mbox);
    %%%%%%WD commented out next line 04jun2019
    %fscanf(arduino,'%u'); %try '%c'? - it still says %u is the wrong type
end


