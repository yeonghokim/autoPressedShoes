clear all
clc 

disp("자동압박제어 시뮬레이션");

Datanum = 490;
data = ReadIMUFunction("imu_test1.txt",Datanum);
% data{1} = Gyro data{2} = Linear data{3} = etc
i=1;
Gyro = data{1};
Linear = data{2};
etc = data{3};

diff = 0;
oldnum = 0;
oldtime= etc(i,1)-20;

SwingNum = -1;
SwingCheck = 0;

IsSwing = -1;
IsSwingData =0;
while(1)
    time=(etc(i,1)-oldtime);
    if(SwingCheck>0)
        SwingCheck=SwingCheck-time;
    end
    pause(time*0.001);

    %미분값 생성
    diff(i) =  (Gyro(i,2)-oldnum)/ time;

    if(diff(i)>0.06 && SwingCheck<=0)
        IsSwing=IsSwing*(-1);
        SwingCheck=200;
    end

    disp("IsSwing == "+IsSwing);
    IsSwingData(i)=IsSwing*2;
    
    %figure(5),plot((etc(1:i,1)-etc(1,1)),[diff',IsSwingData'] );
    figure(5),plot((etc(1:i,1)-etc(1,1)),[Gyro(1:i,2),IsSwingData'] );

    oldnum=Gyro(i,2);
    oldtime= etc(i,1);
    i=i+1;
    if(i>Datanum)
        break;
    end
end

