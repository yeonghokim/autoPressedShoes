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
st=[-1,-1,-1,-1,-1,-1];
s=[-1,-1,-1,-1,-1];
H=[1,1];
L=-2;

FourierData = zeros(1,1);
istart=0;

averL=-3;
CheckingS = 0;
while(1)
    time=(etc(i,1)-oldtime);
    if(SwingCheck>0)
        SwingCheck=SwingCheck-time;
    end
    pause(time*0.001);

    %미분값 생성
    diff(i) =  (Gyro(i,2)-oldnum)/ time;

    switch(CheckingS)
        case 0
            if(diff(i)>0 && Gyro(i,2)>0.5)
                istart=i;
                st(1) = etc(i,1);
                CheckingS=1;
            end
        case 1
            if(Gyro(i,2)>H(1))
                st(2) = etc(i,1);
                H(1) = Gyro(i,2);
            end
            if(Gyro(i,2)<0)
                s(1)=st(2)-st(1);
                CheckingS=2;
            end
        case 2
            if(diff(i)>0 || Gyro(i,2)<averL)
                st(3) = etc(i,1);
                s(2)=st(3)-st(2);
                CheckingS=3;
            end
            if(Gyro(i,2)<L)
                 L=Gyro(i,2);
            end
        case 3
            if(diff(i)>0 || Gyro(i,2)>averL)
                st(4) = etc(i,1);
                s(3)=st(4)-st(3);
                CheckingS=4;
            end
            if(Gyro(i,2)<L)
                 L=Gyro(i,2);
            end
        case 4
            if(Gyro(i,2)>H(2))
                st(5) = etc(i,1);
                H(2) = Gyro(i,2);
            end
            if(Gyro(i,2)<0.5 && st(5)~=-1)
                s(4)=st(5)-st(4);
                CheckingS=5;
            end

        case 5
            if(Gyro(i,2)<0.5 && diff(i)<0.5)
                s(5)=etc(i,1)-st(5);
                
                FourierData(istart:i) = GetFourier(s,H,L,i-istart);

                st=[-1,-1,-1,-1,-1,-1];
                s=[-1,-1,-1,-1,-1];
                H=[1,1];
                L=-2;

                CheckingS=0;
            end
    end

    if(diff(i)>0.06 && SwingCheck<=0)
        IsSwing=IsSwing*(-1);
        SwingCheck=200;
    end

    disp("IsSwing == "+IsSwing);
    IsSwingData(i)=IsSwing*2;
    
    %figure(5),plot((etc(1:i,1)-etc(1,1)),[diff',IsSwingData'] );
    ttmp =size(FourierData,2);
    figure(5),plot((etc(1:i,1)-etc(1,1)),[Gyro(1:i,2),IsSwingData',[FourierData zeros(1,i-ttmp)]'] );

    oldnum=Gyro(i,2);
    oldtime= etc(i,1);
    i=i+1;
    if(i>Datanum)
        break;
    end
end