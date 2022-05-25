fileID = fopen('imu_test1.txt','r');
n=490;%n=289;
Gyro=zeros(n,3);
Linear=zeros(n,3);
etc=zeros(n,5);
for i= 1:n
    formatSpec = 'Gyro:	x= %f |	y= %f |	z= %f\n';
    temp = fscanf(fileID,formatSpec);
    Gyro(i,1)=temp(1);
    Gyro(i,2)=temp(2);
    Gyro(i,3)=temp(3);
    formatSpec = 'Linear:	x= %f |	y= %f |	z= %f\n';
    temp = fscanf(fileID,formatSpec);
    Linear(i,1)=temp(1);
    Linear(i,2)=temp(2);
    Linear(i,3)=temp(3);
    formatSpec = 'time : %f Calibration: Sys=%f Gyro=%f Accel=%f Mag=%f\n';
    temp = fscanf(fileID,formatSpec);
    etc(i,1)=temp(1);
    etc(i,2)=temp(2);
    etc(i,3)=temp(3);
    etc(i,4)=temp(4);
    etc(i,5)=temp(5);
end

etc(:,1)=etc(:,1)-etc(1,1);
figure(5),plot(etc(:,1),[Linear(:,1),Linear(:,3),Gyro(:,2)] );
title('x축 선형가속도(파란색) + z축 선형가속도(빨간색) + y축 각속도(노란색)');


