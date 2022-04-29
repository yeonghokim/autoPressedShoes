fileID = fopen('mester_answer_Ex1.txt','r');
A=zeros(5);
B=1;
C=1;
for i = 1:115
    formatSpec = ' Sensor : %f motor %s pressure : %f %f %f';
    temp = fscanf(fileID,formatSpec);
    A(i,1)=temp(1);
    if(temp(2)==76)
        A(i,2)='L';
        A(i,3)=temp(5);
        A(i,4)=temp(6);
        A(i,5)=temp(7);
    else
        A(i,2)='H';
        A(i,3)=temp(6);
        A(i,4)=temp(7);
        A(i,5)=temp(8);
    end
    formatSpec = '%f';
    B(i) = fscanf(fileID,formatSpec);
    formatSpec = 'target_pressure=%f;';
    C(i) = fscanf(fileID,formatSpec);
end

for i = 116:2348
    formatSpec = ' Sensor : %f motor %s pressure : %f %f %f';
    temp = fscanf(fileID,formatSpec);
    A(i,1)=temp(1);
    if(temp(2)==76)
        A(i,2)='L';
        A(i,3)=temp(5);
        A(i,4)=temp(6);
        A(i,5)=temp(7);
    else
        A(i,2)='H';
        A(i,3)=temp(6);
        A(i,4)=temp(7);
        A(i,5)=temp(8);
    end
    formatSpec = '%f';
    B(i) = fscanf(fileID,formatSpec);
end
% A(1) 압력센서 값
% A(2) motor LOW(76) HIGH(72)
% A(3) target pressure
% A(4) current pressure
% A(5) motor current degree
% B motor speed
% C target pressure

figure(5),plot(A);


