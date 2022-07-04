function data = ReadDataFunction(text)
fileID = fopen(text,'r');
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

data = {A, B, C};


end


