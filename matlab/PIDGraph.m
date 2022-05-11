result = 0;                                 
target_degree=200;
mMatrix=zeros(2);
current_degree=0;

mP=2;
mI=0.6;
mD=0.01;
mTime=0.01;

pControl=0;
iControl=0;
dControl=0;
pidControl=0;

realError=0;
errorGap=0;
accError=0;
i=0;
while(1)
    i=i+1;

    mMatrix(i,1) = target_degree;
    mMatrix(i,2) = current_degree;
    if(i>100)
        plot(mMatrix(i-100:i,:));
    else
        plot(mMatrix);
    end

    ylim([0 400]);
    drawnow;

    if(mod(i/20,1)==0)
        m = i/20;
        m =mod(m,5)+1;
        target_degree=m*50;
    end
    errorGap = target_degree - current_degree - realError; 
    realError = target_degree - current_degree;
    accError = accError + realError;
    pControl = mP * realError;
    iControl = mI * (accError * mTime);
    dControl = mD * (errorGap / mTime);

    pidControl = pControl + iControl + dControl;

    current_degree= current_degree + pidControl*0.2;
    
    pause(mTime)
end    