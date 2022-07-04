function data = GetFourier(s,H,L,n)

y=0;
l=[s(1) s(1)+s(2) s(1)+s(2)+s(3) s(1)+s(2)+s(3)+s(4) s(1)+s(2)+s(3)+s(4)+s(5)];
taw = l(5);
w=2*pi/taw;
t= 0:(taw/n):taw;

for n=1:3
    nw=n*w;
    Bn=0;
    Bn= Bn + (   -cos(nw*s(1))*s(1)/nw +sin(nw*s(1))/nw^2    )*H(1)/s(1);

    Bn= Bn + (-(l(2)*cos(nw*l(2)))/nw  +  s(1)*cos(nw*s(1))/nw  +  sin(nw*l(2))/nw^2-sin(nw*s(1))/nw^2)*(L-H(1))/s(2);
    Bn= Bn - (H(1)-s(1)*(L-H(1))/s(2))*(cos(nw*l(2))-cos(nw*s(1)))/nw;

    Bn= Bn + (-cos(nw*l(3))/nw + cos(nw*l(2))/nw)*L;

    Bn= Bn + (-l(4)*cos(nw*l(4))/nw+l(3)*cos(nw*l(3))/nw+sin(nw*l(4))/nw^2-sin(nw*l(3))/nw^2)*(H(2)-L)/s(4);
    Bn= Bn + ((H(2)-L)*(-l(3))/s(4)+L)*(cos(nw*l(3))-cos(nw*l(4)))/nw;

    Bn= Bn + (l(5)*cos(nw*l(5))/nw-l(4)*cos(nw*l(4))/nw-sin(nw*l(5))/nw^2+sin(nw*l(4))/nw^2)*H(2)/s(5);
    Bn= Bn + (H(2)*l(5)/s(5))*(-cos(nw*l(5))+cos(nw*l(4)))/nw;

    Bn=Bn*2/taw;

    y = y + Bn*sin(nw*t);

    An=0;

    An= An + ( sin(nw*s(1))*s(1)/nw +(cos(nw*s(1))-1)/nw^2 )*H(1)/s(1);

    An= An + ((l(2)*sin(nw*l(2)))/nw  -  s(1)*sin(nw*s(1))/nw  +  cos(nw*l(2))/nw^2-cos(nw*s(1))/nw^2)*(L-H(1))/s(2);
    An= An + (H(1)-s(1)*(L-H(1))/s(2))*(sin(nw*l(2))-sin(nw*s(1)))/nw;

    An= An + (sin(nw*l(3)) - sin(nw*l(2)))*L/nw;

    An= An + (l(4)*sin(nw*l(4))/nw-l(3)*sin(nw*l(3))/nw   +  cos(nw*l(4))/nw^2-cos(nw*l(3))/nw^2)*(H(2)-L)/s(4);
    An= An + ((H(2)-L)*(-l(3))/s(4)+L)*(sin(nw*l(4))-sin(nw*l(3)))/nw;

    An= An + (l(5)*sin(nw*l(5))/nw-l(4)*sin(nw*l(4))/nw  +  cos(nw*l(5))/nw^2-cos(nw*l(4))/nw^2)*(-H(2))/s(5);
    An= An + (H(2)*l(5)/s(5))*(sin(nw*l(5))-sin(nw*l(4)))/nw;

    An=An*2/taw;

    y = y + An*cos(nw*t);
end

y= y + (s(1)*H(1)+(L+H(1))*s(2)+2*s(3)*L+(L+H(2))*s(4)+s(5)*H(2))/(2*taw);

%plot(t,y);
data=y;
end