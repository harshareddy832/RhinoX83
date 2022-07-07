#Joint angles for a Rhino X83 Robot
w=input('Enter the position x y z:')
pitch=input('Enter value for pitch:')
roll=input('Enter value for roll:')
yaw=input('Enter value for yaw:')
x=w(1,1);
y=w(1,2);
z=w(1,3);

%Joint angle
d5=0.16;
d1=0.26;
a2=0.2286;
a3=0.2286;
a4=0.009;
dcm = zeros(3,3);
dcm(1,1) = cosd(pitch)*cosd(yaw);
dcm(1,2) = cosd(pitch)*sind(yaw);
dcm(1,3) = -sind(pitch);
dcm(2,1) = sind(roll)*sind(pitch)*cosd(yaw) - cosd(roll)*sind(yaw);
dcm(2,2) = sind(roll)*sind(pitch)*sind(yaw) + cosd(roll)*cosd(yaw);
dcm(2,3) = sind(roll)*cosd(pitch);
dcm(3,1) = cosd(roll)*sind(pitch)*cosd(yaw) + sind(roll)*sind(yaw);
dcm(3,2) = cosd(roll)*sind(pitch)*sind(yaw) - sind(roll)*cosd(yaw);
dcm(3,3) = cosd(roll)*cosd(pitch);
R11=dcm(1,1);R12=dcm(1,2);R13=dcm(1,3);
R21=dcm(2,1);R22=dcm(2,2);R23=dcm(2,3);
R33=dcm(3,3);

teta1=atan2d(y,x);                                          %base joint
s1=sind(teta1);
c1=cosd(teta1);

teta3=acosd((((a2^2+2*a2*a3+a3^2)^2)-(a2^2)-(a3^2))/(2*a2*a3)); 
s3=sind(teta3); 
c3=cosd(teta3);  

teta234 = atan2d(-(c1*z+s1*R13),-R33);                      %elbow joint
s234=sind(teta234);
c234=cosd(teta234);
b1=c1*2+s1*x-a4*teta234+d5*s234;
b2=d1-R13*s234-d5*c234-z;
teta2=atan2d((a2+a3*c3)*b2-a3*s3*b1,(a2+a3*c3)*b1+a3*s3*b2);%shoulder joint
teta4=teta234-teta2-teta3;                                  %tool-pitch
teta5=atan2d(s1*R11-c1*R21,s1*R12-c1*R22)                   %Tool roll joint
 
q=[teta1 teta2 teta3 teta4 teta5]                           %join angles
