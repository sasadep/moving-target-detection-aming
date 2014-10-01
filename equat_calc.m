%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sandeep Aswath Narayana              %
% UID: 302-00-0104                     %
% Department of Electrical engineering %
% Kate Gleason College of engineering  %
% Rochester Institute of technology    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% project:Moving Target Detection and aiming %
%         (machine learning implementation)  % 
%Authors:                                    %
%Sandeep Aswath Narayana    (sa5641@rit.edu) %
%Chris A. Tatsch            (cxa9493@rit.edu)%
% Modified: 12/14/2013                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% intial coordinate(pixel) values of x,y when it detect the target 
x1=[669 896 189 161 568 425 669 862 347];
y1=[374 444 286 384 562 277 385 585 367];


% final coordinate (pixel) values of x,y when it aims the target
x2=[586 597 586 589 589 590 589 589 590];
y2=[462 484 464 467 467 469 476 476 469];

%%
%training data 

% intial servo values of  when it detect the target
s11=[1555 1380 1380 1380 1081 1380 1380 1255 1430];
s13=[1325 1400 1400 1400 1250 1400 1400 1235 1330];

% final servo values of  when it aims the target
s21=[1610 1550 1081 1071 1071 1255 1430 1430 1255];
s23=[1275 1350 1250 1325 1325 1235 1330 1330 1235];
%%

% distance moved in x co-ordinates  
Xdis=x2-x1;
% distance moved in x co-ordinates
Ydis=y2-y1;

%converting angles to servo values
 y111=(s11 -1380)*0.1122;  
 y113=(s13-1390)/10.346;
y121=(s21 -1380)*0.1122;
 y123=(s23-1390)/10.346;
% calculating change in angles   
   theta1=y121-y111; %change in angles for servo1  
   theta3=y123-y113;   % change in angle for Servo2
   
   
   
%%
% prediction using data 
% plotting the regression for the change in angles with the distance 
    
   figure(1);
  title('theta1,X')
 plotregression(Xdis,theta1);
 
     figure(2)
     title('theta1,Y')
 plotregression(Ydis,theta1);

   figure(5);
  title('theta3,X')
 plotregression(Xdis,theta1);
     figure(6)
     title('theta3,Y')
 plotregression(Ydis,theta1);


 
 % equations for the relation for change in angles for the change in servo
 % values

%  s1x=(-0.077.*Xdis)-1.4
% s1y=(-0.12.*Ydis)+1.7
% %s2x=((-0.0057).*Xdis)+4.3
% %s2y=(0.062.*Ydis)+1
% S3x=(-0.077.*Xdis)-1.4
% s3y=(-0.12.*Ydis)+1.7
