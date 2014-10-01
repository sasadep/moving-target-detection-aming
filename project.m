%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sandeep Aswath Narayana              %
% UID: 302-00-0104                     %
% Department of Electrical engineering %
% Kate Gleason College of engineering  %
% Rochester Institute of technology    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% project:Moving Target Detection and aiming %
%Authors:                                    %
%Sandeep Aswath Narayana    (sa5641@rit.edu) %
%Chris A. Tatsch            (cxa9493@rit.edu)%
% Modified: 12/14/2013 (rmb3518)             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% robot = ArmRobot('COM1');
% 
% 
% % Robot servo values for the robots on the lab.
% robotn4centers = [1380 1520 1380 1480 1460 1500 500];
% % robotn1centers = [1410 1505 1520 1440 1480 1440 1580];
% % robotn2centers = [1500 1520 1380 1510 1520 1440 1500];
% % robotn1mins = [500 645 500 500 500 500 1193];
% % robotn1maxs = [2410 2296 1900 2461 2460 2500 2309];
% robotn4mins = [500 640 500 500 500 500 500];
% robotn4maxs = [2390 2260 1980 2460 2500 2500 2309];
% % robotn2mins     = [643 759 580 647 608 540 900];
% % robotn2maxs     = [2500 2211 2210 2380 2420 2337 2191];
% 
% 
% robot.setServoCenters(robotn4centers);  %set the center position
% robot.setServoBounds(robotn4mins,robotn4maxs);  %set the servo bounds
% robot.setLinkLengths([5 5 2 12.3 6.9 4.5 7.2 5.3]); %set the lenght of the links
% % Connect to the Robot
% robot.connect();
% 
% display('robot connected');
% robot.centerJoints;
% 
% 
% % Capture the video frames using the videoinput function
% %RGB 980x720 used
vid = videoinput('winvideo', 1, 'RGB24_640x480');

% Set the properties of the video object
set(vid, 'FramesPerTrigger', Inf);
set(vid, 'ReturnedColorspace', 'rgb')
vid.FrameGrabInterval = 7;%interval between frames

%start the video aquisition here
start(vid)

% Set a loop that stop after 200 frames of aquisition 
while(vid.FramesAcquired<=200)
    
    % Get the snapshot of the current frame
    data = getsnapshot(vid);
    

    % Subtract the red component 
    % from the grayscale image to extract the red components in the image.
    diff_im = imsubtract(data(:,:,1), rgb2gray(data));
      
    %Use a median filter to filter out noise
    diff_im = medfilt2(diff_im, [3 3]);
  
    % Convert the resulting grayscale image into a binary image.
    diff_im = im2bw(diff_im,0.18);
    
    % Remove all those pixels less than 300px
    diff_im = bwareaopen(diff_im,300);
    
    % Label all the connected components in the image.
    bw = bwlabel(diff_im, 8);
    
    
    % Calculating Centroid and Bounding box and store it on a struct
    stats = regionprops(bw, 'BoundingBox', 'Centroid');
    
    % Display the image
    imshow(data)
   
    
    %test if an object is detected
    a=sum(bw(:));  

    
    if(a>=1)%if object is detected  
  
    XCentroid=stats(1,1).Centroid(1,1)      %get the position x of the centroid of the object
    YCentroid=stats(1,1).Centroid(1,2)      %get the position y of the centroid of the object
   
    x=483;    % x coordinates of the laser for a frame (where it is pointing)  
    y=361;    % y coordinates of the laser for a frame (where it is pointing)  
    Xdis=(XCentroid-x);     %diference between the position of the object and where the laser points x axis
    Ydis=(YCentroid-y);     %diference between the position of the object and where the laser points x axis
   
    s1x=((0.067.*Xdis)-0.78)-5.15   %regression equation for the angle difference 
    s3y=((0.12.*Ydis)+0.7)-14.5     %
  
    hold on
    

   robot.moveRelativeLinear([s1x 0 s3y 0 0 0 50 ],[0 1 2 3 4 5 6],10); % move the robot and close the gripper
     hold off
    end
    

    robot.moveRelativeLinear([0 0 0 0 0 0 -30 ],[0 1 2 3 4 5 6],3); %open the gripper
 
    if(a==0)%when no object is detected
    robot.centerJoints; %move the robot to the center position
    robot.moveRelativeLinear([45 0 0 0 0 0 0],[0 1 2 3 4 5 6],15);%moves to right to try to find an object
       pause(1.5);
        data1 = getsnapshot(vid);   %same as used befor, it will try to detect an object
        diff_im1 = imsubtract(data1(:,:,1), rgb2gray(data1));
        diff_im1 = im2bw(diff_im1,0.18);
        diff_im1 = bwareaopen(diff_im1,300);
        bw1 = bwlabel(diff_im1, 8);
        stats = regionprops(bw1, 'BoundingBox', 'Centroid');   
         D=sum(bw1(:));  
         if(D==0)   %if no object detected
         robot.moveRelativeLinear([-90 0 0 0 0 0 0],[0 1 2 3 4 5 6],15);%moves to the left to try to find an object
         pause(1.5);
         end
    end
  
    
    
%     hold on
%     %This is a loop to bound the red objects in a rectangular box.
%     for object = 1:length(stats)
%         bb = stats(object).BoundingBox;
%         bc = stats(object).Centroid;
%         rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
%         plot(bc(1),bc(2), '-m+')
%         a=text(bc(1)+15,bc(2), strcat('X: ', num2str(round(bc(1))), '    Y: ', num2str(round(bc(2)))));
%         set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
%     end  
%     
%     
%     hold off
end%end of the while loop


% Stop the video aquisition.
stop(vid);

% Flush all the image data stored in the memory buffer.
flushdata(vid);

% Clear all variables
clear all









