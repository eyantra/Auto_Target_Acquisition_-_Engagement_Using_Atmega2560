% /********************************************************************************
% 
%    Copyright (c) 2011, Pranav P(CSE IITB),Arun Sai S                    -*- c -*-
%    All rights reserved.
% 
%    Redistribution and use in source and binary forms, with or without
%    modification, are permitted provided that the following conditions are met:
% 
%    * Redistributions of source code must retain the above copyright
%      notice, this list of conditions and the following disclaimer.
% 
%    * Redistributions in binary form must reproduce the above copyright
%      notice, this list of conditions and the following disclaimer in
%      the documentation and/or other materials provided with the
%      distribution.
% 
%    * Neither the name of the copyright holders nor the names of
%      contributors may be used to endorse or promote products derived
%      from this software without specific prior written permission.
% 
%    * Source code can be used for academic purpose.
%      For commercial use permission form the author needs to be taken.
% 
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%   POSSIBILITY OF SUCH DAMAGE.
% 
%   Software released under Creative Commence cc by-nc-sa licence.
%   For legal information refer to:
%   http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode
% 
% %********************************************************************************/
% 

clear;
clc
%getting the stream from the webcam
vid= videoinput('winvideo',2,'YUY2_640x480');
% Set the properties of the video object
set(vid, 'FramesPerTrigger', Inf);
set(vid, 'ReturnedColorspace', 'rgb')
vid.FrameGrabInterval = 5;
preview(vid)
%opening a serial port
ser=serial('COM25'); % Set COM port
fopen(ser)
%scouting module
upper_limit=60;
lower_limit=0;
angle_per_rotation=5;
vertical_angle=upper_limit;
%there are two modes of scouting where in mode 0 we decrease the angle
%while in mode 1 we increase the servo motor angle
mode=0;
%setting to 0 ie setting angle =160
fprintf(ser,char(58+upper_limit/angle_per_rotation));
while(1)
pause(.5);

while(1)   
    disp('scounting');
    rgb = getsnapshot(vid);
   %rgb=imread('1.bmp');
   %imview(rgb)
   [a b c]=size(rgb);
%subtracting the blue component so as to identify it
   I = imsubtract(rgb(:,:,3), rgb2gray(rgb)); 
   % Convert the resulting grayscale image into a binary image.
   I = im2bw(I,0.18);
   %dcounting the no of white pixels in the image I
   y=a;
    x=b;
    count=0;
   for m=1:a
       for n=1:b
           if (I(m,n)==1)
               count=count+1;   
           end
      end
   end
   disp(count)
   %imview(I)   
   %I=wiener2(I,[3 3]);
%imwrite(I,'shapes.jpg','jpg');
%imview(I)

%I2 = imfill(I,'holes');
disp(vertical_angle);
disp(mode);
%if the object is sufficiently large so as to ignore noise
    if(count>4000)
        
        break;
    else
        %else scouting first up then right then down
        %if vertical angle exceeds 160 should move right and start
        %decrementing angle
        if(vertical_angle>=upper_limit)
            mode=0;
            fprintf(ser,'2');
            pause(.1);
            %fprintf(ser,'4');
            fprintf(ser,char(58+(vertical_angle/angle_per_rotation)));
            vertical_angle=vertical_angle-angle_per_rotation;
            pause(0.07);
        else
             %if vertical angle is below 90 should move right and start
             %incrementing angle
            if(vertical_angle<=lower_limit)
                 mode=1;
                fprintf(ser,'2');
                pause(0.1);
                %fprintf(ser,'6');
                fprintf(ser,char(58+(vertical_angle/angle_per_rotation)));
                 vertical_angle=vertical_angle+angle_per_rotation;
                pause(0.07);
            else
                %if it is not at either end should move either up or down
                %based on the mode
                if(mode==0)
                    %fprintf(ser,'4');
                    fprintf(ser,char(58+(vertical_angle/angle_per_rotation)));
                    vertical_angle=vertical_angle-angle_per_rotation;
                    pause(0.07);
                else
                    %fprintf(ser,'6');
                    fprintf(ser,char(58+(vertical_angle/angle_per_rotation)));
                    vertical_angle=vertical_angle+angle_per_rotation;
                    pause(0.07);
                end
            end
            
        end
    end
end
%imview(I);
disp('found target');
pause(.07);
%end of scouting module
p=0;
while(1)
    %first time can reuse picture taken above else taking snapshot
    if(p~=0)
        rgb = getsnapshot(vid);  
    end
        % Now to track blue objects in real time
        % we have to subtract the blue component 
        % from the grayscale image to extract the blue components in the image.
        I = imsubtract(rgb(:,:,3), rgb2gray(rgb)); 
        % Convert the resulting grayscale image into a binary image.
        I = im2bw(I,0.18);
        %filling holes in images due to noise or reflection
        I=imfill(I,'holes');
        % Remove all those pixels less than 500px
        I = bwareaopen(I,500);
        % Label all the connected components in the image.
        bw = bwlabel(I, 8);   
        %imview(I);
        %analysing theconnected compopnents
        stats = regionprops(bw, 'BoundingBox');
        obj_area=regionprops(bw,'Area');
        
        c1=0;c2=0;area=0;
        if(length(stats)==0)
            break;
        end
        
        for object = 1:length(stats)
            %getting boundry box for each of the objects
            bb = stats(object).BoundingBox;
            curr_area=obj_area(object).Area;
            %finding the maximum area rectagle so as to fing max object
            if(area<curr_area)
                %centre of the object
                c1=bb(1)+bb(3)/2;%x-coordinate of center of the target
                c2=bb(2)+bb(4)/2;%y-coordinate of center of the target
                area=curr_area;
            end
        end
        
p=1;
   %I=wiener2(I,[3 3]);
%imwrite(I,'shapes.jpg','jpg');
%imview(I)

%I2 = imfill(I,'holes');
%find the appropriate box and send the command to the bot
block1=floor(c1*3/x);% y-coordinate of the block (0 or 1 or 2)
block2=floor(c2*3/y);% x-coordinate of the block (0 or 1 or 2)
disp(c1);disp(c2);
%if the center of the target is in the middle block then 
%divide the middle block into 9 blocks and again check which
% block the center falls in.If the center falls in the middle 
%block then the target is centered.

% -----------------------------------
% |		|		|		|
% |	(0,0)	|	(1,0)	|	(2,0)	|
% |		|		|		|
% -----------------------------------
% |		|   |    |	|		|
% |	(0,1)	|   |1,1 |	|	(2,1)	|
% |		|   |	   |	|		|
% -----------------------------------
% |		|		|		|
% |	(0,2)	|	(1,2)	|	(2,2)	|
% |		|		|		|
% -----------------------------------
if(block1==1 && block2==1)
    c11=c1-x/3;
    c22=c2-y/3;
    block1=1;block2=1;
    if(c11<x/9 )
        block1=0;
    end
    if(c11>x/3-x/9)
        block1=2;
    end
    if(c22<y/9 )
        block2=0;
    end
    if(c22>y/3-y/9)
        block2=2;
    end
end
if(block1==0)
    fprintf(ser,'2');%command to rotate the bot left 
end
if(block1==2)
    fprintf(ser,'8');%command to rotate the bot right 
end
pause(0.1);
if(block2==0)
    if(vertical_angle>upper_limit)%check whether the max angle limit of servo motor is reached 
        fprintf(ser,char(1));
        pause(.5);
        vertical_angle=vertical_angle-angle_per_rotation;
        fprintf(ser,char(58+vertical_angle/angle_per_rotation));%rotate the servo motor by 10 degrees downward
    else
        fprintf(ser,char(58+vertical_angle/angle_per_rotation));
        vertical_angle=vertical_angle+angle_per_rotation;
    end
end
if(block2==2)
    if(vertical_angle<lower_limit)%check whether the min angle limit of servo motor is reached 
        fprintf(ser,char(1));
        pause(.5);
        vertical_angle=vertical_angle+angle_per_rotation;
        fprintf(ser,char(58+vertical_angle/angle_per_rotation));%rotate the servo motor by 10 degrees upward
    else
        fprintf(ser,char(58+vertical_angle/angle_per_rotation));
        vertical_angle=vertical_angle-angle_per_rotation;
    end
end
pause(0.1);
disp(3*block1+block2+1);
%fprintf(ser,3*block1+block2+1);
if(block1==1 && block2==1)%if the target is centered
    %fprintf(ser,char(2));
    %pause(0.09);
    fprintf(ser,'5');
    pause(.1);
    disp('centered');
    final_image=getsnapshot(vid);
    imshow(final_image);
    hold on;
    plot([x/2,c1],[y/2,c2],'g','LineWidth',2);%draw a line between the center of image and center of the target.
    plot(x/2,y/2,'r*');%center of image
    plot(c1,c2,'b*');%center of target
   
    pause(1.5);
    break;
end
pause(0.2);
end
end

%closing the serial communications
fclose(ser)
%deleting the videi stream
stop(vid);
delete(vid);
clear vid;
