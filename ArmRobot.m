classdef ArmRobot < handle
    % ArmRobot is a class that contains all the functions that need to
    % control the Arm Robot.
    %
    %Authors:  Vu Nguyen        (vkn3505@rit.edu)
    %          Ryan M. Bowen    (rmb3518@rit.edu)
    %
    %Modified: 10/5/2011 (rmb3518)
    
    properties(Constant)
        %----- Arm Joints--------
        base            =   0;              % Base joint number
        shoulder        =   1;              % Shoulder joint number
        elbow           =   2;              % Elbow joint number
        elbowTwist      =   3;              % Elbow twist joint number
        wrist           =   4;              % Wrist joint number
        wristTwist      =   5;              % Wrist twist joint number
        gripper         =   6;              % Gripper joint number
        
        allJoints       = [0 1 2 3 4 5 6];  % Array containing all joint numbers
        
        defaultCenter   = 1500;             % Default servo center value
        robotn2mins     = [643 759 580 647 608 563 900];
        robotn2maxs     = [2500 2211 2210 2380 2420 2337 2191];
        maxdegrees      = [90 90 90 90 90 90 90];
        mindegrees      = [-90 -90 -77 -90 -90 -90 -90];
        robotn2centers = [1500 1520 1380 1510 1520 1440 1500];
       end
    
    properties(SetAccess = private, GetAccess = private)
        
        controller;                     % Communicator for SSC-32
        frame                           % Tool Frame w.r.t Universe
        
        linkLengths     = ones(1,8);    % Lengths of the links of the robot
        
        servoCenters    = ones(1,7);    % Center values for the servos
        servoMin        = ones(1,7);    % Min values for each servo
        servoMax        = ones(1,7);    % Max values for each servo
        
        servoValues     = ones(1,7);    % Current values of the servos
        
        connected       = 0;            % Connectivity flag

    end
    
    properties(SetAccess=public, GetAccess=public)
            
        % Universal frames
        UtA     = eye(4,4);
        UtB     = eye(4,4);
        UtC     = eye(4,4);
        UtD     = eye(4,4);
        UtE     = eye(4,4);
        UtF     = eye(4,4);
        UtT     = eye(4,4);
        
        % Intermediate frames
        AtB     = eye(4,4);
        BtC     = eye(4,4);
        CtD     = eye(4,4);
        DtE     = eye(4,4);
        EtF     = eye(4,4);
        FtT     = eye(4,4);
        
    end
    
    methods(Access=public)
        
        function obj = ArmRobot(portName)
            % ArmRobot Contructor
            %
            % @params  portName - port to communicate with robot ie 'COM1'
            
            % Create new SSC-32 object
            obj.controller  = SSC32(portName);
            
            % Set the cneters
            obj.setServoCenters(ArmRobot.defaultCenter*ones(1,length(ArmRobot.allJoints)));
            
            % Set some general bounds for the robot
            obj.setServoBounds([500 760 900 550 550 500 1100],...
                [2400 2100 1800 2300 2300 2300 2300]);
            
%             obj.setIntermediateFrame();
%             obj.updateToolFrame();
            
        end
        
        function delete(obj)
            % ArmRobot destrctor, disconnect from the robot and destroy
            % objects.
            
            obj.disconnect();
            obj.controller.delete();
            clear obj;
            display('Arm Robot destroyed!');
        end
        
        function obj = setServoCenters(obj, centers)
            % Sets the servo center values
            %
            % @params centers - the center values for ALL servos.
            
            if (isequal(length(ArmRobot.allJoints),length(centers)))
                obj.servoCenters = centers;
            else
                error('Center Values must match the sizeof(Joints.allJoints)');
            end
        end
        
        function obj = setServoBounds(obj,min,max)
            % Sets the bounds for ALL servos.
            %
            % @params min - the min servo values.
            %         max - the max servo values.
            
            if( isequal(length(min),length(max)) && ...
                    (isequal(length(min),length(ArmRobot.allJoints))) )
                obj.servoMin = min;
                obj.servoMax = max;
            else
                error('Min and Max values must match the sizeof(Joints.allJoints)');
            end
        end
        
        function obj = setLinkLengths(obj,linkLengths)
            % Sets the lengths of ALL the links for the robot.
            %
            % @params linkLengths - link lengths for ALL the links.
            
            if (isequal(length(obj.linkLengths),length(linkLengths)))
                obj.linkLengths = linkLengths;
            else
                error('Link Lengths must match.');
            end
            
%             obj.setIntermediateFrame();
%             obj.updateToolFrame();
        end
        
        function obj = moveJoints(obj,values,joints)
            % Move servos based on the values and joint numbers specified,
            % throws an error is length(values) ~= length(joints) or if any
            % of the values are out of bounds.
            %
            % @params  value  - servo values (array or single value)
            %          joints - respective joint numbers for values.
            
            % Check lengths error if not the same
            if (isequal(length(values),length(joints)) )
                obj.servoValues = values;
            else
                error('"values" must match the sizeof("joints")');
            end
            
            % Out of bounds error flag
            outOfBounds = 0;
            
            % Store temp values of the servos
            tempServoValues = zeros(size(obj.servoValues));
            
            % Iterate through all the values
            for i = 1:length(values)
                
                % Check bounds break out if violation.
                if( (values(i) > obj.servoMax(joints(i)+1)) || ...
                        (values(i) < obj.servoMin(joints(i)+1) ))
                    outOfBounds = 1;
                    break;
                else
                    % In bound update new servo values.
                    tempServoValues(joints(i)+1) = values(i);
                end
            end
            
            % All values are in bounds
            if( ~outOfBounds )
                % Send values to SSC-32 object
                obj.controller.setServoValues(values,joints);
                % Save servo values state
                obj.servoValues = tempServoValues;
            else
                error('Servo value outside of bounds');
            end
            
        end
        
        function obj = centerJoints(obj)
            % Sets all the servos to their center positions.
            obj.moveJoints(obj.servoCenters,ArmRobot.allJoints);
        end
        
        function obj = connect(obj)
            % Connects to the robot by connecting to the SSC-32 controller
            display('Attemping to Connect to Robot...');
            obj.controller.connect();
            obj.connected = 1;
            display('Connected to Robot!');
        end
        
        function obj = disconnect(obj)
            % Disconnects from the robot by disconnecting from the SSC-32
            if( obj.connected )
                obj.controller.disconnect();
                obj.connected = 0;
                display('Robot disconnected!');
            end
        end
        
        function servoValues = getServoValues(obj)
            % Returns the current servos values
            servoValues = obj.servoValues;
        end
        
        function obj = moveAbsolute(obj,degrees,joints)
            % Moves the specified servo(s) to the desired degree(s) with
            % respect the servo(s) home position. 
            %
            % @params degrees - the desired degree(s) to set.
            %         joints  - respective joint number(s) for degree(s).
                        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            vals = zeros(size(degrees));
            vals = obj.deg2val(degrees,joints);
            obj.moveJoints(vals,joints);
            
        end
        
        function obj = moveRelative(obj,degrees,joints)
            % Moves the specified servo(s) to the desired degree(s) with 
            % respect to the current position(s) of the servos.
            %
            % @params degrees - the desired relative degree(s) to set.
            %         joints  - respective joint number(s for degree(s).
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            CurrentVals = obj.servoValues; 
            CurrVal = [];
            [m,ncol] = size(degrees);
            for i=1:ncol;
                    if CurrentVals(joints(i)+1)==0
                        CurrVal = [CurrVal obj.robotn2centers(joints(i)+1)];                
                    else
                        CurrVal = [CurrVal CurrentVals(joints(i)+1)];                
                    end
            end
                
            CurrDegrees = obj.val2deg(CurrVal,joints);
            newdegrees = CurrDegrees + degrees;
            vals = obj.deg2val(newdegrees,joints);
            
            for i=1:ncol
                if vals(1,i) > obj.robotn2maxs(1,joints(i)+1)
                   vals(1,i) = obj.robotn2maxs(1,joints(i)+1);
                end
                if vals(1,i) < obj.robotn2mins(1,joints(i)+1)
                   vals(1,i) = obj.robotn2mins(1,joints(i)+1);
                end
            end
            
            obj.moveJoints(vals,joints);
                        
        end
        
        function obj = moveRelativeLinear(obj,degrees,joints,nStep)
            % Moves the specified servo(s) to the desired degree(s) with
            % respect to the current positions(s) of the servos. This is
            % done in a linear fashion with number of steps specified.
            %
            % @params degrees - the desired relative degree(s) to set.
            %         joints  - respective joint number(s for degree(s).
            %         nStep   - the number of steps in linear motion. 
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            StepSize = degrees/nStep;
            for i=1:nStep
                obj.moveRelative(StepSize,joints);
                pause(0.015);
            end
        end
        
        function obj = moveAbsoluteLinear(obj,degrees,joints,nStep)
            % Moves the specified servo(s) to the desired degree(s) with
            % respect to the home positions(s) of the servos. This is
            % done in a linear fashion with number of steps specified.
            %
            % @params degrees - the desired relative degree(s) to set.
            %         joints  - respective joint number(s for degree(s).
            %         nStep   - the number of steps in linear motion.  
            
                 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            CurrentVals = obj.servoValues; 
            CurrVal = [];
            [m,ncol] = size(degrees);
            for i=1:ncol;
                if not(degrees(i)==0)
                    if CurrentVals(joints(i)+1)==0
                        CurrVal = [CurrVal obj.robotn2centers(joints(i)+1)];                
                    else
                        CurrVal = [CurrVal CurrentVals(joints(i)+1)];                
                    end
                end
            end
                
            CurrDegrees = obj.val2deg(CurrVal,joints);
            newdegrees = degrees - CurrDegrees ;
            obj.moveRelativeLinear(newdegrees,joints,nStep);
        end
        
%         function setIntermediateFrame(obj) 
%             % Sets the initial value of the intermediate frames.
%    
%             obj.UtA = FrameTransformation.trans('xz',[-obj.linkLengths(1) obj.linkLengths(2)]);
%             obj.AtB = FrameTransformation.trans('z',obj.linkLengths(3))*FrameTransformation.rot('x',90);            
%             obj.BtC = FrameTransformation.trans('y',obj.linkLengths(4));                        
%             obj.CtD = FrameTransformation.trans('x',obj.linkLengths(5))*FrameTransformation.rot('y',90);                        
%             obj.DtE = FrameTransformation.trans('z',obj.linkLengths(6))*FrameTransformation.rot('y',-90);            
%             obj.EtF = FrameTransformation.trans('x',obj.linkLengths(7))*FrameTransformation.rot('y',90);            
%             obj.FtT = FrameTransformation.trans('z',obj.linkLengths(8));        
%             
%         end
%         
%         function updateToolFrame(obj)
%             % Update the tool frame given that the intermediate ones have
%             % already been updated. 
%            
%             obj.UtB = obj.UtA*obj.AtB;
%             obj.UtC = obj.UtB*obj.BtC;
%             obj.UtD = obj.UtC*obj.CtD;
%             obj.UtE = obj.UtD*obj.DtE;            
%             obj.UtF = obj.UtE*obj.EtF;
%             obj.UtT = obj.UtF*obj.FtT;            
%         end
%         
%         function displayFrames(obj)
%             % Display all frames
%             display('UtA');obj.UtA
%             display('AtB');obj.AtB
%             display('BtC');obj.BtC
%             display('CtD');obj.CtD
%             display('DtE');obj.DtE
%             display('EtF');obj.EtF
%             display('FtT');obj.FtT
%             
%             display('UtA');obj.UtA
%             display('UtB');obj.UtB
%             display('UtC');obj.UtC
%             display('UtD');obj.UtD
%             display('UtE');obj.UtE            
%             display('UtT');obj.UtT            
%         end       
        
    end
    
    methods(Static)
        
        function value = deg2val(degree,joints)
            % Converts degree(s) values into servo value(s) for the specified
            % joints.
            %
            % @params degree - degree(s) to be converted
            %         joints - respectful joint numbers
            robotmax = ArmRobot.robotn2maxs;
            robotmin = ArmRobot.robotn2mins;
            value = 0.*degree;
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            for i = 1 : length(degree)
                o = joints(i)+1;
                switch joints(i)
                    case {0,'base'}
                        % base conversion
                        m = ((robotmax(1,o)-robotmin(1,o))/(180));
                        value(1,i) = (m*degree(1,i))+m*90+robotmin(1,o);                        
                    case {1,'shoulder'}
                        % shoulder conversion                   
                        m = ((robotmax(1,o)-robotmin(1,o))/(180));
                        value(1,i) = (m*degree(1,o))+m*90+robotmin(1,o);                                                
                    case {2,'elbow'}
                        % elbow conversion
                        m = ((robotmax(1,o)-robotmin(1,o))/(90+77));
                        value(1,i) = (m*degree(1,i))+m*77+robotmin(1,o);                                                
                    case {3,'elbowTwist'}
                        % elbowTwist conversion
                        m = ((robotmax(1,o)-robotmin(1,o))/(180));
                        value(1,i) = (m*degree(1,i))+m*90+robotmin(1,o);                                                
                    case {4,'wrist'}
                        % wrist conversion
                        m = ((robotmax(1,o)-robotmin(1,o))/(180));
                        value(1,i) = (m*degree(1,i))+m*90+robotmin(1,o);                                               
                    case {5,'wristTwist'}
                        %wristTwist conversion
                        m = ((robotmax(1,o)-robotmin(1,o))/(180));
                        value(1,i) = (m*degree(1,i))+m*90+robotmin(1,o);                                                
                    case {6,'gripper'}
                        % gripper conversion
                        m = ((robotmax(1,o)-robotmin(1,o))/(180));
                        value(1,i) = (m*degree(1,i))+m*90+robotmin(1,o);                                                
                end
            end
            
        end
        
        function degree = val2deg(val,joints)
            % Converts servo values(s) values into degree(s)for the specified
            % joints.
            %
            % @params val - servo value(s) to be converted
            %         joints - respectful joint numbers
            degree = 0.*val;
            robotmax = ArmRobot.robotn2maxs;
            robotmin = ArmRobot.robotn2mins;        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FINISH IMPLEMENTING THIS FUNCTION            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            for i = 1 : length(degree)
                o = joints(i)+1;
                switch joints(i)
                    case {0,'base'}
                        % base conversion
                        m = (180)/(robotmax(o) - robotmin(o));
                        degree(1,i) = (m*(val(1,i)))-m*robotmin(o)-90;
                    case {1,'shoulder'}
                        % shoulder conversion
                        m = (180)/(robotmax(o) - robotmin(o));
                        degree(1,i) = (m*(val(1,i)))-m*robotmin(o)-90;
                    case {2,'elbow'}
                        % elbow conversion
                        m = (90+77)/(robotmax(o) - robotmin(o));
                        degree(1,i) = (m*(val(1,i)))-m*robotmin(o)-77;
                    case {3,'elbowTwist'}
                        % elbowTwist conversion
                        m = (180)/(robotmax(o) - robotmin(o));
                        degree(1,i) = (m*(val(1,i)))-m*robotmin(o)-90;
                    case {4,'wrist'}
                        % wrist conversion
                        m = (180)/(robotmax(o) - robotmin(o));
                        degree(1,i) = (m*(val(1,i)))-m*robotmin(o)-90;
                    case {5,'wristTwist'}
                        %wristTwist conversion
                        m = (180)/(robotmax(o) - robotmin(o));
                        degree(1,i) = (m*(val(1,i)))-m*robotmin(o)-90;
                    case {6,'gripper'}
                        % gripper conversion
                        m = (180)/(robotmax(o) - robotmin(o));
                        degree(1,i) = (m*(val(1,i)))-m*robotmin(o)-90;
                end
            end
            
        end
    end
    
end

