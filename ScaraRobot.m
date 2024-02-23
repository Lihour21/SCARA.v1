% SCARA Robot
% ______________________________________________________________________
% There is a bug wich prevents to open the GUI directly. It is necessary to
% execute from the command window: 
%
% >>SCARA.v1 Robot
%_______________________________________________________________________
% To execute the GUI with no robot: disable the communication with the
% Arduino (line 106): handles.portPresent = false;
%_______________________________________________________________________
% Graphical User Interface: SCARA.v1 Robot
%   Author:
%       S.Lihour
%
%       AUTOBOTx Lab, AI FARM Factory
%
%       Phnom Penh, Cambodia
%
%       Platform:   GUI MATLAB R2020a
%
%       January 2024
%_______________________________________________________________________
function varargout = ScaraRobot(varargin)
% SCARAROBOT MATLAB code for ScaraRobot.fig
%      SCARAROBOT, by itself, creates a new SCARAROBOT or raises the existing
%      singleton*.
%
%      H = SCARAROBOT returns the handle to a new SCARAROBOT or the handle to
%      the existing singleton*.
%
%      SCARAROBOT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SCARAROBOT.M with the given input arguments.
%
%      SCARAROBOT('Property','Value',...) creates a new SCARAROBOT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ScaraRobot_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ScaraRobot_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ScaraRobot

% Last Modified by GUIDE v2.5 22-Feb-2024 16:29:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ScaraRobot_OpeningFcn, ...
                   'gui_OutputFcn',  @ScaraRobot_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% Initialization of variables and handles
% --- Executes just before ScaraRobot is made visible.
function ScaraRobot_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ScaraRobot (see VARARGIN)

% Choose default command line output for ScaraRobot
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ScaraRobot wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% Code for robot control 
% Time events (timer)
handles.t = timer;
handles.t.Period = 1;
handles.t.ExecutionMode = 'fixedRate';
handles.t.TimerFcn = {@timer1,hObject,handles};
start(handles.t)

% General variables
handles.click = false;
handles.estadoSwitche = false;
handles.arduinoList = false;
% Code to enable communication (false to use the GUI without Arduino)
handles.portPresent = false; %true / false
handles.pointsCounter = 0;

% Code to set up the serial port
if handles.portPresent
    % Port address (e.g.: Windows: 'COM14', OSX: '/dev/tty.usbmodem641'. 
    % It is possible to use instrfind( ) to identify the port address
    handles.port = serialport('COM16' ,9600); 
    set(handles.port,'BAUDRATE');
    handles.port.BytesAvailableFcnMode = 'terminator';
    fopen(handles.port);
    % Waiting for serial port
    tf = 0;
    while tf == 0
        tf = strcmp(get(handles.port,'Status'), 'open');
    end
    handles.port.ReadAsyncMode = 'continuous';
end

% Table of preloaded (x,y) coordinates for inverse kinematics
handles.coordenadasXY = {14 160; 40 180; 40 130; 0 180; 0 130};
set(handles.uitable2,'data',handles.coordenadasXY);

% Table of preloaded (q1,q2) angles for forward kinematics
handles.anglesq1q2 = {120 60; 105 57; 115 42; 120 70; 133 60};
set(handles.uitable3,'data',handles.anglesq1q2);
% Preloaded angles for initial configuration
handles.angleMotorIzDefecto = 131.9; % deg
handles.angleMotorDrDefecto = 48.1;  % deg
handles.angleMotorLeft = handles.angleMotorIzDefecto;
handles.angleMotorRight = handles.angleMotorDrDefecto;
axes(handles.axes1);

% Joint positions (Robot drawing)
handles.x = [0 0 0 0 0];
handles.y = [0 0 0 0 0];

% Array for robot animation (including axis and central point)
handles.xFigura = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
handles.yFigura = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];

% Link's parameters - mm. (to be updated to the specific design)
handles.L1 = 133; handles.L4 = handles.L1;
handles.L2 = 122; handles.L3 = handles.L2;
% Point A1 (fixed)
handles.x(1) = 0; 
handles.y(1) = 0;
% Point A2 (Fixed)
handles.x(5) = 28;
handles.y(5) = 0;
% MIC: Maximum Inscribed Circle within the workspace (used to filter out
% positions). To calculate it see the fiveRmic( ) function. The center of 
% the circle is the 'mechanism's home'.
handles.Rmic = 88.7556*.95; % 95% to prevent singularities
handles.Xmic = handles.x(5)/2; 
handles.Ymic = 164.6503;

% Parameters visualization (GUI)
set(handles.text38,'String',num2str(handles.L1));
set(handles.text42,'String',num2str(handles.L2));
set(handles.text46,'String',num2str(handles.x(1)));
set(handles.text48,'String',num2str(handles.x(5)));

% Arduino communication commands
if handles.portPresent
    handles.port.BytesAvailableFcn = {@recibirSerie,hObject,handles};
end
guidata(hObject,handles);

if handles.portPresent
    % Waiting Arduino's response
    while not(handles.arduinoList)
        handles = guidata(hObject);
    end
    % Sending (x,y) coordinates to Arduino (Home)
    fwrite(handles.port, '*XY=')
    % x
    fwrite(handles.port, handles.Xmic, 'float32')
    % y
    fwrite(handles.port, handles.Ymic, 'float32')
    fwrite(handles.port, '#')
end
% Updating drawing in GUI
handles.x1 = handles.Xmic;
handles.y1 = handles.Ymic;
guidata(hObject,handles);
cinematicaInversa(hObject, eventdata, handles)

% --- Outputs from this function are returned to the command line.
function varargout = ScaraRobot_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% Closing Arduino communication
% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, ~, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
if handles.portPresent
    fclose(handles.port);
    delete(handles.port);
end
stop(handles.t);
delete(handles.t);
delete(hObject);


% Forward kinematics function
function  cinematicaDirecta(~, ~, handles)
xb = handles.x(1) + handles.L1*cos(handles.angleMotorLeft*pi/180);
yb = handles.y(1) + handles.L1*sin(handles.angleMotorLeft*pi/180);
xd = handles.x(5) + handles.L4*cos(handles.angleMotorRight*pi/180);
yd = handles.y(5) + handles.L4*sin(handles.angleMotorRight*pi/180);
d = sqrt((xd - xb)^2 + (yd - yb)^2);
phi = atan2((yd - yb),(xd - xb));
gama = acos(d/(2*handles.L2));  % Assuming L2 = L3
xc = xb + handles.L2*cos(phi + gama);
yc = yb + handles.L2*sin(phi + gama);
% Updating the GUI
set(handles.text23,'String',num2str(xc));
set(handles.text24,'String',num2str(yc));
set(handles.text68,'String',num2str(handles.angleMotorLeft))
set(handles.text69,'String',num2str(handles.angleMotorRight))
handles.x(3) = xc;
handles.y(3) = yc;
handles.x(2) = xb;
handles.y(2) = yb;
handles.x(4) = xd;
handles.y(4) = yd;
axis(handles.axes1);
plot([-124 154],[0 0], [0 0], [-5 245], [14 14], [157 173], [6 22], [165 165], ...
    handles.x,handles.y,handles.xFigura, handles.yFigura,'LineWidth', 2)
axis(handles.axes1,[-124  154   -5  245]); 

% Inverse kinematics
function cinematicaInversa(hObject, eventdata, handles)
% Left motor
d1 = sqrt(handles.x1^2 + handles.y1^2);
gama1 = atan2(handles.y1, handles.x1);
b = (handles.L1^2 + d1^2 - handles.L2^2) / (2 * handles.L1 * d1); 
if b > 1		
	b = 1;
end
if b < -1		
	b = -1;
end
psi1 = acos(b);
handles.angleMotorLeft = (psi1 + gama1)*180/pi;
% Rigth motor
d2 = sqrt((handles.x1 - handles.x(5))^2 + (handles.y1 - handles.y(5))^2); 
gama2 = atan((handles.y1 - handles.y(5)) / (handles.x(5) - handles.x1));
if gama2 >= 0	
	gama2 = pi - gama2;
else
	gama2 = -gama2;
end
if(gama2 > pi)	
	gama2 = -gama2;
end
b = (handles.L4^2 + d2^2 - handles.L3^2) / (2 * handles.L4 * d2);
if (b > 1)	
	b = 1;
end
if (b < -1)	
	b = -1;
end
psi2 = acos(b);
handles.angleMotorRight = (gama2 - psi2)*180/pi;
% Updating the GUI
set(handles.text68,'String',num2str(handles.angleMotorLeft));
set(handles.text69,'String',num2str(handles.angleMotorRight));
guidata(hObject,handles);
cinematicaDirecta(hObject, eventdata, handles)

% (X,Y) table data to robot
% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Data reading
handles.pointsCounter = 0;
data = get(handles.uitable2,'data');
t = size(data);
for i = 1:t(1)
    if not(isnan(data{i,1}))   
        if handles.portPresent
            fwrite(handles.port,'*XY=')
            % x coordinate
            fwrite(handles.port,data{i,1},'float32')
            % y coordinate
            fwrite(handles.port,data{i,2},'float32')
            fwrite(handles.port,'#')
        end
            handles.pointsCounter = handles.pointsCounter + 1;
            set(handles.text72,'String',num2str(handles.pointsCounter));
            x1 = data{i,1};
            y1 = data{i,2};
            handles.x1 = x1;
            handles.y1 = y1;
            guidata(hObject,handles);
            cinematicaInversa(hObject, eventdata, handles)
            pause(0.01 + (0.5 - get(handles.slider4,'value') / 2));
    end
end

% --- Executes when entered data in editable cell(s) in uitable2.
function uitable2_CellEditCallback(~, ~, ~)
% hObject    handle to uitable2 (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when selected cell(s) is changed in uitable2.
function uitable2_CellSelectionCallback(~, ~, ~)
% hObject    handle to uitable2 (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) currently selecteds
% handles    structure with handles and user data (see GUIDATA)


% (q1,q2) table to robot
% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Reading data
handles.pointsCounter = 0;
data = get(handles.uitable3,'data');
 t = size(data);
for i = 1:t(1)
    if not(isnan(data{i,1}))   
        if handles.portPresent
                % Sending angles to Arduino
                fwrite(handles.port,'*MS=')
                % Left motor
                fwrite(handles.port,data{i,1},'float32')
                % Rigth motor
                fwrite(handles.port,data{i,2},'float32')
                fwrite(handles.port,'#')
        end
        handles.pointsCounter = handles.pointsCounter + 1;
        set(handles.text72,'String',num2str(handles.pointsCounter));
        q1 = data{i,1};
        q2 = data{i,2};
        handles.angleMotorLeft = q1;
        handles.angleMotorRight = q2;            
        guidata(hObject,handles);
        cinematicaDirecta(hObject, eventdata, handles)
        pause(0.01 + (0.5 - get(handles.slider4,'value') / 2));
    end
end


% Inverse kinematics from screen (click on robot workspace)
% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% (x,y) coordinartes enable from mouse
handles.click = true;
guidata(hObject,handles);
% (x,y) coordinates reading
pos = get(hObject,'currentPoint');
handles.x1 = pos(1);
handles.y1 = pos(3);

% Sending (x,y) coordinates to robot
if handles.portPresent
    fwrite(handles.port,'*XY=')
    % x
    fwrite(handles.port,handles.x1,'float32')
    % y
    fwrite(handles.port,handles.y1,'float32')
    fwrite(handles.port,'#')
end
guidata(hObject,handles);
cinematicaInversa(hObject, eventdata, handles)


% Capturing cursor motion from screen
% --- Executes on mouse motion over figure - except title and menu.
function figure1_WindowButtonMotionFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% This function calibrates the graphics window with respect to the robot's
% parameters (linear model). 
% Algorithm
%   * Assign: despl = 0, escal = 1. Execute the GUI
%   * Locate the cursor on the origin. The x and y values correspon to the
%   displacement.
%   * Adjust the values of desplX and desplY.
%   * Open the GUI. Locate the cursor far away from the origin (e.g. in
%   (150,200)). The scale factor will be: (cursor measure) / (plot value)
desplX = 50;
escalX = 150 / 164; 
desplY = 35;
escalY = 201 / 224;
pos = get(hObject,'currentPoint');
x = get(handles.axes1,'Xlim');
y = get(handles.axes1,'Ylim');
handles.x1 = x(1) + 1 * (escalX) * (pos(1) - desplX);
handles.y1 = y(1) + 1 * (escalY) * (pos(2) - desplY);
set(handles.text59,'String',num2str(round(handles.x1)));
set(handles.text60,'String',num2str(round(handles.y1)));
if handles.click
    if handles.portPresent
        % Sending coordinates
        fwrite(handles.port,'*XY=')
        % x
        fwrite(handles.port,handles.x1,'float32')
        % y
        fwrite(handles.port,handles.y1,'float32')
        fwrite(handles.port,'#')
    end
    cinematicaInversa(hObject, eventdata, handles)
end
guidata(hObject,handles);


% Click release
% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonUpFcn(hObject, ~, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Closing coordinates availability
handles.click = false;
guidata(hObject,handles);

% LF terminator from serial port 
function recibirSerie(~,~,hObject,~)
handles = guidata(hObject);
if handles.portPresent
    dato = fscanf(handles.port);
    if dato(1) == 'R'
        handles.arduinoList = true;
        guidata(hObject,handles);
    end
end

% (q1,q2) table from file
% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(~, ~, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fileID = fopen('angles.txt');
if fileID == 3  
    sizeC = [2 Inf];
    anglesq1q2 = fscanf(fileID,'%f  %f',sizeC);
    set(handles.uitable3,'data',num2cell(anglesq1q2'));
    fclose(fileID);
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(~, ~, ~)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(~, ~, ~)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% timer function
function timer1(~,~,~,handles)
a = datestr(now);
set(handles.text73, 'String',a);


% --- Executes on slider movement.
function slider4_Callback(~, ~, ~)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, ~, ~)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% (x,y) coordinates from file
% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
opcion = get(hObject, 'Value');
switch opcion
    case 1 % Rooster
        archive = 'Gallo.txt';
    case 2 % Hand
        archive = 'mano.txt';
    case 3 % Polygon
        archive = 'poligono.txt';
    case 4 % Penguin
        archive = 'Pinguino.txt';
    case 5 % Flamingo
        archive = 'Flamenco.txt';
    case 6 % Lihour
        archive = 'lihour_1.txt';
    case 7 % Matlab Logo
        archive = 'MatLabLogo.txt';
    otherwise
end
fileID = fopen(archive);
if fileID == 3  
    sizeC = [2 Inf];
    handles.coordenadasXY = fscanf(fileID,'%f  %f',sizeC);
    handles.xFigura = handles.coordenadasXY(1:2:end);
    handles.yFigura = handles.coordenadasXY(2:2:end);
    set(handles.uitable2,'data',num2cell(handles.coordenadasXY'));
    fclose(fileID);
    handles.x1 = handles.xFigura(1);
    handles.y1 = handles.yFigura(1);
    if handles.portPresent
            % Sending (x,y) coordinates (first point)
            fwrite(handles.port,'*XY=')
            % x
            fwrite(handles.port,handles.x1,'float32')
            % y
            fwrite(handles.port,handles.y1,'float32')
            fwrite(handles.port,'#')
    end
    guidata(hObject,handles);
    cinematicaInversa(hObject, eventdata, handles)
else
    disp('File not found');
end
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, ~, ~)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% Function to constrain the end point to the MIC
function inFlag = circunferenciaInscrita(hObject, eventdata, handles)
% Forward kinematics
[ xp, yp ] = cinematicaDirecta(hObject, eventdata, handles);
% Distance from P to the MIC center
d = ( (xp - handles.Xmic)^2 + (yp - handles.Ymic)^2 )^(0.5);
% Evaluating internal / external point
inFlag = handles.Rmic >= d;


% --- Executes during object creation, after setting all properties.
function uibuttongroup2_CreateFcn(~, ~, ~)
% hObject    handle to uibuttongroup2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(~, ~, ~)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key press with focus on uitable2 and none of its controls.
function uitable2_KeyPressFcn(~, ~, ~)
% hObject    handle to uitable2 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
