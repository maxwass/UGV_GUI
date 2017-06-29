function varargout = Control_GUI(varargin)
% CONTROL_GUI MATLAB code for Control_GUI.fig
%      CONTROL_GUI, by itself, creates a new CONTROL_GUI or raises the existing
%      singleton*.
%
%      H = CONTROL_GUI returns the handle to a new CONTROL_GUI or the handle to
%      the existing singleton*.
%
%      CONTROL_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONTROL_GUI.M with the given input arguments.
%
%      CONTROL_GUI('Property','Value',...) creates a new CONTROL_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Control_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Control_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Control_GUI

% Last Modified by GUIDE v2.5 22-Jun-2017 12:25:28

global g_obs_cell;
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Control_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Control_GUI_OutputFcn, ...
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

% --- Executes just before Control_GUI is made visible.
function Control_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Control_GUI (see VARARGIN)
   global g_obs_cell;
   clear g_obs_cell;
   global g_map_boundary_lat;
   global g_map_boundary_lon;
   global test_timer;
   g_map_boundary_lat = [39.952375    39.951886   ];
   g_map_boundary_lon = [-75.192187    -75.190929   ];

%    g_map_boundary_lat = [39.952375    39.952186   ];
%    g_map_boundary_lon = [-75.192187    -75.191929   ];
   plot(g_map_boundary_lon,g_map_boundary_lat,'.')
   plot_google_map
%[x,y] = ginput(4)

% Choose default command line output for Control_GUI
handles.output = hObject;

% test_timer = timer(...
%     'ExecutionMode', 'fixedRate', ...       % Run timer repeatedly
%     'Period', 1, ...                        % Initial period is 1 sec.
%     'TimerFcn', {@update_display,hObject});
% start(test_timer)
% rosshutdown
% MasterIP = '192.168.0.21'; %master is in this case is with mavros on companinon computer
% rosinit(MasterIP)
% global gps;
% gps = rossubscriber('/mavros/global_position/raw/fix', @GPSCallback);

global Longitude;
global Latitude;
global Altitude;

% Update handles structure
guidata(hObject, handles);
%stop(test_timer)
% UIWAIT makes Control_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function GPSCallback(src, message)
    %%ROS callback function for recieving raw GPS data
    global Longitude;
    global Latitude;
    global Altitude;
    Longitude = message.Longitude;
    Latitude  = message.Latitude;
    Altitude  = message.Altitude;
    %fprintf('Latitude: %f, Longitude: %f \n', Latitude, Longitude);
    pause(1);
    drawnow;


% START USER CODE
function update_display(hObject, eventdata, handles)
% Timer timer1 callback, called each time timer iterates.
% Gets surface Z data, adds noise, and writes it back to surface object.
%disp('hello')
% a=1
global h_start;
global Longitude;
global Latitude;
 set(h_start,'XData',Longitude);
 set(h_start,'YData',Latitude);
% END USER CODE


% --- Outputs from this function are returned to the command line.
function varargout = Control_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in sattlite.
function sattlite_Callback(hObject, eventdata, handles)
   
   
   plot_google_map('maptype', 'satellite')



% --- Executes on button press in roadmap.
function roadmap_Callback(hObject, eventdata, handles)
   
   
   plot_google_map('MapType', 'roadmap') 




% --- Executes on button press in aStart.
function aStart_Callback(hObject, eventdata, handles)

addpath('/home/daewon/Documents/MATLAB/ASTAR')

global g_start;
global g_goal;
global g_obs_cell;
global g_map_boundary_lat;
global g_map_boundary_lon;

disp('in ASTAR');
disp('you have these obstacles: ')
length(g_obs_cell)

map_boundary_1 = llToMeters(g_map_boundary_lon(1), g_map_boundary_lat(1));
map_boundary_2 = llToMeters(g_map_boundary_lon(2), g_map_boundary_lat(2));
alpha = 20; % to have enough space for the map
x_map = round([min([map_boundary_1(1), map_boundary_2(1)])-alpha, max([map_boundary_1(1), map_boundary_2(1)])]+alpha);
y_map = round([min([map_boundary_1(2), map_boundary_2(2)])-alpha, max([map_boundary_1(2), map_boundary_2(2)])]+alpha);


start_m = round(llToMeters(g_start(1), g_start(2)));
goal_m = round(llToMeters(g_goal(1), g_goal(2)));

start_ll = metersToll(start_m);
goal_ll = metersToll(goal_m);

obs_m = cell(0);
for i=1:length(g_obs_cell)
    for j=1:size(g_obs_cell{i},1)
        obs_m{i}(j,:) = round(llToMeters(g_obs_cell{i}(j,1), g_obs_cell{i}(j,2)));
    end
end
path = findPathAStar( obs_m, x_map, y_map, start_m, goal_m );

% 
% for i = 1: size(ID_path_old,1)
%     set(ID_path_old(i),'marker','none');
% end
for i=1:size(handles.axes1.Children,1)
    isMark = findprop(handles.axes1.Children(i),'Marker');
    if(~isempty(isMark))
        set(handles.axes1.Children(i),'Marker','none');
    end
end

for i = 1: size(path,1)
   path_ll(i,:) =  metersToll(path(i,:));
   ID_path(i) = plot(path_ll(i,1), path_ll(i,2),'--gs',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
end
fileID = fopen('path.txt','w');


for ii=1:size(path_ll,1)
    fprintf(fileID,'%12.8f\t%12.8f\n',path_ll(ii,1),path_ll(ii,2));
end

%fprintf(fileID,'%12.8f\t%12.8f\n',path_ll);
fclose(fileID);
disp('Path txt saved!!');
aa=1;






% --- Executes on button press in rrt.
function rrt_Callback(hObject, eventdata, handles)



% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
global g_start;
global h_start;
[start_x,start_y] = ginput(1)
 g_start = [start_x, start_y];
%  start_m = (llToMeters(g_start(1), g_start(2)));
%  start_ll = metersToll(start_m);

hold on
h_start = plot(start_x, start_y,'o','linewidth',2,'color','r');
%  plot(start_ll(1), start_ll(2),'o','linewidth',2,'color','b');



% --- Executes on button press in goal.
function goal_Callback(hObject, eventdata, handles);
global g_goal;
[goal_x,goal_y] = ginput(1)
hold on
gl=plot(goal_x, goal_y,'x','linewidth',2,'color','r')
g_goal = [goal_x, goal_y];



% --- Executes on button press in obstacle_4pt.
function obstacle_4pt_Callback(hObject, eventdata, handles)

global g_obs_cell;
if isempty(g_obs_cell)
    obs = cell(0);
end

[obs_x,obs_y] = ginput(4);
obs_4ll=[obs_x,obs_y;obs_x(1), obs_y(1)];

g_obs_cell{length(g_obs_cell) + 1} = obs_4ll;

hold on
h4 = fill(obs_x,obs_y,'r');
set(h4,'facealpha',.5)



% --- Executes on button press in clear.
function clear_Callback(hObject, eventdata, handles)
global start_x start_y;
global goal_x  goal_y;
global g_obs_cell;
global test_timer;
delete(test_timer);

clear all
clear start_x;
clear start_y;
clear goal_x;
clear goal_y;
clear g_obs_cell;


% --- Executes on button press in realtime.
function realtime_Callback(hObject, eventdata, handles)
% hObject    handle to realtime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global g_start;
global Longitude;
global Latitude;
value=get(hObject,'Value');
if (value == 1.0)
    g_start = [Longitude,Latitude];
end
% Hint:  returns toggle state of realtime


% --- Executes on button press in heading.
function heading_Callback(hObject, eventdata, handles)
% hObject    handle to heading (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Attraction.
function Attraction_Callback(hObject, eventdata, handles)
% hObject    handle to Attraction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in loadObs.
function loadObs_Callback(hObject, eventdata, handles)
% hObject    handle to loadObs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global g_obs_cell;
FileName = uigetfile('*.dat');
data = load(FileName);
%data = load('obs.dat');
for i=1:size(data,1)
    g_obs_cell{i}(:,1)=data(i,1:5);
    g_obs_cell{i}(:,2)=data(i,6:10);
    hold on
    h4 = fill(g_obs_cell{i}(:,1),g_obs_cell{i}(:,2),'r');
    set(h4,'facealpha',.5)
end



% --- Executes on button press in saveObs.
function saveObs_Callback(hObject, eventdata, handles)
% hObject    handle to saveObs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global g_obs_cell;
FileName = uiputfile('*.dat','Save As');
fileID = fopen(FileName,'w');
[nrows, ncols] = size(g_obs_cell{1});
formatSpec = '%e %e %e %e %e %e %e %e %e %e\n';
for i=1:length(g_obs_cell)
   fprintf(fileID, formatSpec, g_obs_cell{i}(:,:)); 
end



