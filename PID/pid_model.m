function varargout = pid_model(varargin)
% PID_MODEL MATLAB code for pid_model.fig
%      PID_MODEL, by itself, creates a new PID_MODEL or raises the existing
%      singleton*.
%
%      H = PID_MODEL returns the handle to a new PID_MODEL or the handle to
%      the existing singleton*.
%
%      PID_MODEL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PID_MODEL.M with the given input arguments.
%
%      PID_MODEL('Property','Value',...) creates a new PID_MODEL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before pid_model_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to pid_model_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help pid_model

% Last Modified by GUIDE v2.5 12-Dec-2019 17:42:05

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @pid_model_OpeningFcn, ...
                   'gui_OutputFcn',  @pid_model_OutputFcn, ...
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


% --- Executes just before pid_model is made visible.
function pid_model_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to pid_model (see VARARGIN)

% Choose default command line output for pid_model
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes pid_model wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = pid_model_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%%%PID控制算法
ts=0.5;  
sys=tf(4,[1 2 0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');
e_1=0;     
e_2=0;
u_1=0.0;    
u_2=0.0;
y_1=0;     
y_2=0;
%PID参数
global Kp;
global Ki;
global Kd;
u=zeros(1,100);
time=zeros(1,100);
ts=0.5;
for k=1:1:100
    time(k)=k*ts;  
    r(k)=1;    
    y(k)=-1*den(3)*y_2-den(2)*y_1+num(3)*u_2+num(2)*u_1+num(1)*u(k);
    e(k)=r(k)-y(k);
    u(k)=Kp*(e(k)-e_1)+Ki*e(k)+Kd*(e(k)-2*e_1+e_2); 
    u_2=u_1;
    y_2=y_1;
    e_2=e_1;
    u_1=u(k);    	
    y_1=y(k);    	
    e_1=e(k);       
end
maxy=max(y);
axes(handles.axes1);
p1=plot(time,r,'-.');xlim([0,50]);hold on;%指令信号的曲线（即期望输入）
p2=plot(time,y,'b--');xlim([0,50]);ylim([0,maxy+0.07]);hold on;%不含积分分离的PID曲线
title('PID响应--时间')
%峰值时间
for k=1:1:100
        if (y(k)==maxy)
            Tp=ts*k;
            break;
        end
end
%最大值不超过1的情况
if maxy<=0.98
    set(handles.Trise,'String','无穷大');
    set(handles.Tover,'String','无');
    set(handles.Tpeak,'String',Tp);
    set(handles.ess,'String',y(99)-1);
end
%最大值超过1
if maxy>0.98
    for k=2:1:99
        if (y(k)<1)&&(y(k+1)>=0.998)
            Tr=ts*k;
            break;
        end
    end
end
if maxy>0.98
    set(handles.Tpeak,'String',Tp);
    set(handles.Trise,'String',Tr);
    set(handles.Tover,'String',maxy-1);
    set(handles.ess,'String',y(99)-1);
end


maxy
y(99)-1
% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% 重置清空图片 
cla(handles.axes1,'reset');

function Kp_Callback(hObject, eventdata, handles)
% hObject    handle to Kp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Kp;
Kp=str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of Kp as text
%        str2double(get(hObject,'String')) returns contents of Kp as a double


% --- Executes during object creation, after setting all properties.
function Kp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Kp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ki_Callback(hObject, eventdata, handles)
% hObject    handle to Ki (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Ki;
Ki=str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of Ki as text
%        str2double(get(hObject,'String')) returns contents of Ki as a double


% --- Executes during object creation, after setting all properties.
function Ki_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ki (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Kd_Callback(hObject, eventdata, handles)
% hObject    handle to Kd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Kd;
Kd=str2double(get(hObject,'String'));
% Hints: get(hObject,'String') returns contents of Kd as text
%        str2double(get(hObject,'String')) returns contents of Kd as a double


% --- Executes during object creation, after setting all properties.
function Kd_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Kd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1



function Trise_Callback(hObject, eventdata, handles)
% hObject    handle to Trise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Trise as text
%        str2double(get(hObject,'String')) returns contents of Trise as a double


% --- Executes during object creation, after setting all properties.
function Trise_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Trise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Tpeak_Callback(hObject, eventdata, handles)
% hObject    handle to Tpeak (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tpeak as text
%        str2double(get(hObject,'String')) returns contents of Tpeak as a double


% --- Executes during object creation, after setting all properties.
function Tpeak_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tpeak (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Tover_Callback(hObject, eventdata, handles)
% hObject    handle to Tover (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tover as text
%        str2double(get(hObject,'String')) returns contents of Tover as a double


% --- Executes during object creation, after setting all properties.
function Tover_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tover (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ess_Callback(hObject, eventdata, handles)
% hObject    handle to ess (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ess as text
%        str2double(get(hObject,'String')) returns contents of ess as a double


% --- Executes during object creation, after setting all properties.
function ess_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ess (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
imshow(imread('gs.gif'));
% Hint: place code in OpeningFcn to populate axes2


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
imshow(imread('kuangtu.gif'));
% Hint: place code in OpeningFcn to populate axes3


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close;



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
