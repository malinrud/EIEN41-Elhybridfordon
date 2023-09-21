function varargout = OptimizationGUI(varargin)
% OPTIMIZATIONGUI MATLAB code for OptimizationGUI.fig
%      OPTIMIZATIONGUI, by itself, creates a new OPTIMIZATIONGUI or raises the existing
%      singleton*.
%
%      H = OPTIMIZATIONGUI returns the handle to a new OPTIMIZATIONGUI or the handle to
%      the existing singleton*.
%
%      OPTIMIZATIONGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in OPTIMIZATIONGUI.M with the given input arguments.
%
%      OPTIMIZATIONGUI('Property','Value',...) creates a new OPTIMIZATIONGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before OptimizationGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to OptimizationGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help OptimizationGUI

% Last Modified by GUIDE v2.5 15-Sep-2023 10:04:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @OptimizationGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @OptimizationGUI_OutputFcn, ...
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


% --- Executes just before OptimizationGUI is made visible.
function OptimizationGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to OptimizationGUI (see VARARGIN)

% Choose default command line output for OptimizationGUI
handles.output = hObject;
handles.NomReq.Data = [0 0;0 0;0 0;0 0;0 0;0 0;];
handles.PeakReq.Data = [0 0;0 0;0 0;0 0;0 0;0 0;];

handles.EMindex.String = num2str(1);
handles.KironInd.String = num2str(1);
handles.KzInd.String = num2str(1);
handles.qInd.String = num2str(1);
handles.NpInd.String = num2str(1);
handles.KpmInd.String = num2str(1);
handles.K3Ind.String = num2str(1);

handles.EMindex.Visible = 'on';

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes OptimizationGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = OptimizationGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function PECcost_Callback(hObject, eventdata, handles)
% hObject    handle to PECcost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PECcost as text
%        str2double(get(hObject,'String')) returns contents of PECcost as a double


% --- Executes during object creation, after setting all properties.
function PECcost_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PECcost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function EMTransCost_Callback(hObject, eventdata, handles)
% hObject    handle to EMTransCost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EMTransCost as text
%        str2double(get(hObject,'String')) returns contents of EMTransCost as a double


% --- Executes during object creation, after setting all properties.
function EMTransCost_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EMTransCost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end






% --- Executes during object creation, after setting all properties.
function Energy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Energy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in VdcList.
function VdcList_Callback(hObject, eventdata, handles)
% hObject    handle to VdcList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns VdcList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from VdcList


% --- Executes during object creation, after setting all properties.
function VdcList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to VdcList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function EMlength_Callback(hObject, eventdata, handles)
% hObject    handle to EMlength (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EMlength as text
%        str2double(get(hObject,'String')) returns contents of EMlength as a double


% --- Executes during object creation, after setting all properties.
function EMlength_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EMlength (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Nt_Callback(hObject, eventdata, handles)
% hObject    handle to Nt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Nt as text
%        str2double(get(hObject,'String')) returns contents of Nt as a double

% --- Executes during object creation, after setting all properties.
function Nt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Nt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Gr1_Callback(hObject, eventdata, handles)
% hObject    handle to Gr1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Gr1 as text
%        str2double(get(hObject,'String')) returns contents of Gr1 as a double


% --- Executes during object creation, after setting all properties.
function Gr1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Gr1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Gr2_Callback(hObject, eventdata, handles)
% hObject    handle to Gr2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Gr2 as text
%        str2double(get(hObject,'String')) returns contents of Gr2 as a double


% --- Executes during object creation, after setting all properties.
function Gr2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Gr2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in EMSelList.
function EMSelList_Callback(hObject, eventdata, handles)
% hObject    handle to EMSelList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.EMSelList.Value == 1
	plot(handles.EMAxes,1)
elseif handles.EMSelList.Value == 2
    disp('Loading EM Database....')
    load('PMSMdatabase') 
%     PMSMdatabase = Decode(PMSMdatabase);
    handles.EMSelList.UserData.out_database = PMSMdatabase;
%     disp('PMSM Database ready')  
    msgbox('PMSM Database ready','non-modal')
    
elseif handles.EMSelList.Value == 3
    disp('Loading IM Database....')
    load('IMdatabase.mat') 
    handles.EMSelList.UserData.out_database = IMdatabase;
%     disp('IM Database ready')
    msgbox('IM Database ready','non-modal')
end


% Hints: contents = cellstr(get(hObject,'String')) returns EMSelList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from EMSelList


% --- Executes during object creation, after setting all properties.
function EMSelList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EMSelList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function EMindex_Callback(hObject, eventdata, handles)
% hObject    handle to EMindex (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




% Hints: get(hObject,'String') returns contents of EMindex as text
%        str2double(get(hObject,'String')) returns contents of EMindex as a double


% --- Executes during object creation, after setting all properties.
function EMindex_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EMindex (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in ConceptList.
function ConceptList_Callback(hObject, eventdata, handles)
% hObject    handle to ConceptList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.ConceptList.Value == 1
    plot(handles.PowertrainAxes,1)
elseif handles.ConceptList.Value == 2
    A = imread('SingleSpeed.png');
    image(handles.PowertrainAxes,A)
    handles.Gr2.Visible = 'off';
else
    A = imread('2Speed.png');
    image(handles.PowertrainAxes,A)
    handles.Gr2.Visible = 'on';
end




% Hints: contents = cellstr(get(hObject,'String')) returns ConceptList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ConceptList


% --- Executes during object creation, after setting all properties.
function ConceptList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ConceptList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Nunits_Callback(hObject, eventdata, handles)
% hObject    handle to Nunits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Nunits as text
%        str2double(get(hObject,'String')) returns contents of Nunits as a double


% --- Executes during object creation, after setting all properties.
function Nunits_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Nunits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





function SteelMass_Callback(hObject, eventdata, handles)
% hObject    handle to SteelMass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SteelMass as text
%        str2double(get(hObject,'String')) returns contents of SteelMass as a double


% --- Executes during object creation, after setting all properties.
function SteelMass_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SteelMass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function CopperMass_Callback(hObject, eventdata, handles)
% hObject    handle to CopperMass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CopperMass as text
%        str2double(get(hObject,'String')) returns contents of CopperMass as a double


% --- Executes during object creation, after setting all properties.
function CopperMass_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CopperMass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function MagnetMass_Callback(hObject, eventdata, handles)
% hObject    handle to MagnetMass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of MagnetMass as text
%        str2double(get(hObject,'String')) returns contents of MagnetMass as a double


% --- Executes during object creation, after setting all properties.
function MagnetMass_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MagnetMass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes on button press in UpdateEM.
function UpdateEM_Callback(hObject, eventdata, handles)
% hObject    handle to UpdateEM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

index = str2num(handles.EMindex.String);
DrawEM_black_and_white(handles.EMSelList.UserData.out_database(index).designdata.geom,handles.GeomAxes);
% axis(handles.GeomAxes,'equal')
axis(handles.GeomAxes, [0 0.13 0 0.13])

% EM = handles.EMSelList.UserData.out_database(index);
% EM.PACtotmap = squeeze(EM.PACtotmap_harmonics(8,:,:));
% if handles.EMSelList.Value == 2
%     EMtype = 'VIPMSM';
%     EM.etamap = (EM.Tmap.*EM.wvec-EM.Pfemap-EM.Pmecmap)./(EM.Tmap.*EM.wvec+EM.Pcumap+EM.PACtotmap);
% elseif handles.EMSelList.Value == 3
%     EMtype = 'IM';   
%     EM.etamap = EM.eta;
% end
% gg = find(EM.etamap<0); EM.etamap(gg) = 0;
% gg = find(isnan(EM.etamap)); EM.etamap(gg) = 0;
% Imag = EM.Ismap;
% imodpeak = Imag;
% EM.Imax_new = max(EM.Ismap(100,:));
% imodpeak(Imag > EM.Imax_new) = NaN;
% onematrix2 = imodpeak./imodpeak;
% tmodpeak   = onematrix2.*EM.Tmap;
% EM.Tmax_lim = max(tmodpeak);  % max torque boundry
% plot(handles.EMAxes,EM.wvec*30/pi,EM.Tmax_lim)
% hold on
% [C,h] = contourf(handles.EMAxes,EM.wvec*30/pi/1000,EM.Tvec,EM.etamap*100,85:1:98);
% axis(handles.EMAxes,'equal')
% xlim([0 15])





function Weight_Callback(hObject, eventdata, handles)
% hObject    handle to Weight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Weight as text
%        str2double(get(hObject,'String')) returns contents of Weight as a double


% --- Executes during object creation, after setting all properties.
function Weight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Weight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function FrontArea_Callback(hObject, eventdata, handles)
% hObject    handle to FrontArea (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FrontArea as text
%        str2double(get(hObject,'String')) returns contents of FrontArea as a double


% --- Executes during object creation, after setting all properties.
function FrontArea_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FrontArea (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function DragCoeff_Callback(hObject, eventdata, handles)
% hObject    handle to DragCoeff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of DragCoeff as text
%        str2double(get(hObject,'String')) returns contents of DragCoeff as a double


% --- Executes during object creation, after setting all properties.
function DragCoeff_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DragCoeff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function RollResist_Callback(hObject, eventdata, handles)
% hObject    handle to RollResist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of RollResist as text
%        str2double(get(hObject,'String')) returns contents of RollResist as a double


% --- Executes during object creation, after setting all properties.
function RollResist_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RollResist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function MaxSpeed_Callback(hObject, eventdata, handles)
% hObject    handle to MaxSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of MaxSpeed as text
%        str2double(get(hObject,'String')) returns contents of MaxSpeed as a double


% --- Executes during object creation, after setting all properties.
function MaxSpeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MaxSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function AccTime_Callback(hObject, eventdata, handles)
% hObject    handle to AccTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of AccTime as text
%        str2double(get(hObject,'String')) returns contents of AccTime as a double


% --- Executes during object creation, after setting all properties.
function AccTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AccTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in VehReqCalculate.
function VehReqCalculate_Callback(hObject, eventdata, handles)
% hObject    handle to VehReqCalculate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load('WLTC')
load('WLTPclass3')
Cycle_data.SpeedKph = WLTC_Alek.speedKph;
Cycle_data.wheelTrq = WLTC_Alek.wheelTrq;
Cycle_data.Time = WLTPclass3(:,1);
Cycle_data.Speed = WLTPclass3(:,2);
Cycle_data.Distance = WLTPclass3(:,3);
Cycle_data.Altitude = WLTPclass3(:,4);
Cycle_data.Slope = WLTPclass3(:,5);
Cycle_data.Acceleration = diff(Cycle_data.Speed)./diff(Cycle_data.Time);
Cycle_data.Acceleration  = [0; Cycle_data.Acceleration];
Cycle_data.SamplingTime = mean(diff(Cycle_data.Time));
App.Cycle_data = Cycle_data;
App.Cycle_data.SpeedKph = WLTC_Alek.speedKph;
App.Cycle_data.wheelTrq = WLTC_Alek.wheelTrq;
App.Vehicle.rho_air = 1.225;
App.Vehicle.grav = 9.8;
App.Vehicle.mu = 0.9;
App.Vehicle.rw = 0.3;
App.Vehicle.WheelBase_b = 1.3;
App.Vehicle.WheelBase = 2.7;
App.Vehicle.CG_Height = 0.5;

Req.Cont_load_req(:,1) = str2num(handles.MaxSpeed.String);
Req.Cont_load_req(:,2) = 0;  % Inclination
Req.Cont_load_req(:,4) = 1;  % Cd add
Req.Cont_load_req(:,3) = 0;  % mass add

App.Vehicle.Mv = str2num(handles.Weight.String);
App.Vehicle.Av = str2num(handles.FrontArea.String);
App.Vehicle.Cd = str2num(handles.DragCoeff.String);
App.Vehicle.Cr = str2num(handles.RollResist.String);
App.Vehicle.Top_speed_kph = str2num(handles.MaxSpeed.String);
App.Vehicle.Top_speed_mps = App.Vehicle.Top_speed_kph/3.6;
Acctime = str2num(handles.AccTime.String);
Req.Peak_acceleration_req = [0 100 Acctime];

speed_mps = Req.Cont_load_req(:,1)/3.6;

% disp('Calculating vehicle requirements....')
msgbox('Calculating vehicle requirements....', 'Veh Req', 'modal');

[v_kph_row,WheelTrq_Nm_row,WheelPower,Cont_Tractive_torque,Cont_req, Peak_req, Cont_Power, Peak_Power] = Calculate_Wheel_Requirements(App,Req,0);
Peak_Power
% handles.VehReqCalculate.UserData.Peak_Req = Peak_req;
% handles.VehReqCalculate.UserData.Cont_req = Cont_req;
% handles.VehReqCalculate.UserData.Peak_Power = Peak_Power;
% handles.VehReqCalculate.UserData.Cont_Power = Cont_Power;
% hanles.VehReqCalculate.UserData.App = App;
handles.App = App;
handles.App.Peak_Req = Peak_req;
handles.App.Cont_Req = Cont_req;
handles.App.Peak_Power = Peak_Power;
handles.App.Cont_Power = Cont_Power;
handles.App.UseEmThermalModel = 0;
handles.App.Cycle_data = Cycle_data;
guidata(hObject,handles); % save the updated struct

hold(handles.VehReqAxes,'off')
plot(handles.VehReqAxes,NaN)
hold(handles.VehReqAxes,'on')

plot(handles.VehReqAxes, App.Cycle_data.SpeedKph, abs(App.Cycle_data.wheelTrq),'.k','LineWidth',0.2)
hold(handles.VehReqAxes,'on')  
contour(handles.VehReqAxes, v_kph_row,WheelTrq_Nm_row,WheelPower, 'ShowText','on', 'LineWidth',2, 'LevelStep',50)
hold(handles.VehReqAxes,'on') 
plot(handles.VehReqAxes, Peak_req(1:11,1), Peak_req(1:11,2),'or','LineWidth',2)
hold(handles.VehReqAxes,'on') 
plot(handles.VehReqAxes, speed_mps*3.6, Cont_Tractive_torque,'bd','LineWidth',2)
hold(handles.VehReqAxes,'on')  
title(handles.VehReqAxes, 'Wheel Torque [Nm] vs Speed [km/h]')
hold(handles.VehReqAxes,'off')
% disp('Vehicle Calculation Done!')
msgbox('Vehicle Calculation Done!', 'Veh Req', 'modal');



% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over VehReqCalculate.
function VehReqCalculate_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to VehReqCalculate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function Range_Callback(hObject, eventdata, handles)
% hObject    handle to Range (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Range as text
%        str2double(get(hObject,'String')) returns contents of Range as a double


% --- Executes during object creation, after setting all properties.
function Range_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Range (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit33_Callback(hObject, eventdata, handles)
% hObject    handle to edit33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit33 as text
%        str2double(get(hObject,'String')) returns contents of edit33 as a double


% --- Executes during object creation, after setting all properties.
function edit33_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





function RegTag_Callback(hObject, eventdata, handles)
% hObject    handle to RegTag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of RegTag as text
%        str2double(get(hObject,'String')) returns contents of RegTag as a double


% --- Executes during object creation, after setting all properties.
function RegTag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RegTag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in NtCheck.
function NtCheck_Callback(hObject, eventdata, handles)
% hObject    handle to NtCheck (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Nt = str2num(handles.Nt.String);
App.Nw_vec = [4 6 8];
index = str2num(handles.EMindex.String);
EM = handles.EMSelList.UserData.out_database(index);
Position = [1 Nt 1];
[Winding,NtP,checkflag] = NtCheck(Position,EM,App);
if checkflag == 0 
%    disp(['Nt invalid! Possible Nt:' num2str(NtP')]) 
   msgbox(['Nt invalid! Possible Nt:' num2str(NtP')],'NtCheck','modal')
else
%    disp('Nt is valid') 
   msgbox('Nt is valid','NtCheck','modal')
end

% --- Executes on button press in CalculateButtom.
function CalculateButtom_Callback(hObject, eventdata, handles)
% hObject    handle to CalculateButtom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.EMSelList.Value == 2
    App.MachineType = 'VIPMSM';
elseif handles.EMSelList.Value == 3
    App.MachineType = 'IM';
end

EMlength = str2num(handles.EMlength.String);

if handles.VdcList.Value == 1
    Vdc = 800;
elseif handles.VdcList.Value == 2    
    Vdc = 600;
elseif handles.VdcList.Value == 3    
    Vdc = 400;
end

    
% [ReqsFulfilled, Cost, out, Overload, Draw] = Eval_LvsNt(EMlength,Nt,Vdc,handles.EMSelList.UserData.out_database(index),Tmax,rw,vmax/3.6,Kvo,gr,handles.NomReq.Data,handles.PeakReq.Data,EMtype,Nunits);
Nt = str2num(handles.Nt.String);
App.Nw_vec = [4 6 8];
index = str2num(handles.EMindex.String);
EM = handles.EMSelList.UserData.out_database(index);
Position = [1 Nt 1];
[Winding,NtP,checkflag] = NtCheck(Position,EM,App);

ka = EMlength/0.2;
kov = 1;
App.PEC.Udc = Vdc;
App.UseEmThermalModel = 0;
if handles.EMSelList.Value == 2
   [SEM] = Scalefuction_hairpin(ka,Winding,kov,EM,App);
elseif handles.EMSelList.Value == 3
   [SEM] = Scalefuction_IM(ka,Winding,kov,EM,App);
end

handles.MaxPower.String = max(SEM.Tmax_lim)*SEM.basespeedNom/9550;
handles.MaxTorque.String = max(SEM.Tmax_lim);
handles.BaseSpeed.String = SEM.basespeedNom;


hold(handles.ScalingAxes,'off')
plot(handles.ScalingAxes,NaN)
plot(handles.ScalingAxes,SEM.wvec*30/pi,SEM.Tmax_lim,'b','LineWidth',2)
hold(handles.ScalingAxes,'on')
[C,h]=contourf(handles.ScalingAxes,SEM.wvec*30/pi,SEM.Tvec,SEM.etamap*100,80:1:98);
set(h,'EdgeColor',[0 1 0],'LineStyle','-'); h = clabel(C,h); set(h,'FontSize',14)
%     set(h,'BackgroundColor',[0.6 1 0.6],'Edgecolor',[.7 .7 .7])
axis(handles.ScalingAxes, [0 SEM.n_max 0 max(SEM.Tmax_lim)])



% plot(handles.ScalingAxes, Draw.NomWmecEnv,Draw.NomTorqEnv,'b','LineWidth',2)
% hold(handles.ScalingAxes,'on')
% plot(handles.ScalingAxes, Draw.MaxWmecEnv,Draw.MaxTorqEnv,'r','LineWidth',2)
% plot(handles.ScalingAxes, handles.NomReq.Data(:,1),handles.NomReq.Data(:,2),'bo')
% plot(handles.ScalingAxes, handles.PeakReq.Data(:,1),handles.PeakReq.Data(:,2),'rx')
% title(handles.ScalingAxes, 'Wheel Torque vs Speed')
% hold(handles.ScalingAxes,'off')
% 
% contourf(handles.PECAxes, Draw.wmech*60/2/pi/1e3, Draw.TvecInv, Draw.EffInv)
% title(handles.PECAxes, 'PEC')
% axis(handles.PECAxes, [0 Draw.wem_max*60/2/pi/1e3 min(Draw.TvecInv) max(Draw.TvecInv)])
% colorbar(handles.PECAxes)
% 
% contourf(handles.EMEffAxes, Draw.wmech*60/2/pi/1e3, Draw.Tvec, Draw.EffEM')
% title(handles.EMEffAxes, 'EM')
% axis(handles.EMEffAxes, [0 Draw.wem_max*60/2/pi/1e3 0 max(Draw.Tvec)])
% colorbar(handles.EMEffAxes)
% 
% handles.PECcost.String = num2str(Cost.PEC_Cost);
% handles.EMTransCost.String = num2str(Cost.EM_Cost+Cost.Trans);
% handles.SteelMass.String = num2str(Cost.Steel);
% handles.CopperMass.String = num2str(Cost.Copper);
% handles.MagnetMass.String = num2str(Cost.Magnet);
% handles.Overload.String = num2str(Overload);
% if ReqsFulfilled == 1
% %     handles.RegTag.String = 'YES';
% else
% %     handles.RegTag.String = 'NO';
% end



function BattPrice_Callback(hObject, eventdata, handles)
% hObject    handle to BattPrice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BattPrice as text
%        str2double(get(hObject,'String')) returns contents of BattPrice as a double


% --- Executes during object creation, after setting all properties.
function BattPrice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BattPrice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function SteelPrice_Callback(hObject, eventdata, handles)
% hObject    handle to SteelPrice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SteelPrice as text
%        str2double(get(hObject,'String')) returns contents of SteelPrice as a double


% --- Executes during object creation, after setting all properties.
function SteelPrice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SteelPrice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function MagnetPrice_Callback(hObject, eventdata, handles)
% hObject    handle to MagnetPrice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of MagnetPrice as text
%        str2double(get(hObject,'String')) returns contents of MagnetPrice as a double


% --- Executes during object creation, after setting all properties.
function MagnetPrice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MagnetPrice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function CopperPrice_Callback(hObject, eventdata, handles)
% hObject    handle to CopperPrice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CopperPrice as text
%        str2double(get(hObject,'String')) returns contents of CopperPrice as a double


% --- Executes during object creation, after setting all properties.
function CopperPrice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CopperPrice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ElectricityPrice_Callback(hObject, eventdata, handles)
% hObject    handle to ElectricityPrice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ElectricityPrice as text
%        str2double(get(hObject,'String')) returns contents of ElectricityPrice as a double


% --- Executes during object creation, after setting all properties.
function ElectricityPrice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ElectricityPrice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Mileage_Callback(hObject, eventdata, handles)
% hObject    handle to Mileage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Mileage as text
%        str2double(get(hObject,'String')) returns contents of Mileage as a double


% --- Executes during object creation, after setting all properties.
function Mileage_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Mileage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in EMCostCal.
function EMCostCal_Callback(hObject, eventdata, handles)
% hObject    handle to EMCostCal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
VehRange = str2num(handles.Range.String);
BattPrice = str2num(handles.BattPrice.String);
SteelPrice = str2num(handles.SteelPrice.String);
MagnetPrice = str2num(handles.MagnetPrice.String);
CopperPrice = str2num(handles.CopperPrice.String);
ElectricityPrice = str2num(handles.ElectricityPrice.String);
Mileage = str2num(handles.Mileage.String);



function Length_vec_Callback(hObject, eventdata, handles)
% hObject    handle to Length_vec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Length_vec as text
%        str2double(get(hObject,'String')) returns contents of Length_vec as a double


% --- Executes during object creation, after setting all properties.
function Length_vec_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Length_vec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function G1_vec_Callback(hObject, eventdata, handles)
% hObject    handle to G1_vec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of G1_vec as text
%        str2double(get(hObject,'String')) returns contents of G1_vec as a double


% --- Executes during object creation, after setting all properties.
function G1_vec_CreateFcn(hObject, eventdata, handles)
% hObject    handle to G1_vec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function G2_vec_Callback(hObject, eventdata, handles)
% hObject    handle to G2_vec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of G2_vec as text
%        str2double(get(hObject,'String')) returns contents of G2_vec as a double


% --- Executes during object creation, after setting all properties.
function G2_vec_CreateFcn(hObject, eventdata, handles)
% hObject    handle to G2_vec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Nt_vec_Callback(hObject, eventdata, handles)
% hObject    handle to Nt_vec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Nt_vec as text
%        str2double(get(hObject,'String')) returns contents of Nt_vec as a double


% --- Executes during object creation, after setting all properties.
function Nt_vec_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Nt_vec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in CalculateAll.
function CalculateAll_Callback(hObject, eventdata, handles)
% hObject    handle to CalculateAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.EMSelList.Value == 2
    App.MachineType = 'VIPMSM';
elseif handles.EMSelList.Value == 3
    App.MachineType = 'IM';
end
App.El_price = str2num(handles.Eprice.String); % kWh/€
App.Depreciation_time = 1; % Year
App.Cost_function_k_running = 1;
App.Cost_function_k_uppfront = 1;
App.Nw_vec = [4 6 8];

App.PEC.OptimizerVariables.switchingFrequency = 10000;
App.PEC.OptimizerVariables.Carea =1; 
App.PEC.OptimizerVariables.heatSinkMaterial = 'Cu'; %'Cu' / 'Al'
% App.PEC.UserParameters = UserParameters_init('excel');
% App.PEC.PowerModule = PowerModule_init(App.PEC.UserParameters);
App.PEC.VehicleParameters = 0;  
App.PEC.Udc = 800;                              % Nominal DC-link voltage
App.PEC.UserParameters.DC_VOLTAGE = App.PEC.Udc; 
App.PEC.UserParameters.ENABLE_PLOTTING = 0; % erase plotting 

App.MT.Number_of_gears = 1;                     % sigle-speed MT
App.MT.Cruise_speed    = 60;                    % km/h - optimize trans. eff 
App.MT.Max_gear_ratio   = 19;                    % Max gear ratio to be considered
App.MT.layout          = 1;                     % layout of Transmission
App.MT.RatioRange = 1; % Interval for gear ratio optimization
App.MT.RatioStepOptimization = 0.1; % minimum discreate step for gr opt.
App.MT.MaxRatioStep = 2; % % Maximum ratio between 2 consecutive gears 
App.MT.MinRatioStep = 1.1; % Minimum ratio between 2 consecutive gears 
App.MT.MinNumRatioSteps = 10; 
% App.MT.DefaultParameters = load('defaultParameters.mat');
App.Overloading_time = 30;
App.UseEmThermalModel = 0;

Nt_vec = str2num(handles.Nt_vec.String);  % vector
ka_vec = str2num(handles.Length_vec.String)/0.2;  % vector

VehRange = str2num(handles.Range.String); 
BattPrice = str2num(handles.BattPrice.String); 
BattDep = str2num(handles.BattDep.String); 
AnualM = str2num(handles.AnualM.String);

index = str2num(handles.EMindex.String);
EM = handles.EMSelList.UserData.out_database(index);

App.Cont_req = handles.App.Cont_Req;
App.Peak_req = handles.App.Peak_Req;
App.Peak_Power = handles.App.Peak_Power;
App.Cont_Power = handles.App.Cont_Power; 
App.Vehicle = handles.App.Vehicle;
App.Cycle_data = handles.App.Cycle_data;

handles.App = App;

for i = 1:length(Nt_vec)
    for j = 1:length(ka_vec)
            Nt = Nt_vec(i);
            ka = ka_vec(j);
                   
            Position = [ka Nt 1];
            [CostFunctionValue,EnergyLoss, Solution] = EvaluateDesign(EM,App,Position,handles);
            
            TotCost{i,j} = CostFunctionValue;
            PowertrainSolution{i,j} = Solution;

            if CostFunctionValue == 1e20
                EMCost(i,j) = nan;
                EnergyCost{i,j} = nan;
                BattCost{i,j} = nan;
                Gr{i,j} = nan;
            else
                EC = [EnergyLoss.Yearly_kWh];
                EMCost(i,j) = Solution.PowertrainCost.EM.tot;
                EnergyCost{i,j} = EC*App.El_price;
                BattCost{i,j} = VehRange.*(EC/AnualM)*1.2*BattPrice/BattDep;  % 7 is the deprciation year
                Gr{i,j} = Solution.Transmission;
            end
                            
    end
end
SteelPrice = str2num(handles.SteelPrice.String);
CopperPrice = str2num(handles.CopperPrice.String);
MagnetPrice = str2num(handles.MagnetPrice.String);



if size(EMCost,1)==1 && size(EMCost,2)==1 
    if ~isempty(Solution.PowertrainCost)
        handles.SteelMass.String = num2str(Solution.PowertrainCost.EM.steel/SteelPrice);
        handles.CopperMass.String = num2str(Solution.PowertrainCost.EM.copper/CopperPrice);
        handles.MagnetMass.String = num2str(Solution.PowertrainCost.EM.magnet/MagnetPrice);
    else
        handles.SteelMass.String = 'Design invalid';
        handles.CopperMass.String = 'Design invalid';
        handles.MagnetMass.String = 'Design invalid';
    end
else
end
      
% startingFolder = pwd;  % current folder
save('EMCOST.mat','EMCost')
save('EnergyCOST.mat','EnergyCost')
save('BattCOST.mat','BattCost')
save('GR.mat','Gr')

% arr = {EMCost};
% filename = 'Results.xlsx'
% data1 = EMCost
% data2 = EnergyCost
% data3 = Gr
% data4 = BattCost
% xlswrite(filename,data1,'Sheet1');
% xlswrite(filename,data2,'Sheet2');
% xlswrite(filename,data3,'Sheet3');
% xlswrite(filename,data4,'Sheet4');
% msgbox('Excel file created successfully', 'Success', 'modal');

% hold(handles.OptAxes,'off')
% surf(handles.OptAxes,x,y,z(1,:),'bd','LineWidth',2)
% hold(handles.OptAxes,'on')  
% title(handles.OptAxes, 'EM Cost vs Operating Cost [km/h]')

% hold(handles.OptAxes,'off')
% plot(handles.OptAxes,ka_vec,EMCost(1,:),'bd','LineWidth',2)
% hold(handles.OptAxes,'on')  
% title(handles.OptAxes, 'EM Cost vs Operating Cost [km/h]')




% axis(handles.ScalingAxes, [0 max()SEM.n_max 0 max(SEM.Tmax_lim)])




function Energy_Callback(hObject, eventdata, handles)
% hObject    handle to Energy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Energy as text
%        str2double(get(hObject,'String')) returns contents of Energy as a double


function Eprice_Callback(hObject, eventdata, handles)
% hObject    handle to Eprice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Eprice as text
%        str2double(get(hObject,'String')) returns contents of Eprice as a double


% --- Executes during object creation, after setting all properties.
function Eprice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Eprice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function AnualM_Callback(hObject, eventdata, handles)
% hObject    handle to AnualM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of AnualM as text
%        str2double(get(hObject,'String')) returns contents of AnualM as a double


% --- Executes during object creation, after setting all properties.
function AnualM_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AnualM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function BattDep_Callback(hObject, eventdata, handles)
% hObject    handle to BattDep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BattDep as text
%        str2double(get(hObject,'String')) returns contents of BattDep as a double


% --- Executes during object creation, after setting all properties.
function BattDep_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BattDep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function MaxPower_Callback(hObject, eventdata, handles)
% hObject    handle to MaxPower (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of MaxPower as text
%        str2double(get(hObject,'String')) returns contents of MaxPower as a double


% --- Executes during object creation, after setting all properties.
function MaxPower_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MaxPower (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function BaseSpeed_Callback(hObject, eventdata, handles)
% hObject    handle to BaseSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BaseSpeed as text
%        str2double(get(hObject,'String')) returns contents of BaseSpeed as a double


% --- Executes during object creation, after setting all properties.
function BaseSpeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BaseSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function MaxTorque_Callback(hObject, eventdata, handles)
% hObject    handle to MaxTorque (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of MaxTorque as text
%        str2double(get(hObject,'String')) returns contents of MaxTorque as a double


% --- Executes during object creation, after setting all properties.
function MaxTorque_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MaxTorque (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
