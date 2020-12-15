function varargout = RoboticArm3Dof(varargin)
% ROBOTICARM3DOF MATLAB code for RoboticArm3Dof.fig
%      ROBOTICARM3DOF, by itself, creates a new ROBOTICARM3DOF or raises the existing
%      singleton*.
%
%      H = ROBOTICARM3DOF returns the handle to a new ROBOTICARM3DOF or the handle to
%      the existing singleton*.
%
%      ROBOTICARM3DOF('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOTICARM3DOF.M with the given input arguments.
%
%      ROBOTICARM3DOF('Property','Value',...) creates a new ROBOTICARM3DOF or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before RoboticArm3Dof_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to RoboticArm3Dof_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help RoboticArm3Dof

% Last Modified by GUIDE v2.5 17-Aug-2018 13:20:15

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RoboticArm3Dof_OpeningFcn, ...
                   'gui_OutputFcn',  @RoboticArm3Dof_OutputFcn, ...
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


%% --- Funzione di apertura. viene cariata prima ancora che RoboticArm3Dof inizi
function RoboticArm3Dof_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to RoboticArm3Dof (see VARARGIN)
 global piConnected
 piConnected = false;
 
 global oldTheta;
 oldTheta = [ 0 0 0];
 
open('simulazioneBraccio');
% Choose default command line output for RoboticArm3Dof
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes RoboticArm3Dof wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%% funzione necessaria per la calibrazione iniziale dei motori
function calibraMotori() %----------------------------Calibrazione motori----------------------------------
global oldTheta;
global finecorsa;
global pi;
global MotoreSpalla;
global MotoreGomito;
global MotorePolso;
global stepAngle;

writeDigitalPin(pi,MotoreSpalla.('dir'),1);
writeDigitalPin(pi,MotoreGomito.('dir'),0);
writeDigitalPin(pi,MotorePolso.('dir'),0);


% --------------trovo un punto conosciuto per il polso--------------
while(readDigitalPin(pi,finecorsa(3))~=0)
    stepMotore(pi,MotorePolso.('step'));
end
disp('raggiunto fine polso');
pause(1);

writeDigitalPin(pi,MotorePolso.('dir'),1);
for i=0:int16(55/stepAngle) 
    stepMotore(pi,MotorePolso.('step'));
end

pause(2);

%--------------trovo un punto conosciuto per il gomito--------------
while(readDigitalPin(pi,finecorsa(2))~=0)
    stepMotore(pi,MotoreGomito.('step'));
end
disp('raggiunto fine gomito');
pause(1);

writeDigitalPin(pi,MotoreGomito.('dir'),1);
for i=0:int16(65/stepAngle) 
    stepMotore(pi,MotoreGomito.('step'));
end
for i=0:int16(90/stepAngle) 
    stepMotore(pi,MotorePolso.('step'));
end

pause(2);



%------trovo un punto conosciuto della struttura di base - spalla----------
while(readDigitalPin(pi,finecorsa(1))~=0)
    stepMotore(pi,MotoreSpalla.('step'));
end
disp('raggiunto fine base');
pause(1)
%riporto la base alla posizione 0
writeDigitalPin(pi,MotoreSpalla.('dir'),0);
for i=0:int16(180/stepAngle) 
    stepMotore(pi,MotoreSpalla.('step'));
end

disp('fine calibrazione');

oldTheta = [0 0 0]; % posizione iniziale dei motori

settaAngoliSimulazione(0,0,0);



% --- Outputs from this function are returned to the command line.
function varargout = RoboticArm3Dof_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



%% --- Sposta i motori verso le cordinate XYZ descritte nello stesso pannello
function SpostaButton_Callback(hObject, eventdata, handles)
% hObject    handle to SpostaButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global a2;
global a3;
global oldTheta;
global piConnected;
a2 = 10;
a3 = 12;

%prendo i valori delle entry
newX = get(handles.CordinataX,'String');
newY = get(handles.CordinataY,'String');
newZ = get(handles.CordinataZ,'String');

newPoint = [ str2double(newX)  str2double(newY)  -str2double(newZ)]; %scrive il nuovo punto in cui spostarsi

checkNAN = isnan(newPoint); % serve a determinare se nel nuovo punto ci sono solo numeri

%controllo e scrivo i valori nel nuovo punto da raggiungere
if(not(piConnected))
    
    set(handles.errText,'String','ERROR: Connessione non stabilita -> Devi connettere la raspberry al computer per poter impostare dei valori');

elseif checkNAN==0 % determina se sono tutti numeri
    set(handles.errText,'String', 'Error Text'); 
    
   %CINEMATICA INVERSA
   try
        c3= (newPoint(1)^2 + newPoint(2)^2 + newPoint(3)^2 - a2^2 - a3^2)/(2*a2*a3);
        s3= sqrt(1 - c3^2);
        s2=((a2+a3*c3)*newPoint(3) - a3*s3*sqrt(newPoint(1)^2 + newPoint(2)^2))/(newPoint(1)^2 + newPoint(2)^2 + newPoint(3)^2);
        c2=((a2+a3*c3)* sqrt(newPoint(1)^2 + newPoint(2)^2) + a3 * s3 *newPoint(3))/(newPoint(1)^2 + newPoint(2)^2 + newPoint(3)^2);

        theta1Radians = atan2(newPoint(2),newPoint(1));
        theta3Radians = atan2(s3,c3);
        theta2Radians = atan2(s2,c2);

        theta1Deg = rad2deg(theta1Radians);
        theta2Deg = rad2deg(theta2Radians);
        theta3Deg = rad2deg(theta3Radians);

        newTheta = [ theta1Deg theta2Deg theta3Deg ];
        
        differenzaTheta = newTheta - oldTheta;
        muoviMotori(differenzaTheta);

        %stampo i valori corrente sull'interfaccia grafica
        set(handles.teorTheta1,'String', newTheta(1));
        set(handles.teorTheta2,'String', newTheta(2));
        set(handles.teorTheta3,'String', newTheta(3));

        set(handles.teorX,'String', newPoint(1));
        set(handles.teorY,'String', newPoint(2));
        set(handles.teorZ,'String', newPoint(3));
        
        settaCurrentValues(handles, differenzaTheta);
        
        settaErr(handles);
        
        %cambia la simulazione per vedere se il punto nello spazio
        %corrisponde a quello reale
        
        settaAngoliSimulazione(theta1Radians,theta2Radians,theta3Radians);
        
    catch 
        
        errorMsgText = 'ERROR: punto irraggiungibile -> punto fuori dallo spazio di lavoro del braccio';
        set(handles.errText, 'String', errorMsgText);
    end
    
else
    set(handles.errText,'String','ERROR: valore non numerico -> devi mettere valori numerici entro le caselle: X Y Z');
end


%% --- Imposta i valori dei theta a quelli decritti e sposta il motore
% verso quel punto
function ImpostaButton_Callback(hObject, eventdata, handles)
% hObject    handle to ImpostaButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global oldTheta;
global piConnected;

newTheta1 = get(handles.Theta1edit, 'String');
newTheta2 = get(handles.Theta2edit, 'String');
newTheta3 = get(handles.Theta3edit, 'String');

newTheta = [ str2double(newTheta1)  str2double(newTheta2)  str2double(newTheta3)]; %scrive il nuovo punto in cui spostarsi

checkNAN = isnan(newTheta); % controlla se nella conversione dei numeri c'è un NotANumber ovvero non è un numero

if(not(piConnected))
    
    set(handles.errText,'String','ERROR: Connessione non stabilita -> Devi connettere la raspberry al computer per poter impostare dei valori');

elseif  checkNAN==0  % determina se sono tutti numeri
    checkSpace = newTheta(1)>=-180 && newTheta(1)<= 180 && newTheta(2)>=-225 && newTheta(2)<= 45 && newTheta(3)>=-135 && newTheta(3)<= 135; % controlla chje gli angoli sianop fra 360 e -360
    if checkSpace
        set(handles.errText,'String', 'Error Text'); 

       thetaDifference = newTheta - oldTheta;
       muoviMotori(thetaDifference); % fa muovere i morori grazie alla differenza degli angoli

        %stampo il valore corrente sull'interfaccia grafica
        set(handles.teorTheta1,'String', newTheta(1));
        set(handles.teorTheta2,'String', newTheta(2));
        set(handles.teorTheta3,'String', newTheta(3));
        
        %setta gli angoli della simulaizione del braccio
        settaAngoliSimulazione(newTheta(1)*2*pi/360, newTheta(2)*2*pi/360,newTheta(3)*2*pi/360);
        
        %scrive i valori del punto raggiunto in teoria sulla gui
        settaPuntoXYZ(handles,newTheta,'teor')
        
        %setta i valori correnti sulla gui
        settaCurrentValues(handles,thetaDifference);
        
        %setta i valori degli errori sulla gui
        settaErr(handles);
        
    else
        set(handles.errText,'String','ERROR: Bound superato -> il valore dei theta deve essere compreso fra i -180 e 180 gradi per theta1, -225 e 45 per theta2 e -90 e 90 per theta3.');
    end
else
    set(handles.errText,'String','ERROR: valore non numerico -> devi mettere valori numerici entro le caselle: Theta1 Theta2 Theta3');
end

%% scrive i valori correnti degli angoli correnti sulla gui
function settaCurrentValues(handles,thetaDifference)
global stepAngle;
global oldTheta;

%calcolo numero degli step
numStep = int16(thetaDifference/stepAngle);

% trovo gli angoli finali correnti
finalTheta = oldTheta + (double(numStep)*stepAngle);
disp(numStep);
disp(finalTheta);
% setto i valori degli angoli su schermo
set(handles.currTheta1,'String',finalTheta(1));
set(handles.currTheta2,'String',finalTheta(2));
set(handles.currTheta3,'String',finalTheta(3));

% aggiorno il valore di old theta
oldTheta = finalTheta;
%infine calcolo i valori finali correnti dei punti
settaPuntoXYZ(handles, finalTheta,'curr')

%% setta valori correnti o teorici sulla gui in base al tipo passato
function settaPuntoXYZ(handles,theta, tipo)
global a2;
global a3;

theta = theta * 2*pi /360;

x = cos(theta(1))*(a2*cos(theta(2)) + a3 *cos(theta(2)+theta(3)));
y = sin(theta(1))*(a2*cos(theta(2)) + a3 *cos(theta(2)+theta(3)));
z = a2* sin(theta(2)) + a3 * sin(theta(2)+theta(3));

if(strcmp(tipo,'teor'))
    
    set(handles.teorX,'String', x);
    set(handles.teorY,'String', y);
    set(handles.teorZ,'String', z);
    
elseif(strcmp(tipo,'curr'))
    
    set(handles.currX,'String', x);
    set(handles.currY,'String', y);
    set(handles.currZ,'String', z);
    
else
    set(handles.errText, 'String', 'ERRORE: tipo non specificato -> la tipologia di dato potrebbe non essere stata specificata correttamente nel codice o potrebbe essere assente');    
end

%% setta gli errori sulla gui
function settaErr(handles)

    % prendo i valori correnti da quelli stampati su schermo
    currTheta = [str2double(get(handles.currTheta1,'String')) str2double(get(handles.currTheta2,'String')) str2double(get(handles.currTheta3,'String'))];
    currXYZ = [str2double(get(handles.currX,'String')) str2double(get(handles.currY,'String')) str2double(get(handles.currZ,'String'))];
    teorTheta = [str2double(get(handles.teorTheta1,'String')) str2double(get(handles.teorTheta2,'String')) str2double(get(handles.teorTheta3,'String'))];
    teorXYZ = [str2double(get(handles.teorX,'String')) str2double(get(handles.teorY,'String')) str2double(get(handles.teorZ,'String'))];
    
    %calcolo la differenza (il valore di riferimento è quello teorico, ovviamente)
    finalErrTheta = currTheta - teorTheta;
    finalErrXYZ = currXYZ - teorXYZ;
    
    %stampo gli errori su schermo
    set(handles.errTheta1,'String',finalErrTheta(1));
    set(handles.errTheta2,'String',finalErrTheta(2));
    set(handles.errTheta3,'String',finalErrTheta(3));
    set(handles.errX,'String',finalErrXYZ(1));
    set(handles.errY,'String',finalErrXYZ(2));
    set(handles.errZ,'String',finalErrXYZ(3));
    
    %scrivo i dati degli errori su due file per le statistiche
    
    %errori di theta 
    fidTheta = fopen('thetaErrors.txt','a+');
    fprintf(fidTheta,'%3.4f\t%3.4f\t%3.4f\n',finalErrTheta(1),finalErrTheta(2),finalErrTheta(3));
    fclose(fidTheta);
    
    %errori di XYZ
    fidXYZ = fopen('xyzErrors.txt','a+');
    fprintf(fidXYZ,'%3.4f\t%3.4f\t%3.4f\n',finalErrXYZ(1),finalErrXYZ(2),finalErrXYZ(3));
    fclose(fidXYZ);
    
%% setta gli angoli del braccio che sta sulla simulazione grazie agli angoli in radianti precedentemente scelti
function settaAngoliSimulazione(theta1Radians,theta2Radians,theta3Radians)
        set_param('simulazioneBraccio/theta1','Value',num2str(theta1Radians));
        set_param('simulazioneBraccio/theta2','Value',num2str(theta2Radians));
        set_param('simulazioneBraccio/theta3','Value',num2str(theta3Radians));
        
        sim('simulazioneBraccio');
   
%% muove i motori grazie al calcolo della differenza fra angoli correnti e
% nuovi
function muoviMotori(thetaDifference)
global pi;
global MotoreSpalla;
global MotoreGomito;
global MotorePolso;
global stepAngle;


numStep = int16(thetaDifference / stepAngle);

% ---------impostazione direzione motore---------
% MOTORE BASE
if numStep(1)>=0
    writeDigitalPin(pi,MotoreSpalla.('dir'),1);
else
    numStep(1) = -numStep(1);
    writeDigitalPin(pi,MotoreSpalla.('dir'),0);
end

% MOTORE GOMITO
if numStep(2)>=0
    writeDigitalPin(pi,MotoreGomito.('dir'),0);
else
    numStep(2) = -numStep(2);
    writeDigitalPin(pi,MotoreGomito.('dir'),1);
end

% MOTORE POLSO
if numStep(3)>=0
    writeDigitalPin(pi,MotorePolso.('dir'),1);
else
    numStep(3) = -numStep(3);
    writeDigitalPin(pi,MotorePolso.('dir'),0);
end
pause(1);
% % -----Spostamento motori non contemporaneo-----
% %spostamento motore
%  for i=0:numStep(1)
%      stepMotore(pi,MotoreSpalla.('step'));
%  end
% 
% %spostamento motore
% for i=0:numStep(2)
%     stepMotore(pi,MotoreGomito.('step'));
% end
% 
% %spostamento motore
% for i=0:numStep(3)
%     stepMotore(pi,MotorePolso.('step'));
% end

% ------movimento dei motori in contemporanea---------
while (numStep(1)~=0 || numStep(2)~=0 || numStep(3) ~= 0)
    if numStep(1) ~= 0
        stepMotore(pi,MotoreSpalla.('step'));
        numStep(1)= numStep(1)-1;
    end
    if numStep(2) ~= 0
        stepMotore(pi,MotoreGomito.('step'));
        numStep(2) = numStep(2)-1;
    end
    if numStep(3) ~= 0
        stepMotore(pi,MotorePolso.('step'));
        numStep(3) = numStep(3)-1;
    end
    disp(numStep)
end


%% manda un impulso al driver A4998 che equivale ad uno step del motore
function stepMotore(pi,pinStep)
   global delay
   
   writeDigitalPin(pi,pinStep,1);
   pause(delay/12);
   writeDigitalPin(pi,pinStep,0);
   pause(delay/12);
   
   

%% --- Permette la connessione fra raspberry e computer
function RaspConnection_Callback(hObject, eventdata, handles)
% hObject    handle to RaspConnection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clear pi;

global piConnected; % variabile d'appoggio per controllare se la raspberry è connessa
global MS; % Array per decidere la modalità di stepping dello stepper motor
global finecorsa; % array contenente i pin di collegamento dei sensori di finecorsa necessari alla calibrazione dei motori

global MotoreSpalla;
global MotoreGomito;
global MotorePolso;

global pi; % connessione alla raspberry
global delay; %ritardo per step ovvero per creare l'impulso
global stepAngle; % angolo di giro
global a2; %lunghezza del secondo braccio
global a3; % lunghezza del terzo braccio

%Connessione alla Raspberry pi 3

set(handles.ConnectionText, 'String', 'Not Connected');
set(handles.ConnectionText, 'ForegroundColor', 'White');

if(not(piConnected))
    try
        pi = raspi('169.254.186.98','pi','raspberry');
        set(handles.ConnectionText, 'String', 'Connection Established');
        set(handles.ConnectionText, 'ForegroundColor', 'Green');
        set(handles.errText, 'String', 'No Errors');

        
        % Dichiarazione variabili/pin di collegamento
        MotoreSpalla.('dir') = 20;
        MotoreSpalla.('step') = 21;

        MotoreGomito.('dir') = 23;
        MotoreGomito.('step') = 24;

        MotorePolso.('dir') = 5;
        MotorePolso.('step') = 6;
        
        finecorsa = [25 26 27];

        MS = [14 15 18];

        a2 = 10;
        a3 = 12;

        % configurazione dei pin della raspberry
        configurePin(pi,MotoreSpalla.('dir'),'DigitalOutput');
        configurePin(pi,MotoreSpalla.('step'),'DigitalOutput');

        configurePin(pi,MotoreGomito.('dir'),'DigitalOutput');
        configurePin(pi,MotoreGomito.('step'),'DigitalOutput');

        configurePin(pi,MotorePolso.('dir'),'DigitalOutput');
        configurePin(pi,MotorePolso.('step'),'DigitalOutput');
        
        %configurazione dei pin di fine corsa
        configurePin(pi,finecorsa(1),'DigitalInput');
        configurePin(pi,finecorsa(2),'DigitalInput');
        configurePin(pi,finecorsa(3),'DigitalInput');
        
        % configurazione pin dell'angolo di passo
        configurePin(pi,MS(1),'DigitalOutput');
        configurePin(pi,MS(2),'DigitalOutput');
        configurePin(pi,MS(3),'DigitalOutput');
        
        
        % impostazione dell'angolo di passo - in questo caso (0 0 0) è in
        % full step 
        writeDigitalPin(pi,MS(1),1);
        writeDigitalPin(pi,MS(2),1);
        writeDigitalPin(pi,MS(3),0);
        stepAngle = 1.8/8;
        delay = 3/(200*1.8/stepAngle);
        
        piConnected = true; 
    catch exception

        set(handles.ConnectionText, 'String', 'Connection NOT Established');
        set(handles.ConnectionText, 'ForegroundColor', 'Red');
        piConnected = false; 

        errorMsgText = getReport(exception, 'basic');
        set(handles.errText, 'String', errorMsgText);

    end
else
    set(handles.errText, 'String', 'ERROR: Connnessione già stabilita -> la connessione con il dispositivo è gia avvenuta');
end

%% richiamata quando viene premuto il pulsante di calibrazione dei motori
function CalibraMotoriButton_Callback(hObject, eventdata, handles)
% hObject    handle to CalibraMotoriButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global piConnected;

if(piConnected)
    calibraMotori();  
else
    set(handles.errText,'String','ERROR: Raspberry non connessa -> bisogna connettere la raspberry per poter effettuare la calibrazione dei motori.');
end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%FUNZIONI NON
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%UTILIZZATE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function Theta1edit_Callback(hObject, eventdata, handles)
% hObject    handle to Theta1edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta1edit as text
%        str2double(get(hObject,'String')) returns contents of Theta1edit as a double


% --- Executes during object creation, after setting all properties.
function Theta1edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta1edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta2edit_Callback(hObject, eventdata, handles)
% hObject    handle to Theta2edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta2edit as text
%        str2double(get(hObject,'String')) returns contents of Theta2edit as a double


% --- Executes during object creation, after setting all properties.
function Theta2edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta2edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta3edit_Callback(hObject, eventdata, handles)
% hObject    handle to Theta3edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta3edit as text
%        str2double(get(hObject,'String')) returns contents of Theta3edit as a double


% --- Executes during object creation, after setting all properties.
function Theta3edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta3edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function CordinataX_Callback(hObject, eventdata, handles)
% hObject    handle to CordinataX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CordinataX as text
%        str2double(get(hObject,'String')) returns contents of CordinataX as a double


% --- Executes during object creation, after setting all properties.
function CordinataX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CordinataX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function CordinataY_Callback(hObject, eventdata, handles)
% hObject    handle to CordinataY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CordinataY as text
%        str2double(get(hObject,'String')) returns contents of CordinataY as a double


% --- Executes during object creation, after setting all properties.
function CordinataY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CordinataY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function CordinataZ_Callback(hObject, eventdata, handles)
% hObject    handle to CordinataZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CordinataZ as text
%        str2double(get(hObject,'String')) returns contents of CordinataZ as a double


% --- Executes during object creation, after setting all properties.
function CordinataZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CordinataZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



