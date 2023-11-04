clc
clear

GuiStart();

function GuiStart()
%% graficzny panel użytkownika
win = uifigure("Name", "Silnik_obco_ss.exe", 'NumberTitle','off', "WindowState", "fullscreen");
panel = uipanel(win, "Title", "Parametry silnika obcowzbudnego:", "Position", [30, 500, 700, 400]);
%% widgety
        % numer widgety
uilabel(panel, "Text", "Liczba porządkowa", "Position", [50, 350, 150, 20]);
num = uieditfield(panel, "numeric", "Position", [200, 350, 150, 20], "Value", 5);

        % napięcie widgety
uilabel(panel, "Text", "Napięcie znamionowe", "Position", [50, 300, 150, 20]);
uilabel(panel, "Text", "[V]", "Position", [370, 300, 150, 20]);
voltage = uieditfield(panel, "numeric", "Position", [200, 300, 150, 20], "Value", 440);     

        % prąd widgety
uilabel(panel, "Text", "Prąd znamionowy", "Position", [50, 250, 150, 20]);
uilabel(panel, "Text", "[A]", "Position", [370, 250, 150, 20]);
current = uieditfield(panel, "numeric", "Position", [200, 250, 150, 20], "Value", 56.2);

        % RPM widgety
uilabel(panel, "Text", "RPM", "Position", [50, 200, 150, 20]);
uilabel(panel, "Text", "[min^-1]", "Position", [370, 200, 150, 20]);
rpm = uieditfield(panel, "numeric", "Position", [200, 200, 150, 20], "Value", 1500);

        % moment bezwładności widgety
uilabel(panel, "Text", "Moment bezwładności", "Position", [50, 150, 250, 20]);
uilabel(panel, "Text", "[kg*m^2]", "Position", [370, 150, 70, 20]);
momentum = uieditfield(panel, "numeric", "Position", [200, 150, 150, 20], "Value", 2.7);

        % rezystacja uzwojenia widgety
uilabel(panel, "Text", "Rezystancja uzwojenia", "Position", [50, 100, 150, 20]);
uilabel(panel, "Text", "[\Omega]", "Position", [370, 100, 50, 20]);
resistance = uieditfield(panel, "numeric", "Position", [200, 100, 150, 20], "Value", 0.465);

        % stała czasowa elektryczna widgety
uilabel(panel, "Text", "Elektryczna stała czasowa", "Position", [50, 50, 250, 20]);
uilabel(panel, "Text", "[s]", "Position", [370, 50, 50, 20]);
te_ = uieditfield(panel, "numeric", "Position", [200, 50, 150, 20], "Value", 0.003); 

        % przyciski
uibutton(panel, "Text", "Dane", "Position", [250, 10, 60, 30], "ButtonPushedFcn", @(btn, event) scan(win, num.Value, voltage.Value, current.Value, rpm.Value, momentum.Value, resistance.Value, te_.Value));
uibutton(panel, "Text", "Wyjście", "Position", [350, 10, 60, 30], "ButtonPushedFcn", @(btn, event) exit(win));          
end

function scan(win, Y, voltage, current, rpm, momentum, resistance, te)
    % pole placement (zmiana wartości biegunów) 
prompt = {'biegun nr 1','biegun nr 2', 'biegun nr 3'};
dlgtitle = 'Dane biegunów';
dims = [1 40];
definput = {'-15-5j', '-15+5j', '-330'};
answer = inputdlg(prompt, dlgtitle, dims, definput);
   
f = waitbar(0, "Title", "Silnik_obco_wzbudny.exe", 'Uruchamianie algorytmu', "WindowStyle", "alwaysontop");
pause(0.5);
waitbar(0, f, 'Czekaj.');
pause(0.5);
waitbar(0, f, 'Czekaj..');
pause(0.5);
waitbar(0, f, 'Czekaj...');
waitbar(.05, f, 'Obliczanie parametrów');

        % zmienne obliczone  
cem =  (voltage - resistance * current) / (rpm * pi / 30) ;
tm = (resistance * momentum) / cem^2;

    % macierzy
waitbar(.10, f, 'Obliczanie macierzy');
A = [(-1/te) , (-(momentum/(tm*te*cem))) ; (resistance/(tm*cem)) , 0];
B = [(1/(te*resistance)); 0];
C = [0 , 1];

    % model przestrzeni stanu
waitbar(.15, f, 'Budowa modelu przestrzeni stanu');
pause(0.5);
disp("model stanowy silnika DC");
motor = ss(A, B, C, 0, "InputName", "v", "OutputName", "y", "StateName", {'i', 'w'});
display(motor);

    % transmitancja
waitbar(.20, f, 'Wyznaczanie transmitacji operatorowej');
pause(0.5);
disp("Transmitacja operatorowa silnika");
motor_TF = tf(motor);
display(motor_TF);

    % 2.2 Wzmacniacz PWM
waitbar(.25, f, 'Skręcanie wzmacniacza');
pause(0.5);
amp = tf((5.5+Y/10) , [0.001, 1]); % wzmacniacz PWM
amplified = dcgain((motor * amp) / motor);
    
    % 2.3
plant = motor * amp;
plant.StateName{1} = 'i';
plant.StateName{2} = 'w';
plant.StateName{3} = 'x_3';

    % 2.4
waitbar(.30, f, 'Wyznaczanie modelu przestrzeni stanu dla układu otwartego silnika ze wzmacniaczem');
pause(0.5);
disp("model przestrzeni stanu silnika ze wzmacniaczem PWM");
display(plant)

    % 2.5 sprawdzenie warunku sterowalności silnik ze wzmacniczem
ctrb_system = ctrb(plant);
rank(ctrb_system);

plant.C = eye(3);

waitbar(.35, f, 'Wyznaczanie macierzy wzmocnienia');
pause(0.5);
    % macierz wzmocnień
K = place(plant.a, plant.b, [str2double(answer{1}), str2double(answer{2}), str2double(answer{3})]);
back = feedback(plant, K);
disp("model przestrzeni stanu dla silnika ze wzmacniczem PWM w pętli sprzężenia zwrotnego");
display(back);
disp("Macierz wzmocnienia");
display(K);

waitbar(.40, f, 'Budowa prekompensatora');
pause(0.5);
    % prekompensator
N = dcgain(back(2));
plant_gained = back(2)/N;
disp("model przestrzeni stanu silnika ze wzmacniaczem PWM w pętli sprzężenia zwrotnego i prekompensatorem");
display(plant_gained);

%% wyniki
waitbar(.45, f, 'Rezerwacja miejsca w pamięci na dane');
pause(0.5);
results = eig(motor);
plane = uipanel(win, "Title", "wyniki równania charakternego dla macierzy A:", "Position", [30, 400, 400, 80]);
uilabel(plane, "Text", "x_1 = " + results(1), "Position", [10, 40, 280, 20]);
uilabel(plane, "Text", "x_2 = " + results(2), "Position", [10, 10, 280, 20]);

    % dodatkowe parametry 
plane = uipanel(win, "Title", "parametry PWM i prekompensatora:", "Position", [330, 400, 400, 80]);
uilabel(plane, "Text", "wzmocnienie K = " + amplified, "Position", [10, 40, 280, 20]);
uilabel(plane, "Text", "krotność kompensacji N = " + N, "Position", [10, 10, 280, 20]);

    % stabilność i bieguny silnika w układzie otwartym
motor_poles = pole(motor);
if isstable(motor) == 1
    motor_stability = "stabilny";
else
    motor_stability = "nie stabilny";
end
plane_poles = uipanel(win, "Title", "silnik w układzie otwartym:", "Position", [30, 300, 400, 100]);
uilabel(plane_poles, "Text", "stabilność: " + motor_stability, "Position", [10, 60, 280, 20]);
uilabel(plane_poles, "Text", "biegun nr 1: " + motor_poles(1), "Position", [10, 40, 280, 20]);
uilabel(plane_poles, "Text", "biegun nr 2: " + num2str(motor_poles(2)), "Position", [10, 20, 280, 20]);
  
    % stabilność i bieguny silnika w układzie otwartym ze wzmacniaczem PWM
plant_poles = pole(plant);
if isstable(plant) == 1
    plant_stability = "stabilny";
else
    plant_stability = "nie stabilny";
end
plane_poles = uipanel(win, "Title", "silnik w układzie otwartym ze wzmacniaczem PWM:", "Position", [330, 300, 400, 100]);
uilabel(plane_poles, "Text", "stabilność: " + plant_stability, "Position", [10, 60, 280, 20]);
uilabel(plane_poles, "Text", "biegun nr 1: " + plant_poles(1), "Position", [10, 40, 280, 20]);
uilabel(plane_poles, "Text", "biegun nr 2: " + num2str(plant_poles(2)), "Position", [10, 20, 280, 20]);
uilabel(plane_poles, "Text", "biegun nr 3: " + num2str(plant_poles(3)), "Position", [10, 0, 280, 20]);

        % stabilność i bieguny silnika w pętli sprzężenia zwrotnego
back_poles = pole(back);
if isstable(back) == 1
    back_stability = "stabilny";
else
    back_stability = "nie stabilny";
end
back_plane = uipanel(win, "Title", "silnik w układzie ze sprzężeniem zwrotnym:", "Position", [30, 200, 400, 100]);
uilabel(back_plane, "Text", "stabilność: " + back_stability, "Position", [10, 60, 280, 20]);
uilabel(back_plane, "Text", "biegun nr 1: " + back_poles(1), "Position", [10, 40, 280, 20]);
uilabel(back_plane, "Text", "biegun nr 2: " + num2str(back_poles(2)), "Position", [10, 20, 280, 20]);
uilabel(back_plane, "Text", "biegun nr 3: " + num2str(back_poles(3)), "Position", [10, 0, 280, 20]);

    % stabilność i bieguny silnika w układzie sprzężenia zwrotnego z prekompensatorem
plant_gained_poles = pole(plant_gained);
if isstable(plant_gained) == 1
    plant_gained_stability = "stabilny";
else
    plant_gained_stability = "nie stabilny";
end
plane_gained_poles = uipanel(win, "Title", "silnik w układzie pętli sprzężenia zwrotnego z prekompensatorem:", "Position", [330, 200, 400, 100]);
uilabel(plane_gained_poles, "Text", "stabilność: " + plant_gained_stability, "Position", [10, 60, 280, 20]);
uilabel(plane_gained_poles, "Text", "biegun nr 1: " + plant_gained_poles(1), "Position", [10, 40, 280, 20]);
uilabel(plane_gained_poles, "Text", "biegun nr 2: " + num2str(plant_gained_poles(2)), "Position", [10, 20, 280, 20]);
uilabel(plane_gained_poles, "Text", "biegun nr 3: " + num2str(plant_gained_poles(3)), "Position", [10, 0, 280, 20]);

%% Złożenie przebiegów
waitbar(.50, f, 'Operacje na stosie');
pause(0.5);
figure("Name", "Przebiegi dla poszczególnych układów", 'NumberTitle','off',  "WindowState", "maximized");
    % przebieg silnika
subplot(2, 2, 1);
waitbar(.55, f, 'Załadowanie silnika');
pause(0.5);
step(motor);
grid("on");
title("Przebieg dla silnika w układzie otwartym");
    % przebieg silnika ze wzmacniaczem
subplot(2, 2, 2);
waitbar(.60, f, 'Włączanie wzmacniacza');
pause(0.5);
step(plant(2));
grid("on");
title("Przebieg dla silnika ze wzmacniaczem PWM w układzie otwartym");
    % przebieg silnika ze wzmacniaczem i przesunięciem biegunów
subplot(2, 2, 3);
waitbar(.65, f, 'Sprzęganie pętli');
pause(0.5);
step(back(2));
grid("on");
title("Przebieg dla silnika ze wzmacniaczem PWM w pętli sprzężenia zwrotnego");
    % przebieg silnika ze wzmacniaczem, przesunięciem biegunów i przekompensatorem
subplot(2, 2, 4);
waitbar(.70, f, 'Prekompensacja sygnału');
pause(0.5);
step(plant_gained(1));
grid("on");
title("Przebieg dla silnika ze wzmacniaczem PWM w pętli sprzężenia zwrotnego i przekompensatorem");

%% Porównanie przebiegów
waitbar(.75, f, 'Stawianie wykresów');
pause(0.5);
figure("Name", "Porównanie przebiegów", 'NumberTitle','off', "WindowState", "maximized")
step(motor);
hold("on");
waitbar(.80, f, 'Segregacja danych');
pause(0.5);
step(plant(2));
hold("on");
waitbar(.85, f, 'Redukcja danych');
pause(0.5);
step(back(2));
hold("on");
waitbar(.90, f, 'Łączenie wykresów');
pause(0.5);
step(plant_gained(1));
legend("silnik w układzie otwartym", " silnik układ otwarty z PWM", "z pętlą sprzężenia zwrotnego", "pętla sprzężenia z prekompensatorem");
grid("on");
xlabel("czas [s]");
ylabel("amplituda [%]");

%% Wykresy biegunów
waitbar(.92, f, 'Skanowanie danych');
figure("Name", "Wykresy biegunów", 'NumberTitle','off', "WindowState", "maximized")
    % bieguny silnika w układzie otwartym
subplot(2, 2, 1);
waitbar(.94, f, 'Poszukiwanie biegunów');
rlocus(motor);
title("bieguny silnika w układzie otwartym");

    % bieguny silnika ze wzmacniaczem w układzie otwarym
subplot(2, 2, 2);
waitbar(.96, f, 'Wyznaczanie biegunów');
pause(0.5);
rlocus(plant(2));
title("bieguny silnika ze wzmacniaczem PWM w układzie otwartym");

    % bieguny silnika ze wzmacniaczem w pętli sprzężenia zwrotnego
subplot(2, 2, 3);
rlocus(back(2));
waitbar(.98, f, 'Zgrywanie obrazu');
pause(0.5);
title("bieguny silnika ze wzmacniaczem PWM w pętli sprzężenia zwrotnego");

    % bieguny silnika ze wzmacniczem w pętli sprzężenia zwrotnego z prekompensatorem
subplot(2, 2, 4);
rlocus(plant_gained(1));
title("bieguny silnika ze wzmacniaczem PWM w pętli sprzężenia zwrotnego i prekompesatorem");
    % przebiegi poranawcze skokowe i bode
figure("Name", "Porównanie przebiegów", 'NumberTitle','off', "WindowState", "maximized")
subplot(2, 2, 1);
step(motor);
title("Wykres odpowiedzi skokowej silnika");
grid("on");
subplot(2, 2, 2);
bode(motor);
title("Wykresy Bodego silnika");
grid("on");
subplot(2, 2, 3);
step(plant_gained(1));
title("Wykres odpowiedzi skokowej silnika z regulatorem i prekompensatorem");
grid("on");
subplot(2, 2, 4);
bode(plant_gained(1));
title("Wykresy Bodego silnika z regulatorem i prekomensatorem");
grid("on");
waitbar(1, f, 'Koniec psot'); 
pause(0.5);
    % koniec psot
close(f);

end

function exit(win)
    delete(win)
    clc
    clear GuiStart 
    close all
end
