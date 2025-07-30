% Cargar datos
load('datos_fis_pos.mat');  % contiene: data_inputs (2113x2), data_outputs (2113x1)

% Normalización
max_in1 = max(abs(data_inputs(:,1)));
max_in2 = max(abs(data_inputs(:,2)));
max_out = max(abs(data_outputs));

if max_in1 == 0, max_in1 = 1; end
if max_in2 == 0, max_in2 = 1; end
if max_out == 0, max_out = 1; end

Xin = [data_inputs(:,1) / max_in1, ...
       data_inputs(:,2) / max_in2];     % (2113 x 2)

Xout = data_outputs / max_out;          % (2113 x 1)

% Configuración del FIS
options = genfisOptions('GridPartition');
options.NumMembershipFunctions = 7;
options.InputMembershipFunctionType = 'trimf';

% Generación del FIS
fis0 = genfis(Xin, Xout, options);

% Entrenamiento ANFIS
[fis_trained, ~, ~] = anfis([Xin Xout], fis0, 100);

% Guardar FIS entrenado
writefis(fis_trained, 'fis_pos_tracking.fis');
disp("✅ Guardado: fis_pos_tracking.fis");
