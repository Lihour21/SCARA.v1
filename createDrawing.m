function ...
    createDrawing( R1, R2, R3, k, archive, archiveFlag, archiveSal, xman, yman )
% 5R Robot path from graphics
%
% Inputs:
%   R1, input link length
%   R2, coupler link length
%   R3, fixed link half length 
%   k, scale
%   'file', data file (.mat), graphic (.png), or data (.txt) to be
%       processed. If .mat, then it is necessary to define two variables:
%       'X' and 'Y'. If .txt, then it is necessary to define two columns
%       corresponding with 'X' and 'Y' coordinates
%   fileFlag,
%       1 for .mat
%       2 for png, points are selected using the pointer. Process is closed
%           with "return"
%       3 for .txt
%   'outputFile', file name, without extension, of the output to be loaded
%       in PanelRobot. Output is .txt file 
% Optional
%   xman, x-axis offset 
%   yman, y-axis offset
%
%
% Note: Requires the 'fiveRmic' function
% Note: Requires the 'circles' function by Chad Greene (2014). University 
%   of Texas Institute for Geophysics. Available in MatLab Central: File 
%   exchange.

%% Manual offset
switch nargin
    case 7
        xman = 0; yman = 0;
    case 9
    otherwise
        return
end


%% MIC analysis
% non-dimensional factor and non-dimensional variables
D = (R1 + R2 + R3) / 3;
r1 = R1/D; r2 = R2/D; r3 = R3/D;
% MIC analysis
[ rmic, ymic ] = fiveRmic( r1, r2, r3);
% dimensional variabl es
Rmic = rmic*D;
Xmic = R3;
Ymic = ymic*D;

%% Figure development
switch archiveFlag
    case 1 % .mat
        load (archive,'X','Y')
    case 2
        figure
        grafico = imread(archive);
        image(grafico)
        grid on
        axis equal
        [ X, Y ] = ginput;
        Y = -Y;
    case 3
        Datos = load(archive);
        X = Datos(:,1);
        Y = Datos(:,2);
    otherwise
        return
end

%% Displacement and scale
x = k*X'; centroidex = sum(x) / length(x);
y = k*Y'; centroidey = sum(y) / length(y);
x = x + Xmic - centroidex + xman;
y = y + Ymic - centroidey + yman;

%% Reference graphic
plot(x,y,'-b')
hold on 
circles(Xmic,Ymic,Rmic,'facecolor','none')
axis equal
set(gca,'Fontsize',12)

%% .txt file writing
fileID = serialport(strcat(archiveSal,'.txt'),'w');
formatSpec = '%6.3f  %6.3f\n';
col = size(x);
for i = 1:col(2)
    fprintf(fileID,formatSpec, x(i), y(i));
end
fclose(fileID);
end