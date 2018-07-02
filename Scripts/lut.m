% Computes look up tables for crank angles, foot extensions, etc. from the 
% femur angle.  Calibrated for each robot.
% NOTE: there must be as many "crank", "MA, and "extension" entries as 
% there  are entries for "names".

names = {'SALTO_1P_RUDOLPH','SALTO_1P_DASHER','UNUSED'};

fmax = pi/2; % maximum femur deflection for LUT
resolution = 2^8;
x = linspace(0,fmax,resolution); % femur sample points [rad]

% crank function outputs are in radians
crank = [x.*(-5.545933486743863e-1)+x.^2.*4.861385182590551-x.^3.*3.076529866354905+...
    x.^4.*1.110905627213665-x.^5.*6.960572009953411e-2+4.410270509582694e-2; ... % Rudolph
    x.*(-2.219771122747685e-1)+x.^2.*3.934037595625176-x.^3.*2.017414906463222+...
    x.^4.*1.677073832029953e-1+x.^5.*2.253083088706445e-1+1.883563826690602e-1; % Dasher
    2.003.*x.^5.0 - 7.434.*x.^4.0 + 9.429.*x.^3.0 - 2.691.*x.^2.0 + 0.3893.*x - 0.001175]; % Unused
crankMax = 4; % Saturate, since furthest extension is nonphysical
crank(crank>crankMax) = crankMax;
crankScale = floor((2^16-1)./crank(:,end)); % Saturate to max range of femur encoder
crank_scaled = floor(max(0,crank.*crankScale)); %

%crScale = 1/(25*2^16)*crankScale; % Multiply with motor encoder value to get crank
crScale = 1/(25*crankMax)*ones(size(crankScale));

% extension function output is in meters
extension = [-x.*(-9.689463228112449e-2)+x.^2.*6.989908944388874e-2-...
    x.^3.*4.630772363424869e-2 + 0.09; ... % Rudolph
    -x.*(-1.10783409238789e-1)+x.^2.*3.18398134512155e-2-...
    x.^3.*2.62460134201421e-2 + 0.09; ... % Dasher
    - 0.04776*x.^3 + 0.1042*x.^2 + 0.05917*x + 0.07623]; % Unused
extensionScale = floor((2^16-1)./max(extension,[],2));
extension_scaled = floor(max(0,extension.*extensionScale));

extScale = max(extension,[],2); % Leg extension in m per 2^16 counts

% MA function output is in N/Nm (mechanical advantage from femur angle)
% TODO: these are currently wrong
MA = [((-x).*9.722770365181102+(-x).^2.*9.229589599064714+(-x).^3.*4.443622508854662+...
    (-x).^4.*3.480286004976706e-1+5.545933486743863e-1)./...
    ((-x).*1.397981788877775e-1+(-x).^2.*1.389231709027461e-1-9.689463228112449e-2); ... % Rudolph
    ((-x).*7.868075191250353+(-x).^2.*6.052244719389666+(-x).^3.*6.708295328119813e-1-...
    (-x).^4.*1.126541544353222+2.219771122747685e-1)./...
    ((-x).*6.367962690243101e-2+(-x).^2.*7.873804026042631e-2-1.10783409238789e-1); % Dasher
    4963*x.^3-1060*x.^2+75.7*x+1.961]; % Unused
MA(MA>128) = 128; % Saturate, since furthest extension is nonphysical
MA(MA<1) = 1;
maScale = floor((2^16-1)./max(MA,[],2));
MA_scaled = floor(max(0,MA.*maScale));

MAScale = max(MA,[],2); % MA scale in rad/m per 2^16 counts


crankHeader = ['static unsigned int crank_femur_256lut[',num2str(resolution),'] = {'];
extensionHeader = ['static unsigned int leg_femur_256lut[',num2str(resolution),'] = {'];
MAHeader = ['static unsigned int MA_femur_256lut[',num2str(resolution),'] = {'];
footer = '};\n';
for ii = 1:numel(names)
    crankString{ii} = sprintf('%u,' , crank_scaled(ii,:));
    crankString{ii} = crankString{ii}(1:end-1);% strip final comma
    extensionString{ii} = sprintf('%u,', extension_scaled(ii,:));
    extensionString{ii} = extensionString{ii}(1:end-1);% strip final comma
    MAString{ii} = sprintf('%u,', MA_scaled(ii,:));
    MAString{ii} = MAString{ii}(1:end-1); % strip final comma
end

fid = fopen('../lib/lut.h','wt');
fprintf(fid, ['//Lookup tables have ',num2str(resolution),' samples from 0 to 90 degrees of femur rotation.\n\n']);
for ii = 1:numel(names)
    if ii == 1
        fprintf(fid, ['#if ROBOT_NAME == ',names{ii},'\n']);
    else
        fprintf(fid, ['\n#elif ROBOT_NAME == ',names{ii},'\n']);
    end
    fprintf(fid, crankHeader);
    fprintf(fid, crankString{ii});
    fprintf(fid, footer);
    fprintf(fid, ['static float motPos_to_femur_crank_units = ' num2str(crScale(ii)) ';\n']);
    
    fprintf(fid, extensionHeader);
    fprintf(fid, extensionString{ii});
    fprintf(fid, footer);
    fprintf(fid, ['static float femur_leg_units = ' num2str(extScale(ii)) ';']);
    fprintf(fid, ' //Leg extension is femur_leg_units m per 2^16\n');
    
    fprintf(fid, MAHeader);
    fprintf(fid, MAString{ii});
    fprintf(fid, footer);
    fprintf(fid, ['static float femur_MA_units = ' num2str(MAScale(ii)) ';']);
    fprintf(fid, ' //Mechanical advantage is femur_MA_units N/Nm per 2^16.\n');
end
fprintf(fid, '#endif\n');
fclose(fid);