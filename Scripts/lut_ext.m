fmax = pi/2;
resolution = 2^8;
femur = linspace(0,fmax,resolution);
extension = - 0.04776*femur.^3 + 0.1042*femur.^2 + 0.05917*femur + 0.07623; % radians
scale = floor((2^16-200)/extension(end)); % Saturate to max range of femur encoder
extension = floor(max(0,extension.*scale)); % 

crScale = 1/(2^16)*scale; % Multiply with motor encoder value to get crank

header = 'static unsigned int leg_femur_256lut[256] = {\n';
lutString = sprintf('%u,' , extension);
lutString = lutString(1:end-1);% strip final comma
footer = '\n};\n';

fid = fopen('../lib/lut_ext.h','wt');
fprintf(fid, header);
fprintf(fid, lutString);
fprintf(fid, footer);
fprintf(fid, ['static float femur_leg_units = ' num2str(crScale) ';\n']);
fclose(fid);