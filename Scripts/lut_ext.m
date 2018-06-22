fmax = pi/2;
resolution = 2^8;
femur = linspace(0,fmax,resolution);
%extension = - 0.04776*femur.^3 + 0.1042*femur.^2 + 0.05917*femur + 0.07623; % meters
x = femur;
extension = 0.04563*x.^3 + 0.07215*x.^2 - 0.0944*x + 0.0003031 + 0.07; % Rudolph
extScale = extension(end); % Extension scale in m per 2^16 counts
scale = (2^16-1)/extension(end);

extension_scaled = floor(max(0,extension.*scale));

header = ['static unsigned int leg_femur_lut[',num2str(resolution),'] = {\n'];
lutString = sprintf('%u,', extension_scaled);
lutString = lutString(1:end-1);% strip final comma
footer = '\n};\n';

fid = fopen('../lib/lut_ext.h','wt');
fprintf(fid, header);
fprintf(fid, lutString);
fprintf(fid, footer);
fprintf(fid, ['//Lookup table has ',num2str(resolution),' samples from 0 to 90 degrees of femur rotation\n']);
fprintf(fid, ['static float femur_leg_units = ' num2str(extScale) ';\n']);
fprintf(fid, '//Leg extension is femur_leg_units m per 2^16\n');
fclose(fid);