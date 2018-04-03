fmax = pi/2;
resolution = 2^8;
femur = linspace(0,fmax,resolution);
f_MA = @(x) -987.1*x.^7 -5013*x.^6 -1.007e+04*x.^5 -1e+04*x.^4 ... % MA from femur angle
    -4963*x.^3 -1060*x.^2 -75.7*x +1.961;
MA = f_MA(-femur); % MA rad/m
MA_scale = MA(end); % MA scale in rad/m per 2^16 counts
scale = (2^16-1)/MA(end);

MA_scaled = floor(max(0,MA.*scale));

header = ['static unsigned int MA_femur_lut[',num2str(resolution),'] = {\n'];
lutString = sprintf('%u,', MA_scaled);
lutString = lutString(1:end-1);% strip final comma
footer = '\n};\n';

fid = fopen('../lib/lut_MA.h','wt');
fprintf(fid, header);
fprintf(fid, lutString);
fprintf(fid, footer);
fprintf(fid, ['//Lookup table has ',num2str(resolution),' samples from 0 to 90 degrees of femur rotation\n']);
fprintf(fid, ['static float femur_MA_units = ' num2str(MA_scale) ';\n']);
fprintf(fid, '//Mechanical advantage is femur_MA_units rad/m per 2^16\n');
fclose(fid);