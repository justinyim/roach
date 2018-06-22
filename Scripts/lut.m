names = {'SALTO_1P_RUDOLPH','SALTO_1P_DASHER'};

fmax = pi/2;
resolution = 2^8;
x = linspace(0,fmax,resolution);
% crank functions are in radians
crank = [8.17*x.^7 + 39.79*x.^6 + 76.13*x.^5 + 71.72*x.^4 + 33.52*x.^3 + 4.437*x.^2 + 0.516*x + 0.004582; ... % Rudolph
    2.003.*x.^5.0 - 7.434.*x.^4.0 + 9.429.*x.^3.0 - 2.691.*x.^2.0 + 0.3893.*x - 0.001175];
scale = floor((2^16-200)./crank(:,end)); % Saturate to max range of femur encoder
crank_scaled = floor(max(0,crank.*scale)); %

crScale = 1/(25*2^16)*scale; % Multiply with motor encoder value to get crank

header = ['static unsigned int crank_femur_256lut[',num2str(resolution),'] = {\n'];
for ii = 1:size(crank,1)
    lutString{ii} = sprintf('%u,' , crank_scaled(ii,:));
    lutString{ii} = lutString{ii}(1:end-1);% strip final comma
end
footer = '\n};\n';

fid = fopen('../lib/lut.h','wt');
for ii = 1:size(crank,1)
    if ii == 1
        fprintf(fid, ['#if ROBOT_NAME == ',names{ii},'\n']);
    else
        fprintf(fid, ['#elif ROBOT_NAME == ',names{ii},'\n']);
    end
    fprintf(fid, header);
    fprintf(fid, lutString{ii});
    fprintf(fid, footer);
    fprintf(fid, ['//Lookup table has ',num2str(resolution),' samples from 0 to 90 degrees of femur rotation\n']);
    fprintf(fid, ['static float motPos_to_femur_crank_units = ' num2str(crScale(ii)) ';\n']);
end
fprintf(fid, '#endif\n');
fclose(fid);