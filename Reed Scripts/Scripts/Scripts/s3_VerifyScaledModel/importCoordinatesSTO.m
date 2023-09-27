function data = importCoordinatesSTO(file)

% imports coordinate data stored in .mot or .sto format

fid = fopen(file,'r');

% some table properties
data.filename = file;
within_header = true;
nHeaderLines = 0;
while within_header
    this_row = fgets(fid);
    if contains(this_row,'nRows')
        data.nSamples = str2double(this_row(strfind(this_row,'=')+1:end));
    elseif contains(this_row,'nColumns')
        data.nCoordinates = str2double(this_row(strfind(this_row,'=')+1:end)) - 1;
    elseif contains(this_row,'inDegrees')
        if contains(this_row,'inDegrees=yes','IgnoreCase',true)
            data.rotationalUnits = 'degrees';
        elseif contains(this_row,'inDegrees=no','IgnoreCase',true)
            data.rotationalUnits = 'radians';
        else
            data.rotationUnits = '';
        end
    elseif contains(this_row,'endheader')
        within_header = false;
    end
    nHeaderLines = nHeaderLines + 1;
end
fclose(fid);

% read
tbl = readtable(file,'FileType','delimitedtext','Delimiter','\t','NumHeaderLines',nHeaderLines);
data.time = tbl.time';
data.coordinateNames = tbl.Properties.VariableNames;
data.coordinateNames(strcmp(data.coordinateNames,'time')) = [];
for k = 1:data.nCoordinates
    data.coordinate.(data.coordinateNames{k}) = tbl.(data.coordinateNames{k})';
end

end