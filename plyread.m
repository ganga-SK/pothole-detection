%polygon file format files reading
%extracting data from point clouds

function [Elements,varargout] = plyread(Path,Str)

[fid,Msg] = fopen(Path,"rt");  %opening file in read text mode
if fid==-1, error(Msg); 
end

Buf = fscanf(fid,'%s',1); %reading one word from file
if ~strcmp(Buf,'ply')
    fclose(fid);
    error('Not a ply file');
end

Position = ftell(fid);
NumComments = 0;
NumProperties = 0;
NumElements = 0;
Format = '';
Comments = {};
Elements = [];
ElementCount = [];
PropertyTypes = [];
ElementNames = {};
PropertyNames = [];

while 1
    Buf = fgetl(fid);
    BufRem = Buf;
    Token = {};
    Count = 0;

    while ~isempty(BufRem)
        [tmp,BufRem] = strtok(BufRem);

        if ~isempty(tmp)
            Count = Count+1;
            Token{Count} = tmp;
        end
    end

    if Count
        switch(lower(Token{1}))
            case 'format'
                if Count>=2
                    Format = lower(Token{2});

                    if Count==3 && ~strcmp(Token{3},'1.0')
                        fclose(fid);
                        error('Only files of version 1.0 is supported');
                    end
                end
                
                    




