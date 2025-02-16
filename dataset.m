clc; 

kkk = 1;

propath = '.'; % Current directory
pcl_files = dir(fullfile(propath, 'colored_*.ply')); % List .ply files pcl_files stores a list of files matching the pattern colored_*.ply.

for k = 1:size(pcl_files)
    clearvars -except k pcl_files t A kkk propath
    file = pcl_files(k).name;
    frame = file(1:size(file,2)-4);
    extension = file(size(file,2)-3: end);

    fullpath = strcat(propath,file);
    fullfilename = strcat(propath,frame);

    if extension==".ply"
        oo = plyread(fullpath);
        ptCloud = pointCloud([oo.vertex.x,oo.vertex.y,oo.vertex.z]);
    else
        oo = load(fullpath);
        ptCloud = pointCloud(oo.xyzPoints);
    end

    vertices(:,1) = ptCloud(:,1);
    vertices(:,2) = ptCloud(:,2);
    vertices(:,3) = ptCloud(:,3);

    apoints = [vertices(:,1),vertices(:,2),vertices(:,3)];


    salientNeighbours = 30;
    apoints = double(apoints);

    akdtreeobj = KDTreeSearcher(apoints,'distance','euclidean'); %makes a kdtreesearcher to find nearest neighbours of all points
    % kd tree is used to arrange points in space to gauge nearest neigbours
    % etc. The KD-Tree is built recursively by splitting data along one dimension at a time. 
    %The median value of the chosen dimension is used as a splitting point. 
    %The tree alternates between dimensions (e.g., for 3D data, it may first split along x, then y, then z, then repeat)

    aAdjpoints = knnsearch(akdtreeobj,apoints,'k',salientNeighbours+1);
    Adjpointsa = knnsearch(akdtreeobj,apoints,'k',salientNeighbours+1);

    aAdjpoints = aAdjpoints(:,2:end);

    [normalVectors,curvature] = findPointNormals(vertices,[],[0,0,0],true);
    featureSize = size(vertices,1);

    heatmap = zeroes(size(vertices,1),1);

    for i = 1:featureSize
        for j = 1:salientNeighbours
            nn1(j,:) = normalvectors(Adjpointsa(i,j),:);
        end

        convoluted = nn1'*nn1;
        [vb,lb] = eig(convoluted); %getting eigen values of normal vectors
        val(i,1) = lb(1,1);
        val(i,2) = lb(2,2);
        val(i,3) = lb(3,3);

        heatmap(i,1) = 1./norm(val(i,:));
    end

        normheat = (heatmap-min(heatmap))/(max(heatmap)-min(heatmap)); %normalises heatmap to 0,1 scale

        color = zeroes(size(vertices,1),3);
        index = zeroes(size(vertices,1),1);

        range1 = max(normheat);

        cmp = colormap(jet);
        cmpsize = size(cmp,1);

        interval = range1/64; %for every 'interval' jump in heatmap value there is one step increase in color map

        toc
    for i = 1:featureSize
        m=1;
        while(normheat(i,:)>m*interval && m<64)
            m = m+1;
        end
        color(i,:) = cmp(m,:);
        index(i,:) = k;
    end

    saliencyColors = colorisepointcloud(vertices,index);
    endingl = "_eigen_colormap.obj";
    fullvertexpath = strcat(fullfilename,endingl);
    file1 = fopen(fullpath,'w');

    for i = 1:size(vertices,1)
        fprintf(file1,"v %f %f %f %f %f %f",vertices(i,1),vertices(i,2),vertices(i,3),saliencyColors(i,1),saliencyColors(i,2),saliencyColors(i,3));
    end
    fclose(file1);

    ending2 = "_eigen_binary.obj";
    fullbinarypath = strcat(fullfilename,ending2);
    file1 = fopen(fullpath,'w');

    for i = 1:size(vertices,1)
        if index(i,1)<=10
            fprintf(file1,"v %f %f %f %f %f %f",vertices(i,1),vertices(i,2),vertices(i,3),0,0,1);
        else
            fprintf(file1,"v %f %f %f %f %f %f",vertices(i,1),vertices(i,2),vertices(i,3),1,0,0);
        end
    end

    fclose(file1);
end








