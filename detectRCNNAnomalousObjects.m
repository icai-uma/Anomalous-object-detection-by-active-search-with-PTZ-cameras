function detectRCNNAnomalousObjects(params)

close all;
clc;
clear mex;
clear is_valid_handle; % to clear init_key
%run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'startup'));
run('/home/icai21/Modelos/faster_rcnn-master/startup.m');
%% -------------------- CONFIG --------------------
opts.caffe_version          = 'caffe_faster_rcnn';
opts.gpu_id                 = auto_select_gpu;
active_caffe_mex(opts.gpu_id, opts.caffe_version);

opts.per_nms_topN           = 6000;
opts.nms_overlap_thres      = 0.7;
opts.after_nms_topN         = 300;
opts.use_gpu                = true;

opts.test_scales            = 600; %600;

%% -------------------- PATHS --------------------

deltaFrame = 0;

project_path = '/home/icai21/Modelos/codigos/anomalous_detection/';
dirichlet_model = [project_path 'modelDirichlet.mat'];
load(dirichlet_model);
ModelDirichlet = Model;
kmeans_model = [project_path 'modelKMeans.mat'];
load(kmeans_model);
som_model = [project_path 'modelSOM.mat'];
load(som_model);

project_path = [project_path 'temp/' params.videoGT '-' int2str(params.angle) '/' params.anomalousModel '/'];


frames_path = [project_path 'output_PTZ_frames/'];
imageTypeFile = '.png';
output_frames_path = [project_path 'output_RCNN_frames/'];
output_data_path = [project_path 'output_RCNN_data/'];
outputptz_data = [project_path 'output_PTZ_data/'];
outputptz_frames_centroid = [project_path 'output_PTZ_frames_centroid/'];

frames_name = 'in_%06d';
frame_path = [frames_path frames_name];



% remove content from temp folders
[stat, mess, id] = rmdir(frames_path, 's'); % Delete the folder and its files if it was created
mkdir(frames_path); % create the folder
fileattrib(frames_path,'+w','a') % give written permissions

[stat, mess, id] = rmdir(output_frames_path, 's'); % Delete the folder and its files if it was created
mkdir(output_frames_path); % create the folder
fileattrib(output_frames_path,'+w','a') % give written permissions

[stat, mess, id] = rmdir(output_data_path, 's'); % Delete the folder and its files if it was created
mkdir(output_data_path); % create the folder
fileattrib(output_data_path,'+w','a') % give written permissions

[stat, mess, id] = rmdir(outputptz_data, 's'); % Delete the folder and its files if it was created
mkdir(outputptz_data); % create the folder
fileattrib(outputptz_data,'+w','a') % give written permissions

[stat, mess, id] = rmdir(outputptz_frames_centroid, 's'); % Delete the folder and its files if it was created
mkdir(outputptz_frames_centroid); % create the folder
fileattrib(outputptz_frames_centroid,'+w','a') % give written permissions

comando = sprintf('/home/icai21/virtualptz/build/bin/vptz_anomalous_detection %d %s %s %s',params.angle,params.videoName,params.videoGT,params.anomalousModel);
disp(comando);
%%%%system(comando);


%% -------------------- INIT_MODEL --------------------
model_dir                   = fullfile(pwd, 'output', 'faster_rcnn_final', 'faster_rcnn_VOC0712_vgg_16layers'); %% VGG-16
%model_dir                   = fullfile(pwd, 'output', 'faster_rcnn_final', 'faster_rcnn_VOC0712_ZF'); %% ZF
proposal_detection_model    = load_proposal_detection_model(model_dir);

proposal_detection_model.conf_proposal.test_scales = opts.test_scales;
proposal_detection_model.conf_detection.test_scales = opts.test_scales;
if opts.use_gpu
    proposal_detection_model.conf_proposal.image_means = gpuArray(proposal_detection_model.conf_proposal.image_means);
    proposal_detection_model.conf_detection.image_means = gpuArray(proposal_detection_model.conf_detection.image_means);
end

% caffe.init_log(fullfile(pwd, 'caffe_log'));
% proposal net
rpn_net = caffe.Net(proposal_detection_model.proposal_net_def, 'test');
rpn_net.copy_from(proposal_detection_model.proposal_net);
% fast rcnn net
fast_rcnn_net = caffe.Net(proposal_detection_model.detection_net_def, 'test');
fast_rcnn_net.copy_from(proposal_detection_model.detection_net);

% set gpu/cpu
if opts.use_gpu
    caffe.set_mode_gpu();
else
    caffe.set_mode_cpu();
end       

%% -------------------- WARM UP --------------------
% the first run will be slower; use an empty image to warm up

for j = 1:2 % we warm up 2 times
    im = uint8(ones(375, 500, 3)*128);
    if opts.use_gpu
        im = gpuArray(im);
    end
    [boxes, scores]             = proposal_im_detect(proposal_detection_model.conf_proposal, rpn_net, im);
    aboxes                      = boxes_filter([boxes, scores], opts.per_nms_topN, opts.nms_overlap_thres, opts.after_nms_topN, opts.use_gpu);
    if proposal_detection_model.is_share_feature
        [boxes, scores]             = fast_rcnn_conv_feat_detect(proposal_detection_model.conf_detection, fast_rcnn_net, im, ...
            rpn_net.blobs(proposal_detection_model.last_shared_output_blob_name), ...
            aboxes(:, 1:4), opts.after_nms_topN);
    else
        [boxes, scores]             = fast_rcnn_im_detect(proposal_detection_model.conf_detection, fast_rcnn_net, im, ...
            aboxes(:, 1:4), opts.after_nms_topN);
    end
end

%% -------------------- TESTING --------------------
%im_names = {'001763.jpg', '004545.jpg', '000542.jpg', '000456.jpg', '001150.jpg', 'prueba.png', '1.png', '2.png', '3.png', '4.png', '5.png'};
% these images can be downloaded with fetch_faster_rcnn_final_model.m
%%%im_names = dir(fullfile(framespath, pattern));

running_time = [];
samples = [];
timestamps = [];
bbs = [];
lastbb = 0;
object_class = [];
ndx_anomalous = [];

ax = gca();

%for j = 1:length(im_names)
 for j=1:params.numFrames-1 % last frame report an error
     startIteration(j) = tic;
     name = sprintf(frames_name,j + deltaFrame);
     
     disp(sprintf('Waiting for the frame %s',[name imageTypeFile]))
     while (~exist(sprintf([frame_path imageTypeFile],j + deltaFrame),'file'))
         pause(0.001);
     end

    %[~,name,~] = fileparts(im_names(j).name);    
    %im = imread(fullfile(frames_path, im_names(j).name));
    
    im = imread(fullfile(frames_path,[name imageTypeFile]));
    
    if opts.use_gpu
        im = gpuArray(im);
    end
    
    % test proposal
    th = tic();
    [boxes, scores]             = proposal_im_detect(proposal_detection_model.conf_proposal, rpn_net, im);
    t_proposal = toc(th);
    th = tic();
    aboxes                      = boxes_filter([boxes, scores], opts.per_nms_topN, opts.nms_overlap_thres, opts.after_nms_topN, opts.use_gpu);
    t_nms = toc(th);
    
    % test detection
    th = tic();
    if proposal_detection_model.is_share_feature
        [boxes, scores]             = fast_rcnn_conv_feat_detect(proposal_detection_model.conf_detection, fast_rcnn_net, im, ...
            rpn_net.blobs(proposal_detection_model.last_shared_output_blob_name), ...
            aboxes(:, 1:4), opts.after_nms_topN);
    else
        [boxes, scores]             = fast_rcnn_im_detect(proposal_detection_model.conf_detection, fast_rcnn_net, im, ...
            aboxes(:, 1:4), opts.after_nms_topN);
    end
    %%%t_detection = toc(th);
    %%%running_time(end+1) = t_proposal + t_nms + t_detection;
    
    %fprintf('%s (%dx%d): time %.3fs (resize+conv+proposal: %.3fs, nms+regionwise: %.3fs)\n', im_names(j).name, ...
    %    size(im, 2), size(im, 1), t_proposal + t_nms + t_detection, t_proposal, t_nms+t_detection);
    %%%fprintf('%s (%dx%d): time %.3fs (resize+conv+proposal: %.3fs, nms+regionwise: %.3fs)\n', name, ...
    %%%    size(im, 2), size(im, 1), t_proposal + t_nms + t_detection, t_proposal, t_nms+t_detection);
    
    
    % visualize
    classes = proposal_detection_model.classes;
    boxes_cell = cell(length(classes), 1);
    thres = 0.8;
    for i = 1:length(boxes_cell)
        boxes_cell{i} = [boxes(:, (1+(i-1)*4):(i*4)), scores(:, i)];
        boxes_cell{i} = boxes_cell{i}(nms(boxes_cell{i}, 0.3), :);
        
        I = boxes_cell{i}(:, 5) >= thres;
        boxes_cell{i} = boxes_cell{i}(I, :);
        for bbix = 1:size(boxes_cell{i}, 1)
            [~, idx] = ismember(boxes_cell{i}(bbix, 1:4), boxes(:, (1+(i-1)*4):(i*4)), 'rows');
            samples = cat(2, samples, scores(idx, :)');
            timestamps = cat(2, timestamps, j);
            %bbs = cat(2, bbs, [boxes_cell{i}(bbix, 1:4), (boxes_cell{i}(bbix, 3) - boxes_cell{i}(bbix, 1)) / 2, (boxes_cell{i}(bbix, 4) - boxes_cell{i}(bbix, 2)) / 2]');
            centroidX = (boxes_cell{i}(bbix, 3) - boxes_cell{i}(bbix, 1)) / 2 + boxes_cell{i}(bbix, 1);
            centroidY = (boxes_cell{i}(bbix, 4) - boxes_cell{i}(bbix, 2)) / 2 + boxes_cell{i}(bbix, 2);
            bbs = cat(2, bbs, [boxes_cell{i}(bbix, 1:4), centroidX, centroidY]');
            % bbs(1,i) top left corner X
            % bbs(2,i) top left corner Y 
            % bbs(3,i) bottom right corner X
            % bbs(4,i) bottom right corner Y
            % bbs(5,i) centroid X
            % bbs(6,i) centroid Y
        end
    end
    
    % select the non anomalous objects detected in the frame 

    if strcmp(params.anomalousModel,'dirichlet')
        
        TestLogDensities=TestNonParametricDirichlet(ModelDirichlet, samples(:, lastbb+1:end));
        [minLogDensities,MyMostAnomalousNdx]=min(TestLogDensities);
        threshold = -52.653;
    elseif strcmp(params.anomalousModel,'kmeans')
        % calcular distancia entre cada objeto anomalo y los centroides
        % nos quedamos con la distancia mayor, que correspondera al mas anomalo
        
        
        Distances=TestKMeans(ModelKMeans, samples(:, lastbb+1:end));
        Distances = -Distances;
        [minLogDensities,MyMostAnomalousNdx]=min(Distances);
        threshold = -0.820; %%%%%%% modificar!!!!!!!!!!!!!!
        
    elseif strcmp(params.anomalousModel,'som')
        Distances=TestSOM(ModelSOM, samples(:, lastbb+1:end));
        Distances = -Distances;
        [minLogDensities,MyMostAnomalousNdx]=min(Distances);
        threshold = -0.820; %%%%%%% modificar!!!!!!!!!!!!!!
    else
        MyMostAnomalousNdx = [];
        minLogDensities = 0;
        threshold = 0;
    end
   
    %figure(j);
    showboxes(im, boxes_cell, classes, 'voc',MyMostAnomalousNdx);
    %%%showboxes(im, boxes_cell, classes, 'voc', MyAnomalousNdx, MyMostAnomalousNdx);
    
    
    
    if ((size(MyMostAnomalousNdx,2)>0) && (minLogDensities <= threshold)) % if any object exists and it is very anomalous
        i=lastbb+MyMostAnomalousNdx;
        fprintf('Anomalous object is bounding box number %d.\r\n',i);
        % calculate its size
        % the bounding box is a rectangle, so its size is a*b
        
        targetSize = abs((bbs(3,i) - bbs(1,i)) * (bbs(4,i) - bbs(2,i)));
        targetCentroidX = bbs(5,i);
        targetCentroidY = bbs(6,i); 
        
        %targetObjetFrame{j,1} = classes(find(samples(:, lastbb+MyMostAnomalousNdx)==max(samples(:, lastbb+MyMostAnomalousNdx)))); % class of the target object
        %targetObjetFrame{j,2} = TestLogDensities(MyMostAnomalousNdx);
    else
        % no anomalous object was detected
        % the camera should search any anomalous object
        targetSize = 0;
        targetCentroidX = -1;
        targetCentroidY = -1;
    end
    
    if strcmp(params.anomalousModel,'baseline_left')
        targetSize = 0.25*(size(im,1) * size(im,2)); % size between 0.30 and 0.05
        targetCentroidX = 1; % left horizontal
        targetCentroidY = round(size(im,1)/2); % center vertical
    elseif strcmp(params.anomalousModel,'baseline_right')
        targetSize = 0.25*(size(im,1) * size(im,2)); % size between 0.30 and 0.05
        targetCentroidX = size(im,2); % left horizontal
        targetCentroidY = round(size(im,1)/2); % center vertical
    elseif strcmp(params.anomalousModel,'baseline_random')
        targetSize = 0.25*(size(im,1) * size(im,2)); % size between 0.30 and 0.05, no random size
        targetCentroidX = round(rand(1,1)*size(im,2)); % random horizontal
        %targetCentroidY = round(rand(1,1)*size(im,1)); % random vertical
        targetCentroidY = round(size(im,1)/2); % center vertical
	elseif strcmp(params.anomalousModel,'baseline_noMove')
        targetSize = 0.25*(size(im,1) * size(im,2)); % size between 0.30 and 0.05, no random size
        targetCentroidX = round(size(im,2)/2); % center horizontal
        targetCentroidY = round(size(im,1)/2); % center vertical
    end
    
    t_detection = toc(th);
    running_time(end+1) = t_proposal + t_nms + t_detection;
    
    % show what the deep learning is watching
    f = getframe(gca);
    [X, ~] = frame2im(f);
    imwrite(X, fullfile(output_frames_path, sprintf('%s.png', name)), 'png');
    pause(0.001);
    %close all;
    lastbb = size(samples, 2);
     
    
     % create a temp file with the data
    pathOldDataTempNonPan = [output_data_path 'temp.bin'];
    numFrame = name(4:end);
    
	pFile = fopen(pathOldDataTempNonPan, 'w');
    				
    fwrite(pFile,targetSize~=0,'int');
    fwrite(pFile,targetCentroidX,'int'); 
    fwrite(pFile,targetCentroidY,'int'); 
    fwrite(pFile,targetSize,'int'); 

    % if size(MyMostAnomalousNdx,2)>0 % if an anomalous object exists, write the bounding box data   
    if ((size(MyMostAnomalousNdx,2)>0) && (minLogDensities < threshold))
        fwrite(pFile,bbs(1,i),'int'); 
        fwrite(pFile,bbs(2,i),'int'); 
        fwrite(pFile,bbs(3,i) - bbs(1,i),'int'); 
        fwrite(pFile,bbs(4,i) - bbs(2,i),'int'); 
    end
				
	fclose (pFile);
	
    % rename the temp file
    pathNewDataTempNonPan = [output_data_path 'DataFrame_%s.bin'];
    pathNewDataTempNonPan = sprintf(pathNewDataTempNonPan, numFrame);
    movefile(pathOldDataTempNonPan, pathNewDataTempNonPan);
    endIteration(j) = toc(startIteration(j));
end
%fprintf('mean time: %.3fs\n', mean(running_time));
fprintf('mean time: %.3fs, median time: %.3fs\n', mean(endIteration),median(endIteration));

%save(outmat, 'samples', 'timestamps', 'bbs', 'classes');



fileID = fopen([project_path 'time_performance.txt'],'w');
fprintf(fileID,'mean time: %.3fs, median time: %.3fs\n', mean(endIteration),median(endIteration));
fclose(fileID);

caffe.reset_all(); 
clear mex;

end

function proposal_detection_model = load_proposal_detection_model(model_dir)
    ld                          = load(fullfile(model_dir, 'model'));
    proposal_detection_model    = ld.proposal_detection_model;
    clear ld;
    
    proposal_detection_model.proposal_net_def ...
                                = fullfile(model_dir, proposal_detection_model.proposal_net_def);
    proposal_detection_model.proposal_net ...
                                = fullfile(model_dir, proposal_detection_model.proposal_net);
    proposal_detection_model.detection_net_def ...
                                = fullfile(model_dir, proposal_detection_model.detection_net_def);
    proposal_detection_model.detection_net ...
                                = fullfile(model_dir, proposal_detection_model.detection_net);
    
end

function aboxes = boxes_filter(aboxes, per_nms_topN, nms_overlap_thres, after_nms_topN, use_gpu)
    % to speed up nms
    if per_nms_topN > 0
        aboxes = aboxes(1:min(length(aboxes), per_nms_topN), :);
    end
    % do nms
    if nms_overlap_thres > 0 && nms_overlap_thres < 1
        aboxes = aboxes(nms(aboxes, nms_overlap_thres, use_gpu), :);       
    end
    if after_nms_topN > 0
        aboxes = aboxes(1:min(length(aboxes), after_nms_topN), :);
    end
end
