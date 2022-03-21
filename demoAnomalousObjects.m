% Instructions
% In Matlab 2016a, select the path of the faster-rcnn: /home/icai21/Modelos/faster_rcnn-master
%%% Edit the variable "ndxVideo" with the selected video
% Click on the "Run" button
% Click on the "Add to Path" button
% At the beginning of the output of this code, a linux command is provided. Copy it
% Open a linux shell and paste

%%% If you want to edit the virtualptz library, you may edit it as follows:
%%% Edit the variables of the video in /home/icai21/virtualptz/execs/vptz_anomalous_detection/src/main.cpp
%%% cd virtualptz
%%% make

%%% IMPORTANT: Run in MATLAB 2016a

rng('default');

ndxVideo = 2; % edit this variable to change the selected video!!!!!!!!!!!!
ndxAnomalousModel = 1;
ndxAngle = 1;

anomalousModels = {'dirichlet','kmeans','som','baseline_right','baseline_left','baseline_random','baseline_noMove'};
angles = {0,90,180,270};

videoName{1} = 'scenario3horse';
numFrames{1} = 1061;
gt{1} = 'scenario3horse_horse';

videoName{2} = 'scenario3sheep';
numFrames{2} = 1061;
gt{2} = 'scenario3sheep_sheep';

videoName{3} = 'scenario5dog';
numFrames{3} = 1835;
gt{3} = 'scenario5dog_dog';

videoName{4} = 'scenario3dogcow';
numFrames{4} = 531;
gt{4}='scenario3dogcow_cow';

videoName{5} = 'scenario3dogcow';
numFrames{5} = 531;
gt{5}='scenario3dogcow_dog';

videoName{6} = 'scenario3cow';
numFrames{6} = 1061;
gt{6} = 'scenario3cow_cow';

params.videoName = videoName{ndxVideo};
params.numFrames = numFrames{ndxVideo};
params.videoGT = gt{ndxVideo};
params.anomalousModel = anomalousModels{ndxAnomalousModel};
params.angle = angles{ndxAngle};


detectRCNNAnomalousObjects(params);

