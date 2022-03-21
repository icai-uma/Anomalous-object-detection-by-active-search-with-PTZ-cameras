
//======================================================================================
//
// This program is used to view ground truth annotations while controlling the camera.
//
// Before running, set the input scenario/gt paths using the defines below.
//
//======================================================================================

#include "litiv/vptz/virtualptz.hpp"

#include <iostream>
#include <fstream>

using namespace std;
#include <sys/types.h>
#include <sys/stat.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define INPUT_SCENARIO_PATH        "/home/icai21/litiv_vptz_icip2015/scenario3/scenario3.avi"
//#define INPUT_GT_SEQUENCE_PATH     "/home/icai21/litiv_vptz_icip2015/scenario3/gt/scenario3_fullbody02.yml"

// SELECT THE PTZ_CAM_STARTING_HORI_ANGLE
//#define PTZ_CAM_STARTING_HORI_ANGLE 0   // [0-359]

// CHOOSE ONE OF THEM

//#define INPUT_SCENARIO_PATH        "/home/icai21/Modelos/codigos/anomalous_detection/videos/scenario3horse.mp4"
//#define INPUT_GT_SEQUENCE_PATH     "/home/icai21/Modelos/codigos/anomalous_detection/videos/scenario3horse_horse.yml"
//#define VIDEO_NAME                 "scenario3horse"

//#define INPUT_SCENARIO_PATH        "/home/icai21/Modelos/codigos/anomalous_detection/videos/scenario3sheep.mp4"
//#define INPUT_GT_SEQUENCE_PATH     "/home/icai21/Modelos/codigos/anomalous_detection/videos/scenario3sheep_sheep.yml" 
//#define VIDEO_NAME                 "scenario3sheep"

//#define INPUT_SCENARIO_PATH        "/home/icai21/Modelos/codigos/anomalous_detection/videos/scenario5dog.mp4"
//#define INPUT_GT_SEQUENCE_PATH     "/home/icai21/Modelos/codigos/anomalous_detection/videos/scenario5dog_dog.yml" 
//#define VIDEO_NAME                 "scenario5dog"

//#define INPUT_SCENARIO_PATH        "/home/icai21/Modelos/codigos/anomalous_detection/videos/scenario3dogcow.mp4"
//#define INPUT_GT_SEQUENCE_PATH     "/home/icai21/Modelos/codigos/anomalous_detection/videos/scenario3dogcow_dog.yml" 
//#define INPUT_GT_SEQUENCE_PATH     "/home/icai21/Modelos/codigos/anomalous_detection/videos/scenario3dogcow_cow.yml" 
//#define VIDEO_NAME                 "scenario3dogcow"

int PTZ_CAM_STARTING_HORI_ANGLE;
char INPUT_SCENARIO_PATH[200];
char INPUT_GT_SEQUENCE_PATH[200];
char VIDEO_NAME[200];

//



////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// the values of the PTZ camera are taken from the datasheet of the SONY network camera SNC-RZ50N, pag 8
// http://www.networkwebcams.com/downloads/sony/nwlus_sony_snc-rz50n_ds.pdf?phpMyAdmin=07d80722534ef8cc190f6ff8c299d3a4
// PTZ_CAM_HORI_SPEED = 300 degrees/s
// PTZ_CAM_VERTI_SPEED = 300 degrees/s
// 30 fps

#define PTZ_CAM_HORI_MOVE  2 // 300 degrees per second/30 fps --> 10 degrees/frame
#define PTZ_CAM_VERTI_MOVE  2 // 300 degrees per second/30 fps --> 10 degrees/frame
#define PTZ_CAM_HORI_VERTI_MOVE_DEGREES 10 // if we move the cam 10 degrees, then the equivalent is 43 pixel
#define PTZ_CAM_HORI_VERTI_MOVE_PIXELS 43 // if we move the cam 10 degrees, then the equivalent is 43 pixel
#define PTZ_CAM_ZOOM_MOVE  2 // no specified
#define PTZ_CAM_MIN_VERTI_FOV  40 // no specified
#define PTZ_CAM_MAX_VERTI_FOV  140 // no specified

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

cv::Point g_oLastMouseClickPos;/*
void onMouse(int nEventCode, int x, int y, int, void*) {
    if(nEventCode==cv::EVENT_LBUTTONDOWN) {
        std::cout << "Clicked at [" << x << "," << y << "]" << std::endl;
        g_oLastMouseClickPos = cv::Point(x,y);
    }
}*/

void moveCameraRight(vptz::Camera * oCamera){
	int horiAngleTemp = int(oCamera->Get(vptz::PTZ_CAM_HORI_ANGLE));
	int horiAngle = horiAngleTemp;
	horiAngle = horiAngleTemp - PTZ_CAM_HORI_MOVE; //move right
	oCamera->Set(vptz::PTZ_CAM_HORI_ANGLE, horiAngle);
	cout << "Right move \n";
}

void moveCameraLeft(vptz::Camera * oCamera){
	int horiAngleTemp = int(oCamera->Get(vptz::PTZ_CAM_HORI_ANGLE));
	int horiAngle = horiAngleTemp;
	horiAngle = horiAngleTemp + PTZ_CAM_HORI_MOVE; //move left
	oCamera->Set(vptz::PTZ_CAM_HORI_ANGLE, horiAngle);
	cout << "Left move \n";
}

void moveCameraDown(vptz::Camera * oCamera){
	int vertiAngleTemp = int(oCamera->Get(vptz::PTZ_CAM_VERTI_ANGLE));
	int vertiAngle = vertiAngleTemp;
	vertiAngle = vertiAngleTemp + PTZ_CAM_VERTI_MOVE; //move down
	if(vertiAngle>=0 && vertiAngle<=180){
		oCamera->Set(vptz::PTZ_CAM_VERTI_ANGLE, vertiAngle);
		cout << "Down move \n";
	}
}

void moveCameraUp(vptz::Camera * oCamera){
	int vertiAngleTemp = int(oCamera->Get(vptz::PTZ_CAM_VERTI_ANGLE));
	int vertiAngle = vertiAngleTemp;
	vertiAngle = vertiAngleTemp - PTZ_CAM_VERTI_MOVE; //move up
	if(vertiAngle>=0 && vertiAngle<=180){
		oCamera->Set(vptz::PTZ_CAM_VERTI_ANGLE, vertiAngle);
		cout << "Up move \n";
	}
}

void moveCameraZoomIn(vptz::Camera * oCamera){
	int vertiFOVTemp = int(oCamera->Get(vptz::PTZ_CAM_VERTI_FOV)); 
	int vertiFOV = vertiFOVTemp;
	vertiFOV = vertiFOVTemp - PTZ_CAM_ZOOM_MOVE; //zoom in
	if(vertiFOV>=PTZ_CAM_MIN_VERTI_FOV && vertiFOV<=PTZ_CAM_MAX_VERTI_FOV){ //we update the zoom if it is between the valid values
		oCamera->Set(vptz::PTZ_CAM_VERTI_FOV, vertiFOV);
		cout << "Zoom in move \n";
	}	
}

void moveCameraZoomOut(vptz::Camera * oCamera){
	int vertiFOVTemp = int(oCamera->Get(vptz::PTZ_CAM_VERTI_FOV));
	int vertiFOV = vertiFOVTemp;
	vertiFOV = vertiFOVTemp + PTZ_CAM_ZOOM_MOVE; //zoom out
	if(vertiFOV>=PTZ_CAM_MIN_VERTI_FOV && vertiFOV<=PTZ_CAM_MAX_VERTI_FOV){ //we update the zoom if it is between the valid values
		oCamera->Set(vptz::PTZ_CAM_VERTI_FOV, vertiFOV);
		cout << "Zoom out move \n";
	}	
}

void frameProcessing(vptz::Camera * oCamera, cv::Mat oCurrView,int nFrameIdx, cv::Point oTargetPos_XY, cv::Rect bb){

	int frameWidth = oCurrView.cols; //640;
	int frameHeight = oCurrView.rows; //480;

	int frameSize = frameWidth * frameHeight;
	int contCameraSearching = 0;
	
	int targetFound = 0;
	int oTargetRight = 0;
	int oTargetDown = 0;
	int oTargetZoomIn = 0;
    int oGTRight = 0;
	int oGTDown = 0;
	int oGTZoomIn = 0;
	
	// In Matlab we are executing the faster-rcnn method (/home/icai21/Modelos/codigos/anomalous_detection/script_fmcp_dirichlet.m).
	// It returns a frame with all detected objects
	// Based on this image, it generates a file with the coordinates of the centroid and the size of the target object	
	
	char pathDataTempRCNN[200];
	sprintf(pathDataTempRCNN, "/home/icai21/Modelos/codigos/anomalous_detection/temp/%s/output_RCNN_data/DataFrame_%06d.bin", VIDEO_NAME, nFrameIdx); 
	
	// wait to the generated file
	printf( "Waiting for the frame %d\n", nFrameIdx );
	struct stat info;
	while( stat( pathDataTempRCNN, &info ) != 0 ){
    	//printf( "cannot access %s\n", pathDataTempRCNN );
		cv::waitKey(1);
	}
		
	// read the file	
	int *U = (int*)malloc( sizeof(int));
	std::ifstream infile;
    infile.open(pathDataTempRCNN, std::ios::in | std::ios::binary);
    infile.read((char*)U, 4*sizeof(int)); //4 is the number of values to read
    infile.close();
	
	int targetInFrame = U[0];
	targetInFrame = targetInFrame;
	int targetCentroidX = U[1];
	int targetCentroidY = U[2];
	int targetSize = U[3];
	cout << "targetCentroidX = " << targetCentroidX << "\n";
	cout << "targetCentroidY = " << targetCentroidY << "\n";
	cout << "targetSize = " << targetSize << "\n";
	
    
    
    //// Calculate the GT movement of the camera applying a smoothing (suavizado)

	int gtTargetSize = bb.height * bb.width;
	cout << "gtTargetCentroidX = " << oTargetPos_XY.x << "\n";
	cout << "gtTargetCentroidY = " << oTargetPos_XY.y << "\n";
	cout << "gtTargetSize = " << gtTargetSize << "\n";
	
	
	
	if(oTargetPos_XY.x<0 || oTargetPos_XY.x>frameWidth || oTargetPos_XY.y<0 || oTargetPos_XY.y>frameHeight){
		targetFound = 0;
	}else{
		targetFound = 1;
	}	

	if(targetFound==1){ // gt object in the frame
		if((oCurrView.cols/2-oTargetPos_XY.x) < -PTZ_CAM_HORI_VERTI_MOVE_PIXELS){ //target to the right and the distance to it its higher than a cam movement (in pixels) in a frame			
			cout << "GT Right move \n"; 
            oGTRight = 1;
		}else if((oCurrView.cols/2-oTargetPos_XY.x) > PTZ_CAM_HORI_VERTI_MOVE_PIXELS){ 			
			cout << "GT Left move \n"; 
            oGTRight =- 1;
		}else{
			cout << "GT No horizontal move \n";
            oGTRight = 0;
		}

		if((oCurrView.rows/2-oTargetPos_XY.y) < -PTZ_CAM_HORI_VERTI_MOVE_PIXELS*2){ //target to the down //we let lower sensitive than horizontal move multiplying x 2
			cout << "GT Down move \n"; 
            oGTDown = 1;
		}else if((oCurrView.rows/2-oTargetPos_XY.y) > PTZ_CAM_HORI_VERTI_MOVE_PIXELS*2){ 
			cout << "GT Up move \n";
            oGTDown = -1;
		}else{
			cout << "GT No vertical move \n";
            oGTDown = 0;
		}

		if(gtTargetSize > 0.3*frameSize){ //gt target big, so we want to do zoom out
			cout << "GT Zoom out move \n"; 
            oGTZoomIn = -1;
		}else if(gtTargetSize < 0.05*frameSize){ //gt target small, so we want to do zoom in
			cout << "GT Zoom in move \n";
            oGTZoomIn = 1;
		}else{
			cout << "GT No Zoom move \n";
            oGTZoomIn = 0;
		}
	}else{//we need to search a target
		//cout << "No gt, camera looking for one \n";
		//moveCameraRight(oCamera); //move right
            oGTRight = 1;
		// calculate the vertical move of the camera to the vertical center (0 up - 180 down) applying a smoothing (suavizado)
		// in order to move the camera close to the center and then the camera stops the vertical movement
		int vertiAngle = int(oCamera->Get(vptz::PTZ_CAM_VERTI_ANGLE));
		if(vertiAngle < (90 - 2*PTZ_CAM_HORI_VERTI_MOVE_DEGREES)){ //camera is up
			//moveCameraDown(oCamera); //move down
			oGTDown = 1;
		}else if(vertiAngle > (90 + 2*PTZ_CAM_HORI_VERTI_MOVE_DEGREES)){ //camera is down
			//moveCameraUp(oCamera); //move up
			oGTDown = -1;
		}else{
			//cout << "No vertical move \n";
			oGTDown = 0;
		}
		// calculate the zoom move of the camera to the zoom center (40 in - 140 out) applying a smoothing (suavizado)
		// in order to move the camera close to the center and then the camera stops the zoom movement
		int vertiFOV = int(oCamera->Get(vptz::PTZ_CAM_VERTI_FOV)); 
		if(vertiFOV < (90 - PTZ_CAM_HORI_VERTI_MOVE_DEGREES)){ //camera is with a high zoom in
			//moveCameraZoomOut(oCamera); //move zoom out
			oGTZoomIn = -1;
		}else if(vertiFOV > (90 + PTZ_CAM_HORI_VERTI_MOVE_DEGREES)){ //camera is with a high zoom out
			//moveCameraZoomIn(oCamera); //move zoom in
			oGTZoomIn = 1;
		}else{
			//cout << "No zoom move \n";
			oGTZoomIn = 0;
		}
	}
	
	//// End calculate the GT movement
    
    // write the GT data
    char pathDataTempPTZ[200];
	sprintf(pathDataTempPTZ, "/home/icai21/Modelos/codigos/anomalous_detection/temp/%s/output_PTZ_data/GTDataFrame_%06d.bin", VIDEO_NAME, nFrameIdx); 
	std::ofstream outfileGT;
	outfileGT.open(pathDataTempPTZ,ios::binary|ios::out);
	outfileGT.write((char *)&targetFound,sizeof(targetFound));
	outfileGT.write((char *)&oTargetPos_XY.x,sizeof(oTargetPos_XY.x));
    	outfileGT.write((char *)&oTargetPos_XY.y,sizeof(oTargetPos_XY.y));
    	outfileGT.write((char *)&gtTargetSize,sizeof(gtTargetSize));
    	outfileGT.write((char *)&bb.x,sizeof(bb.x));
    	outfileGT.write((char *)&bb.y,sizeof(bb.y));
    	outfileGT.write((char *)&bb.width,sizeof(bb.width));
    	outfileGT.write((char *)&bb.height,sizeof(bb.height));
	outfileGT.close();
    
    
    
    
    
	
	
	oTargetPos_XY.x = targetCentroidX;
	oTargetPos_XY.y = targetCentroidY;

	//cv::circle(oCurrView, oTargetPos_XY, 3, cv::Scalar(0,255,0,255), 5); //draw target centroid
	
	char pathTempFolderFramesPTZ[100]; 
	sprintf(pathTempFolderFramesPTZ, "/home/icai21/Modelos/codigos/anomalous_detection/temp/%s/output_PTZ_frames_centroid/frameWithCentroids%06d.png", VIDEO_NAME, nFrameIdx); 	
	cv::imwrite(pathTempFolderFramesPTZ,oCurrView);
	
	//cout << "oTargetPos_XY.x = " << oTargetPos_XY.x << "\n";
	//cout << "oTargetPos_XY.y = " << oTargetPos_XY.y << "\n";
				
	//// Calculate the movement of the camera applying a smoothing (suavizado)
	
	if(oTargetPos_XY.x<0 || oTargetPos_XY.x>frameWidth || oTargetPos_XY.y<0 || oTargetPos_XY.y>frameHeight){
		//std::cout << "Target out" << std::endl;
		targetFound = 0;
	}else{
		//std::cout << "Target in" << std::endl;
		targetFound = 1;
	}

	if(targetFound==1){//camera following the target
		//cout << "oCurrView.cols/2 = " << oCurrView.cols/2 << "\n";
		//cout << "oCurrView.cols/2-oTargetPos_XY.x = " << oCurrView.cols/2-oTargetPos_XY.x << "\n";
		//cout << "PTZ_CAM_HORI_VERTI_MOVE_PIXELS = " << PTZ_CAM_HORI_VERTI_MOVE_PIXELS << "\n";
		if((oCurrView.cols/2-oTargetPos_XY.x) < -PTZ_CAM_HORI_VERTI_MOVE_PIXELS){ //target to the right and the distance to it its higher than a cam movement (in pixels) in a frame			
			oTargetRight=1; //move right
		}else if((oCurrView.cols/2-oTargetPos_XY.x) > PTZ_CAM_HORI_VERTI_MOVE_PIXELS){ 			
			oTargetRight=-1; //move left
		}

		//cout << "oCurrView.rows/2 = " << oCurrView.rows/2 << "\n";
		//cout << "oCurrView.rows/2-oTargetPos_XY.y = " << oCurrView.rows/2-oTargetPos_XY.y << "\n";
		//cout << "PTZ_CAM_HORI_VERTI_MOVE_PIXELS = " << PTZ_CAM_HORI_VERTI_MOVE_PIXELS << "\n";
		if((oCurrView.rows/2-oTargetPos_XY.y) < -PTZ_CAM_HORI_VERTI_MOVE_PIXELS*2){ //target to the down //we let lower sensitive than horizontal move multiplying x 2
			oTargetDown=1; //move down
		}else if((oCurrView.rows/2-oTargetPos_XY.y) > PTZ_CAM_HORI_VERTI_MOVE_PIXELS*2){ 
			oTargetDown=-1; //move up
		}
	
		if(targetSize > 0.3*frameSize){ //target big, so we want to do zoom out
			oTargetZoomIn=-1;
		}else if(targetSize < 0.05*frameSize){ //target small, so we want to do zoom in
			oTargetZoomIn=1;
		}
	}else{//no target, so that, camera searching
		cout << "No target, camera looking for one \n";
		oTargetRight=1; //move right
		// calculate the vertical move of the camera to the vertical center (0 up - 180 down) applying a smoothing (suavizado)
		// in order to move the camera close to the center and then the camera stops the vertical movement
		int vertiAngle = int(oCamera->Get(vptz::PTZ_CAM_VERTI_ANGLE));
		if(vertiAngle < (90 - 2*PTZ_CAM_HORI_VERTI_MOVE_DEGREES)){ //camera is up
			oTargetDown=1; //move down
		}else if(vertiAngle > (90 + 2*PTZ_CAM_HORI_VERTI_MOVE_DEGREES)){ //camera is down
			oTargetDown=-1; //move up
		}else{
			//cout << "No vertical move \n";
			oTargetDown=0;
		}
		// calculate the zoom move of the camera to the zoom center (40 in - 140 out) applying a smoothing (suavizado)
		// in order to move the camera close to the center and then the camera stops the zoom movement
		int vertiFOV = int(oCamera->Get(vptz::PTZ_CAM_VERTI_FOV)); 
		if(vertiFOV < (90 - PTZ_CAM_HORI_VERTI_MOVE_DEGREES)){ //camera is with a high zoom in
			oTargetZoomIn=-1; //move zoom out
		}else if(vertiFOV > (90 + PTZ_CAM_HORI_VERTI_MOVE_DEGREES)){ //camera is with a high zoom out
			oTargetZoomIn=1; //move zoom in
		}else{
			oTargetZoomIn=0;
			//cout << "No zoom move \n";
		}



		contCameraSearching++; //do nothing
	}
	
	
    
    //// End calculate the movement
	
	//// Move the camera 	

				
	if(oTargetRight==-1){ //target to the left		
		moveCameraLeft(oCamera); //move left
	}else if(oTargetRight==1){ //target to the right
		moveCameraRight(oCamera); //move right
	}else{
		cout << "No horizontal move \n";
	}
	
	if(oTargetDown==-1){ //target to the up		
		moveCameraUp(oCamera); //move up
	}else if(oTargetDown==1){ //target to the down
		moveCameraDown(oCamera); //move down
	}else{
		cout << "No vertical move \n";
	}
	
	if(oTargetZoomIn==-1){ //target big, so we want to do zoom out
		moveCameraZoomOut(oCamera); //zoom out
	}else if(oTargetZoomIn==1){ //target small, so we want to do zoom in
		moveCameraZoomIn(oCamera); //zoom in
	}else{
		cout << "No zoom move \n";
	}
		
    
	//// End move the camera 	
    
    // write the file with GT and approach moves
	sprintf(pathDataTempPTZ, "/home/icai21/Modelos/codigos/anomalous_detection/temp/%s/output_PTZ_data/MovesFrame_%06d.bin", VIDEO_NAME, nFrameIdx); 
	std::ofstream outfile;
	outfile.open(pathDataTempPTZ,ios::binary|ios::out);
    outfile.write((char *)&oGTRight,sizeof(oGTRight));
    outfile.write((char *)&oGTDown,sizeof(oGTDown));
    outfile.write((char *)&oGTZoomIn,sizeof(oGTZoomIn));
	outfile.write((char *)&oTargetRight,sizeof(oTargetRight));
    outfile.write((char *)&oTargetDown,sizeof(oTargetDown));
    outfile.write((char *)&oTargetZoomIn,sizeof(oTargetZoomIn));    
	outfile.close();
    
    cout << "processing end";
	
	//g_oLastMouseClickPos = cv::Point(oCurrView.cols/2+moveCol, oCurrView.rows/2+moveRow);
}



//int main(int /*argc*/, char** /*argv*/) {
int main(int argc, char** argv) {
    try {
	argc = argc;
	PTZ_CAM_STARTING_HORI_ANGLE = atoi(argv[1]);
//strcpy( VIDEO_NAME, argv[2] );
//sprintf(VIDEO_NAME, "%s-%s",argv[2],argv[1]);
sprintf(VIDEO_NAME, "%s-%s/%s",argv[3],argv[1],argv[4]);
cout << "argv[2]: " << argv[2] << std::endl;
cout << "VIDEO_NAME: " << VIDEO_NAME << std::endl;
	

sprintf(INPUT_SCENARIO_PATH,"/home/icai21/Modelos/codigos/anomalous_detection/videos/%s.mp4",argv[2]);
sprintf(INPUT_GT_SEQUENCE_PATH,"/home/icai21/Modelos/codigos/anomalous_detection/videos/%s.yml",argv[3]);

cout << "argv[2]: " << argv[2] << std::endl;
cout << "INPUT_SCENARIO_PATH: " << INPUT_SCENARIO_PATH << std::endl;
cout << "argv[3]: " << argv[3] << std::endl;
cout << "INPUT_GT_SEQUENCE_PATH: " << INPUT_SCENARIO_PATH << std::endl;


        cv::FileStorage oInputGT(INPUT_GT_SEQUENCE_PATH, cv::FileStorage::READ);
        int nGTFrameWidth = oInputGT["frameImageWidth"];
        int nGTFrameHeight = oInputGT["frameImageHeight"];
        double dGTVerticalFOV = oInputGT["verticalFOV"];
	int lastFrame = oInputGT["totalFrameNum"];
	lastFrame = lastFrame - 1; //last frame report an error
		
        vptz::Camera oCamera(INPUT_SCENARIO_PATH);
        vptz::GTTranslator oGTTranslator(&oCamera, nGTFrameWidth, nGTFrameHeight, dGTVerticalFOV);
        cv::Point oTargetPos_XY;
        cv::Point2d oTargetPos_HX;
        cv::Mat oCurrView;
        int nBBoxWidth, nBBoxHeight;
        int nFrameIdx;
		int initializedModel = -1;

        oCamera.Set(vptz::PTZ_CAM_FRAME_POS, 0);
	oCamera.Set(vptz::PTZ_CAM_HORI_ANGLE, PTZ_CAM_STARTING_HORI_ANGLE); 
        oCurrView = oCamera.GetFrame();
        cv::namedWindow("Current View");
        cv::imshow("Current View", oCurrView);
        cv::waitKey(1);
        //cv::setMouseCallback("Current View", onMouse, 0);
		
        g_oLastMouseClickPos = cv::Point(oCurrView.cols/2, oCurrView.rows/2);				

        cv::FileNode oGTNode = oInputGT["basicGroundTruth"];
		bool bPaused = false;
	nFrameIdx = 1;
	auto oGTFrame=oGTNode.begin();
	    int FrameIdxGT = (*oGTFrame)["framePos"];
            nBBoxWidth = (*oGTFrame)["width"];
            nBBoxHeight = (*oGTFrame)["height"];
            oTargetPos_HX.x = (*oGTFrame)["horizontalAngle"];
            oTargetPos_HX.y = (*oGTFrame)["verticalAngle"];

        //for(auto oGTFrame=oGTNode.begin(); oGTFrame!=oGTNode.end(); ++oGTFrame) {
	for(nFrameIdx = 1; nFrameIdx <= lastFrame; nFrameIdx++){
            
		

            oCamera.Set(vptz::PTZ_CAM_FRAME_POS, nFrameIdx);
			std::cout << "\t#" << nFrameIdx << std::endl;			

			
			while(true) {
				
				
				
				oCamera.GoToPosition(g_oLastMouseClickPos);
				oCurrView = oCamera.GetFrame();

				if(oCurrView.empty())
					break;
						
				
				cv::Rect bb;
				if(nFrameIdx>=FrameIdxGT && oGTFrame!=oGTNode.end()){ // if GT exists
				    FrameIdxGT = (*oGTFrame)["framePos"];
			            nBBoxWidth = (*oGTFrame)["width"];
			            nBBoxHeight = (*oGTFrame)["height"];
		        	    oTargetPos_HX.x = (*oGTFrame)["horizontalAngle"];
			            oTargetPos_HX.y = (*oGTFrame)["verticalAngle"];

					if(nFrameIdx==FrameIdxGT){
						oGTTranslator.GetGTTargetPoint(oTargetPos_HX.x, oTargetPos_HX.y, oTargetPos_XY);
				    
					    //cv::circle(oCurrView, oTargetPos_XY, 3, cv::Scalar(0,255,255,255), 5); //draw gt target centroid //cv::Scalar(r,g,b,opacity)	
					    
						//draw bounding box of the target
				
					    
					    oGTTranslator.GetGTBoundingBox(oTargetPos_HX.x, oTargetPos_HX.y, nBBoxWidth, nBBoxHeight, bb);
						//cv::rectangle(oCurrView, bb, cv::Scalar(0,255,255,255)); //draw gt bounding box

					


					    ++oGTFrame;

					}else{
						bb = cv::Rect(cv::Point(0,0), cv::Point(0,0));
						oTargetPos_XY.x = -1;
						oTargetPos_XY.y = -1;
					}

				    

					
				}else{
				    bb = cv::Rect(cv::Point(0,0), cv::Point(0,0));
					oTargetPos_XY.x = -1;
					oTargetPos_XY.y = -1;
				}



				//cv::circle(oCurrView, cv::Point(oCurrView.cols/2,oCurrView.rows/2), 3, cv::Scalar(0,255,0), 5); //draw camera centroid
				
				
				
		
				cv::imshow("Current View", oCurrView);
				
				//save the frame
				
				char pathFolderFramesPTZ[200]; 
				char pathTempFolderFramesPTZ[200]; 
				sprintf(pathTempFolderFramesPTZ, "/home/icai21/Modelos/codigos/anomalous_detection/temp/%s/output_PTZ_frames/temp%06d.png", VIDEO_NAME, nFrameIdx); 
				//this works if the folder is manually before created
				std::cout << "Created frame: " << pathTempFolderFramesPTZ << std::endl;
				cv::imwrite(pathTempFolderFramesPTZ,oCurrView);
				
				sprintf(pathFolderFramesPTZ, "/home/icai21/Modelos/codigos/anomalous_detection/temp/%s/output_PTZ_frames/in_%06d.png", VIDEO_NAME, nFrameIdx); 
cout << pathFolderFramesPTZ;
				if(!rename(pathTempFolderFramesPTZ,pathFolderFramesPTZ)==0)// rename the file
				{
        			printf("Error renaming the file\n");
cout << stdout;
					break;
				}
				
				
				if (initializedModel<1){
					initializedModel++;
				}else{
					frameProcessing(&oCamera,oCurrView,nFrameIdx,oTargetPos_XY,bb); //quitar el ultimo parametro (se obtiene dentro del metodo despues del deep learning)
				}
				
				
				
				char cKey = cv::waitKey(1);
				/*if(cKey==' ')
					bPaused = !bPaused;
				else */if(cKey!=-1)
					break;
				if(!bPaused)
					break;
			}
        }
        return 0;
    }
    catch(const std::exception& e) {
        std::cerr << "top level caught std::exception:\n" << e.what() << std::endl;
    }
    catch(...) {
        std::cerr << "top level caught unknown exception." << std::endl;
    }
}
