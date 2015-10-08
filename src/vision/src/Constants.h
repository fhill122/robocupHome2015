/**Generated by core/generateAllConstants
 */

///Common
//table relative z position to 0, measured by end point position
#define TableZ -0.152
#define GripperLength 0.144

///Module specific
#define CALIBRATION_FILE "/src/vision/data/Calibration.ini"
#define CALIBRATION_IMAGE "/src/vision/data/Calibration.jpg"
#define OBJECTS_FILE "/src/vision/data/Objects.ini"
#define DATA_FOLDER "/src/vision/data"

//webcam: "usb_cam/image_raw", kinect:"/camera/rgb/image_raw"
#define IMAGE_TOPIC "/camera/rgb/image_raw"

//original table block size
//TableBlockY=0.2
//TableBlockX=0.15
//paper block size
//A3 paper Y=0.42 x=0.297
//block = y/4 ,x/3 
#define TableBlockY 0.105
#define TableBlockX 0.099
//pull from bolt to point 5
#define BaseToPoint5 0.265
#define BaseX 0.155
#define CameraZ 0.8
#define CameraX 0.287
