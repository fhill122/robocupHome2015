/**Generated by core/generateAllConstants
 */

///Common
//table relative z position to 0, measured by end point position
#define TableZ -0.152
#define GripperLength 0.144
#define poly_dual   [(0,-0.36),(0.35,-0.26),(0.55,0),(0.35,0.26),(0,0.36)]
#define poly_right   [(0.1,0.8),(0.3,-0.33),(0.6,-0.5),(0.66,0.22),(0.5,0.7),(0.2,0.88),(0,0.9),(0,0.44),(0.2,0.4),(0.2,0.2)]
#define poly_left   [(0.1,-0.08),(0.2,-0.2),(0.2,-0.4),(0,-0.44),(0,-0.9),(0.2,-0.88),(0.5,-0.7),(0.66,-0.22),(0.6,0.5),(0.3,0.33)]

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
#define TableBlockY 0.1805
#define TableBlockX 0.1405
//pull from bolt to point 5
#define BaseToPoint4 0.265
#define BaseX 0.155
#define CameraZ 0.8
#define CameraX 0.287
