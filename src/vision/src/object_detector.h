using namespace std;
using namespace cv;

class DesktopObject {
    string name;
    double height;
    double size;
    vector<Mat> images;
    vector<float> matchDistance;
    vector<int> matchNumber;
    
    public:
        DesktopObject(string n,double h,double s){
            name=n;
            height=h;
            size=s;
        }
        string getName(){return name;}
        void addImages(Mat image,float distance, int number){
            images.push_back(image);
            matchDistance.push_back(distance);
            matchNumber.push_back(number);
        }
        vector<Mat> & getImages(){
            return images;
        }
        float getMatchDistance(int i){ return matchDistance[i];}
        int getMatchNumber(int i){return matchNumber[i];}
        double getHeight(){return height;}
        double getSize(){return size;}
        
};

/** Function Headers */
int detectAndDisplay( Mat frame, Mat obj, float matchDistance, int matchNumber, vector<KeyPoint> keypoints_object, Mat descriptors_object,vision::platePosition::Response &res, Mat H2, double h);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);//copy image from topic to global variable frame
bool get_object_position(vision::platePosition::Request &req, vision::platePosition::Response &res); //service function
bool displayFrame(vision::platePosition::Request &req, vision::platePosition::Response &res);
int SIFTfeatureCalculate(Mat &img, vector<KeyPoint> &keypoints,Mat &descriptors );
Mat readCalibration(ifstream &file);
string findParameter(string line, string name);
int readObjectsDataFile(ifstream &file, vector <DesktopObject> &objects);
template <typename T> string tostr(const T& t) { 
   ostringstream os; 
   os<<t; 
   return os.str(); 
} 

