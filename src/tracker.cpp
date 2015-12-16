#include "tracker.h"
#include <Eigen/Geometry>
using namespace pcl;


Tracker::Tracker()
{
    width = 320;
    height = 240;
    fx= 285.17f; fy= 285.17f; cx=159.5; cy=119.5f; //asus camera parameters for 320x240 resolution
    isDebug = true;
    m_cnt = 0;
    m_headModel.reset(new Cloud);
    m_headPose=Affine3f::Identity();

    String face_cascade_name = "haarcascade_frontalface_alt.xml";
    //1. Load the cascades
    if( !face_cascade.load( face_cascade_name ) )
    {
        printf("--(!)Error loading\n");
        printf("--(!)Error loading\n");
        printf("--(!)Error loading\n");
    }
    else
    {
        printf("--(!)succeed in loading\n");
        printf("--(!)succeed in loading\n");
        printf("--(!)succeed in loading\n");
    }

}

Tracker::~Tracker()
{

}

void Tracker::run(CloudPtr inputCloud, Mat inputImage)
{
    //cout<<"Timestamp: "<<inputCloud->header.stamp<<endl;
    //TODO: for first frame, detect face from inputImage, and extract corresponding head Point cloud
    //if (m_cnt==0){
    extractHeadModel(inputImage, inputCloud);
    cout << "Tracker::run"<<endl;
    //}
    //TODO: estimate the current head pose with Iterative Closest Point algorithm from PCL
    //m_headPose= .........
    m_cnt++;
}

void Tracker::extractHeadModel(Mat &image, CloudPtr inputCloud)
{
    m_headModel.reset(new Cloud);
    //your code to detect face and extract head
    std::vector<Rect> faces;
    Mat frame = image;
    Mat frame_gray;

    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

    cout << faces.size() << endl;
    for( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
        ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }
    //-- Show what you got

    string window_name = "Capture - Face detection";
    imshow( window_name, frame_gray );
    waitKey(10);
}

CloudPtr Tracker::getTransformedHeadModel()
{
    CloudPtr temp;
    temp.reset(new Cloud);
    transformPointCloud(*m_headModel, *temp, m_headPose);
    return temp;
}

geometry_msgs::PoseStamped  Tracker::getHeadPose()
{
    geometry_msgs::PoseStamped poseOutput;
    poseOutput.pose.position.x = m_headPose(0,3);
    poseOutput.pose.position.y = m_headPose(1,3);
    poseOutput.pose.position.z = m_headPose(2,3);
    Quaternion<float> q(m_headPose.rotation()); q.normalize();
    poseOutput.pose.orientation.x = q.x();
    poseOutput.pose.orientation.y = q.y();
    poseOutput.pose.orientation.z = q.z();
    poseOutput.pose.orientation.w = q.w();
    return poseOutput;
}
