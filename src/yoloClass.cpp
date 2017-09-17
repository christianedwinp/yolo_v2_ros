#include <stdio.h>
#include <cv.h>
#include <vector>
#include <highgui.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include "opencv2/video.hpp"
#include "opencv2/imgproc.hpp"
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/gpu/gpu.hpp"


#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
//#include <htf_safe_msgs/SAFEObstacleMsg.h>
#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <boost/algorithm/string.hpp>
//#include <camera_info_manager/camera_info_manager.h>
#include <math.h>












#include<boundingbox_msgs/Boundingbox.h>
extern "C" {


#include "yoloInterface.h"


#include "network.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "demo.h"
#include "list.h"
#include "option_list.h"
#include "blas.h"
}

//using namespace std;
using namespace cv;
using namespace cv::gpu;
float remapYolo2NewObjectTypes[] = {0, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
class MyNode {
public:
	MyNode() :
		nh("~"), it(nh) {
			nh.param<std::string>("basedir",basedir,"/folder/of/ros/node");
			nh.param<std::string>("model_cfg",model_cfg,"/cfg/yolo.cfg");
			nh.param<std::string>("weightfile",weightfile,"/weights/yolo.weights");
			nh.param<std::string>("datafile",datafile,"/cfg/coco.data");
                        //nh.param<std:;string>("bound")
			nh.param<bool>("visualize_detections",visualizeDetections,true);

			nh.param<std::string>("topic_name",topic_name,"/usb_cam/image_raw");
			nh.param<float>("threshold",threshold,0.2);
			std::vector<std::string> strParts;
			boost::split(strParts,topic_name,boost::is_any_of("/"));

			model_cfg = basedir+model_cfg;
			weightfile = basedir+weightfile;
			datafile = basedir+datafile;

			// Distance estimate.
			nh.param<double>("FOV_verticalDeg",FOV_verticalDeg,47.0);
			nh.param<double>("FOV_horizontalDeg",FOV_horizontalDeg,83.0);
			nh.param<double>("angleTiltDegrees",angleTiltDegrees,7.0);
			nh.param<double>("cameraHeight",cameraHeight,1.9);

			FOV_verticalRad = FOV_verticalDeg*M_PI/180;
			FOV_horizontalRad = FOV_horizontalDeg*M_PI/180;
			angleTiltRad = angleTiltDegrees*M_PI/180;

			//cinfor_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh, "test", ""));
			//sub_image = it.subscribeCamera(topic_name.c_str(), 1, &MyNode::onImage, this);

			sub_image = it.subscribe(topic_name.c_str(), 1, &MyNode::onImage, this);
			pub_image = it.advertise("imageYolo", 1);

			std::vector<std::string> outputTopicTmp;
			outputTopicTmp.push_back("AAox");
			outputTopicTmp.push_back(strParts[1]);
			//pub_bb = nh.advertise<std_msgs::Float64MultiArray>(boost::algorithm::join(outputTopicTmp,"/"), 1);
                        pub_bb=nh.advertise<boundingbox_msgs::Boundingbox>(boost::algorithm::join(outputTopicTmp,"/"),1);
			readyToPublish = 1;

			useRemapping = 1;
			options = (list *)read_data_cfg((char*)datafile.c_str());
			std::string name_list = option_find_str(options, "names", "data/names.list");

			name_list = basedir+ '/' + name_list;
			names = get_labels((char*)name_list.c_str());

			maxDetections = load_yolo_model((char*)model_cfg.c_str(), (char*)weightfile.c_str());
			nClasses = get_nclasses();

			boxes = (box*)calloc(maxDetections, sizeof(box));
			probs = (float**)calloc(maxDetections, sizeof(float *));
			for(int j = 0; j < maxDetections; ++j) probs[j] = (float*)calloc(nClasses + 1, sizeof(float *));

			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TOPIC: %s \r\n",topic_name.c_str());
		};

	~MyNode() {
		free(boxes);
		free(probs);
		for(int j = 0; j < maxDetections; ++j) free(probs[j]);
	}
	;

Point GetWrappedPoint(Mat M, const Point& p)
{
    cv::Mat_<double> src(3/*rows*/,1 /* cols */);
         src(0,0)=p.x;
   src(1,0)=p.y;
    src(2,0)=1.0;

    cv::Mat_<double> dst = M*src;
    dst(0,0) /= dst(2,0);
        dst(1,0) /= dst(2,0);
     return Point(dst(0,0),dst(1,0));
 }

/*void lane_detection(Mat & mFrame, int flag){

     int level=0,a=mFrame.rows;
     Mat   roi_Gray,src,mFrame_gray,imageROI,IPM_ROI,IPM,IPM_Gray;
     Mat   IPM1,IPM2,IPM_Blur;
     GpuMat frame_gpu,gray_gpu,resized_gpu,cars_buf;
     if(flag==1){


     src=mFrame.clone();
     imageROI = mFrame(Rect(0,mFrame.rows/2,mFrame.cols,mFrame.rows/2));
     IPM_ROI = imageROI(Rect(0,65,imageROI.cols,(imageROI.rows-65)));
     IPM_ROI = IPM_ROI.clone();  //????????????

     cvtColor(mFrame,mFrame_gray,COLOR_BGR2GRAY);
     cvtColor(imageROI, roi_Gray, COLOR_BGR2GRAY);
     //namedWindow("src",0);
     //imshow("src",roi_Gray);


//IPM
        Point2f inputQuad[4];
        Point2f outputQuad[4];

       // imshow("3333", IPM_ROI);

        Mat IPM_Matrix( 2, 4, CV_32FC1 );
        Mat IPM_Matrix_inverse;
        IPM_Matrix = Mat::zeros( mFrame.rows, mFrame.cols, mFrame.type() );

        inputQuad[0] = Point2f( 0,0);
        inputQuad[1] = Point2f( IPM_ROI.cols,0);
        inputQuad[2] = Point2f( IPM_ROI.cols,IPM_ROI.rows);
        inputQuad[3] = Point2f( 0,IPM_ROI.rows);

        outputQuad[0] = Point2f( 0,0 );
        outputQuad[1] = Point2f( mFrame.cols,0);
        outputQuad[2] = Point2f( mFrame.cols-240,mFrame.rows);
        outputQuad[3] = Point2f( 240,mFrame.rows);


       IPM_Matrix = getPerspectiveTransform( inputQuad, outputQuad );
       invert(IPM_Matrix,IPM_Matrix_inverse);
       warpPerspective(IPM_ROI,IPM,IPM_Matrix,mFrame.size() );
       //imshow("test",mFrame);
       imshow("IPM",IPM);
       cvtColor(IPM, IPM_Gray, COLOR_BGR2GRAY);
       GaussianBlur(IPM_Gray, IPM_Gray, Size(7,7), 1.5, 1.5);
       Canny(IPM_Gray, IPM_Gray, 12, 90, 3);

      IPM.copyTo(IPM1);
        IPM.copyTo(IPM2);



        for (int i=0; i<IPM.rows; i++){
            uchar* data= IPM.ptr<uchar>(i);
            for (int j=0; j<IPM.cols; j++)
            {
                if(i<0 || i>480)
                {
                    // process each pixel
                    data[j]= data[j]>level?level:0;
                }else{
                    if(data[j]<=255 && data[j]>240 ){
                        for(int m=j;m<j+20;m++){
                            a=m;
                            data[m]=0;
                        }
                        j=a;
                        break;
                    }
                }
            }
        }

        for (int i=0; i<IPM.rows; i++){
            uchar* data= IPM.ptr<uchar>(i);
            for(int j=IPM.cols;j>0;j--){
                if(data[j]<=255 && data[j]>240){
                    for(int m=j;m>j-20;m--){
                        data[m]=0;
                    }
                    j=j-20;
                    break;
                }
            }
        }

        GaussianBlur(IPM_Gray,IPM_Blur,Size(5,5),1.5,1.5);
        //Houghline
        GpuMat d_IPM_Gray(IPM_Blur);
        GpuMat lines_gpu;
        HoughLinesBuf d_buf;{
      //  const int64 start = getTickCount();
        gpu::HoughLinesP(d_IPM_Gray,lines_gpu,d_buf,1.0f,(float)(CV_PI/180.0f),50,50);


       // const double timeSec = (getTickCount() - start) / getTickFrequency();


                            }
        vector<Vec4i>lines;
        if(!lines_gpu.empty())
 {
         lines.resize(lines_gpu.cols);
         Mat d_lines(1,lines_gpu.cols,CV_32SC4,&lines[0]);
         lines_gpu.download(d_lines);

 }



        vector<Point> laneShade,laneShade1,laneShade2;
       	float d=0.00,d1=0.00;
        int s=0;
        int n=mFrame.cols;
        Point e,f,g,h,A,B,C,D;
        float angle;float a;
        for( size_t i = 0; i < lines.size(); i++ ){
            float p=0,t=0;
            Vec4i l = lines[i];
            if((l[0]-l[2])==0){
                a=-CV_PI/2;
                angle=-90;
            }else{
                t=(l[1]-l[3])/(l[0]-l[2]);
                a=atan(t);
                angle=a*180/CV_PI;
            }

            if(angle>10 ||  angle<(-10)){

                p=(l[0]+l[2])/2;
                line(IPM1,Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 3,CV_AA);
                if(p<320){
                    if(p>s){
                        s=p;
                        d=320-(s);
                        e=Point(l[0],l[1]);
                        f=Point(l[2],l[3]);
                        A= GetWrappedPoint(IPM_Matrix_inverse,e);
                        B =GetWrappedPoint(IPM_Matrix_inverse,f);
                        A.y += 245;
                        B.y += 245;

                        double lengthAB = sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
                        if(A.y > B.y){
                        A.x = B.x + (B.x - A.x) / lengthAB * -350;
                        A.y = B.y + (B.y - A.y) / lengthAB * -350;

                        }else{
                            B.x = B.x + (B.x - A.x) / lengthAB * 350;
                            B.y = B.y + (B.y - A.y) / lengthAB * 350;

                        }

                    }

                }
                if(p>320){
                    if(p<n){
                        n=p;
                        d1=(n)-320;
                        g=Point(l[0],l[1]);
                        h=Point(l[2],l[3]);
                        C= GetWrappedPoint(IPM_Matrix_inverse,g);
                        C.y +=245;
                        D =GetWrappedPoint(IPM_Matrix_inverse,h);
                        D.y +=245;
                        double lengthCD = sqrt((C.x - D.x)*(C.x - D.x) + (C.y - D.y)*(C.y - D.y));
                        if(C.x > D.x){
                        C.x = D.x + (D.x - C.x) / lengthCD * -350;
                        C.y = D.y + (D.y - C.y) / lengthCD * -350;
                        }else{
                            D.x = D.x + (D.x - C.x) / lengthCD * +350;
                            D.y = D.y + (D.y - C.y) / lengthCD * +350;

                        }
                    }

                }

            }
        }

         imshow("test",mFrame);
        line(IPM2,e, f, Scalar(0,155,255), 3,CV_AA);
        line(IPM2,g, h, Scalar(0,145,255), 3,CV_AA);

        if(A.x < B.x){
            laneShade.push_back(B);
            laneShade.push_back(A);
        }else{
            laneShade.push_back(A);
            laneShade.push_back(B);
        }

        if(C.x > D.x){
            laneShade.push_back(C);
            laneShade.push_back(D);
        }else{
            laneShade.push_back(D);
            laneShade.push_back(C);
        }

        laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[0].y+20));
        laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2 +45,laneShade[1].y));
        laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2 -45,laneShade[2].y));
        laneShade1.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[3].y+20));

        laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[0].y+20));
        laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2 +25,laneShade[2].y));
        laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2 -25,laneShade[2].y));
        laneShade2.push_back(Point((laneShade[0].x+laneShade[3].x)/2,laneShade[3].y+20));


        Point zero  = Point(0,0);
        if(laneShade[0]!=zero && laneShade[1]!=zero && laneShade[2]!=zero && laneShade[3]!=zero && laneShade[2].y>0){
        Mat laneMask= mFrame.clone();
        fillConvexPoly(laneMask, laneShade, Scalar(0,200,0));  //(255,144,30)
        fillConvexPoly(mFrame, laneShade1, Scalar(0,200,0));
        fillConvexPoly(mFrame, laneShade2, Scalar(255,255,255));
        addWeighted(mFrame, 0.6, laneMask, 0.4, 3, mFrame);
        }


        //imshow("HOUGH BEFORE FILTERING",IPM1);
       // imshow("HOUGH AFTER FILTERING",IPM2);





       // imshow("1", mFrame);
       // imshow("2", src);






       waitKey(30);

}

 else{

 printf("lane_detection is closed");
}

}
*/





void onImage(const sensor_msgs::ImageConstPtr& msg) {
		printf("Yolo: image received \r\n");
		if(readyToPublish==1)
		{
			readyToPublish = 0;

			cv_bridge::CvImagePtr cv_ptr;
			try {
				cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
			} catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}

			// Convert to Darknet image format.
			image im = OpencvMat2DarkNetImage(cv_ptr->image);
                       //--- lane_detection(cv_ptr->image,1);
			execute_yolo_model2(im, threshold,boxes, probs); // Returns bounding boxes and probabilities.

			publish_detections(cv_ptr->image, maxDetections, threshold, boxes, probs,names);

			free_image(im);
			readyToPublish = 1;
		}
	}

	// Roughly the same as ipl_to_image() in image.c
	image OpencvMat2DarkNetImage(Mat src)
	{
	    unsigned char *data = (unsigned char *)src.data;
	    int h = src.rows;
	    int w = src.cols;
	    int c = src.channels();
	    int step = src.step1();
	    image out = make_image(w, h, c);
	    int i, j, k, count=0;;

	    for(k= 0; k < c; ++k){
	    //for(k= c-1; k >= 0; --k){
		for(i = 0; i < h; ++i){
		    for(j = 0; j < w; ++j){
		        out.data[count++] = float(data[i*step + j*c + k]/255.);
		    }
		}
	    }
	    return out;
	}
	Mat publish_detections(Mat img, int num, float thresh, box *boxesIn, float **probsIn, char **names)
	{
		int i;
		int cDetections = 0;
                double distance =0;
		box_prob* detections = (box_prob*)calloc(maxDetections, sizeof(box_prob));
		//printf("Number of bounding boxes %i: \n", num);
		for(i = 0; i < num; ++i){
			int topClass = max_index(probs[i],nClasses);
			double prob = probs[i][topClass];
			if(prob > thresh){
				int width = pow(prob, 1./2.)*10+1; // line thickness
				box b = boxes[i];
				 Scalar useColor(0, 0, 0);
				float x  = (b.x-b.w/2.)*(float)(img.cols);
				float y = (b.y-b.h/2.)*(float)(img.rows);
				float w   = b.w*(float)(img.cols);
				float h   = b.h*(float)(img.rows);
                                distance=(0.0397*0.5)/(w*0.00007);


                              if(0==strcmp(names[topClass],"person")){
                                                printf("dis: %f \n",distance);
                                               }

                                printf("bb: %f %f %f %f \n", x,y,w,h);

                                printf("%s: %f%% \n", names[topClass], prob*100);






                                if(visualizeDetections){
					rectangle(img, Rect(x,y,w,h), useColor, 2, 8, 0);
					char numstr[30];
					sprintf(numstr, "%s %.2f",names[topClass], prob);
					putText(img, numstr, Point(x+4,y-14+h),FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2, 8, false);
				}

				// Detection in x and y coordinates (with x, y as upper left corner)

                                detections[cDetections].x = x;
				detections[cDetections].y = y;
				detections[cDetections].w = w;
				detections[cDetections].h = h;
				detections[cDetections].prob = prob;
				//detections[cDetections].objectType =topClass;
                                detections[cDetections].objectName =names[topClass];

				cDetections++;

			}
		}


		/* Creating visual marker
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/laser";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;*/


		// An estimate of the distance to the object is calculated using the camera setup.
		// Estimate is based on two assumptions: 1) The surface is flat. 2) The bottom of the bounding box is the bottom of the detected object.
		/*if(1){
			printf("Start bboxSAFE \n");
			double resolutionVertical = img.rows;
			double resolutionHorisontal = img.cols;
			//htf_safe_msgs::SAFEObstacleMsg msgObstacle;
			// MAYBE CLEARING IS NEEDED
			msgObstacle.xCoordinate.clear();
			msgObstacle.yCoordinate.clear();
			msgObstacle.zCoordinate.clear();
			msgObstacle.quality.clear();
			msgObstacle.objectType.clear();
			msgObstacle.objectID.clear();

			msgObstacle.header.stamp = ros::Time::now();

			for (int n = 0; n < cDetections;n++){
				double buttomRowPosition = detections[n].y+detections[n].h; // bbs(n,2)-bbs(n,4);
				double ColPosition = detections[n].x+detections[n].w/2; // bbs(n,2)-bbs(n,4);

				double distance = tan(M_PI/2-(angleTiltRad+FOV_verticalRad/2) + FOV_verticalRad*(resolutionVertical-buttomRowPosition)/resolutionVertical)*cameraHeight;
				double angle =((ColPosition-resolutionHorisontal/2)/resolutionHorisontal)*FOV_verticalRad;
				double xCoordinate = cos(angle)*distance;
				double yCoordinate = sin(angle)*distance;
				msgObstacle.xCoordinate.push_back(xCoordinate);
				msgObstacle.yCoordinate.push_back(yCoordinate);
				msgObstacle.zCoordinate.push_back(0.0);
				msgObstacle.quality.push_back(detections[n].prob);
				msgObstacle.objectType.push_back(detections[n].objectType);
				msgObstacle.objectID.push_back(0);
				//cout << "x1:" << bbs[n].x1 << ", y2:" << bbs[n].y2 << ", w3:" << bbs[n].width3 << ", h4:" << bbs[n].height4 << ", s5: " << bbs[n].score5 << ",a5: " << bbs[n].angle << endl;
				//cout << "Distance: " <<  bbs[n].distance << endl;
			}
			pub_bbSAFE.publish(msgObstacle);
		}*/


		// Create bounding box publisher (multi array)

/* ============================
		std_msgs::Float64MultiArray bboxMsg;
		bboxMsg.data.clear();

		for (int iBbs = 0; iBbs < cDetections; ++iBbs) {

			bboxMsg.data.push_back(detections[iBbs].x);// /img.cols);
			bboxMsg.data.push_back(detections[iBbs].y);// /img.rows);
			bboxMsg.data.push_back(detections[iBbs].w);// /img.cols);
			bboxMsg.data.push_back(detections[iBbs].h);// /img.rows);

                        bboxMsg.data.push_back(detections[iBbs].prob);
                        bboxMsg.data.push_back(int(detections[iBbs].objectType));
			if(useRemapping){
				bboxMsg.data.push_back(remapYolo2NewObjectTypes[int(detections[iBbs].objectType)]);
			}
			else{
				bboxMsg.data.push_back(int(detections[iBbs].objectType));
			}



		}
		pub_bb.publish(bboxMsg);
==============================*/

/*  peter data type */

          boundingbox_msgs::Boundingbox boundingBox;

           for (int iBbs=0; iBbs<cDetections; ++iBbs){

                      boundingBox.x=detections[iBbs].x;
                      boundingBox.y=detections[iBbs].y;
                      boundingBox.w=detections[iBbs].w;
                      boundingBox.h=detections[iBbs].h;
                      boundingBox.prob=detections[iBbs].prob;
                     // boundingBox.objectType=detections[iBbs].objectType;

                      boundingBox.objectName=detections[iBbs].objectName;
                         pub_bb.publish(boundingBox);



}







	// Create image publisher showing yolo detections.
		if(visualizeDetections){

			//sensor_msgs::CameraInfoPtr cc(new sensor_msgs::CameraInfo(cinfor_->getCameraInfo()));
			sensor_msgs::ImagePtr msg_image_out = cv_bridge::CvImage(std_msgs::Header(),"bgr8", img).toImageMsg();
			msg_image_out->header.stamp = ros::Time::now();
			//pub_image.publish(msg_image_out, cc);
			pub_image.publish(msg_image_out);
			//namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.
			//imshow( "Display window", img);
		}
		free (detections);
		return img;
	}
private:
	double imageResize;
	float threshold;
	std::string basedir;
	std::string model_cfg;
	std::string weightfile;
	std::string topic_name;
	std::string datafile;
        //std::string boundingBoxesTopicName
	bool visualizeDetections;

	cv::Mat img;
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	//image_transport::CameraPublisher pub_image;
	//image_transport::CameraSubscriber sub_image;
	image_transport::Publisher pub_image;
	image_transport::Subscriber sub_image;
	ros::Publisher pub_bb;
	ros::Publisher pub_bbSAFE;
	//boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfor_;
	bool readyToPublish;
	image **alphabet;
	box *boxes;
	float **probs;
	char ** names; // Name of all classes.
	bool useRemapping;
	list *options;

	int maxDetections;
	int nClasses;
	double FOV_verticalDeg,FOV_horizontalDeg,angleTiltDegrees,cameraHeight;
	double FOV_verticalRad, FOV_horizontalRad,angleTiltRad;
};


int main(int argc, char** argv) {

	ros::init(argc, argv, "darknet_stuff");

	MyNode node;

	ros::spin();
}
