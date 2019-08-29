#include <boost/function.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>

ros::Subscriber sub_;
image_transport::Publisher  pub_;
//image_transport::Publisher pub_image_;
//ros::Publisher pub_;
//cv_bridge::CvImage bridge_;
//std_msgs::Float32MultiArray array;

#define CLAMP(x, a, b)   ( (x) > (b) ? (b) : ( (x) < (a) ? (a) : (x) ) )
#define AR_RESIZE_WIDTH     240
#define AR_RESIZE_HEIGHT    240

#define nTARGETCOLOR    2
CvScalar targetColor[nTARGETCOLOR] = {
    CV_RGB(0,255,0),    // 敵のQRコード枠色(green)
    CV_RGB(0,0,255)     // 障害物のQRコード枠色(blue)
};

int tolerance = 80;             // 色許容値
typedef struct _STR_AR_area_
{
	int flag;
	int xmin;
	int ymin;
	int xmax;
	int ymax;
}STR_AR_area;

int pre_ar_area_id = -1;
STR_AR_area ar_areas_[4] = {};

int xmin = 0, ymin = 0;
int xmax = 0, ymax = 0;

inline double get_red(CvScalar color){return color.val[2];}
inline double get_green(CvScalar color){return color.val[1];}
inline double get_blue(CvScalar color){return color.val[0];}

int is_my_area(STR_AR_area* areaInfo, int x, int y)
{
	int is_my_area = false;
	if(areaInfo->flag == false)
	{
		is_my_area = true;
	}
	else
	{
		if((abs(x-areaInfo->xmin) < 115) && (abs(y-areaInfo->ymin) < 115))
		{
			is_my_area = true;
		}
	}

/*
		if(detection == true)
		{
                    if(abs(x-xmin) < 150)
                    {
                        xmin = std::min(x, xmin);    //左端（X最小値）
                    }
                    if(abs(y-ymin) < 150)
                    {
                        ymin = std::min(i, ymin);      //上端（Y最小値）
                    }
                    if(x < (xmin+150) && i < (ymin+150))
                    {
                        xmax = std::max(x, xmax);    //右端（X最大値）
                        ymax = std::max(i, ymax);      //下端（Y最大値）
                    }
		    
                    else
                    {
                        xmin2 = std::min(x,xmin2);
			ymin2 = std::min(i,ymin2);
			if(x-xmin2 < 150)
		        {
				xmax2 = std::max(x,xmax2);
                    }
                }
                else
                {
                    //フラグを物体検知有りにする
                    detection = true;

                    //左端、右端のX座標、上端、下端のY座標を導く
                    //今回の値と今までの値を比較し、最小値、最大値を調べる
                    xmin = std::min(x, xmin);    //左端（X最小値）
                    xmax = std::max(x, xmax);    //右端（X最大値）
                    ymin = std::min(i, ymin);      //上端（Y最小値）
                    ymax = std::max(i, ymax);      //下端（Y最大値）
                }
*/
	return is_my_area;
}

bool sakuAR(cv_bridge::CvImagePtr cv_msg, CvScalar searchColor)
{
    bool detection = false;

    int scan_start_y = 80;
    int sca_end_y = 80;
    
    for(int i = scan_start_y; i < cv_msg->image.rows-sca_end_y; i++)
    {
        int y = i*cv_msg->image.step;
	pre_ar_area_id = -1;

        for(int x = 0; x < cv_msg->image.cols; x++)
        {
            int difRed = abs(get_red(searchColor)-cv_msg->image.data[y+(x*3)+2]);
            int difGreen = abs(get_green(searchColor)-cv_msg->image.data[y+(x*3)+1]);
            int difBlue = abs(get_blue(searchColor)-cv_msg->image.data[y+(x*3)+0]);

            //ROS_INFO("difRed=%d, difGreen= %d, difBlue=%d", difRed, difGreen, difBlue);

            //RGB各色が許容値以内の場合（近似色である場合）
            if((difRed < tolerance && difGreen < tolerance && difBlue < tolerance))
            {
		if(detection == true)
		{
                    if(abs(x-xmin) < 150)
                    {
                        xmin = std::min(x, xmin);    //左端（X最小値）
                    }
                    if(abs(y-ymin) < 150)
                    {
                        ymin = std::min(i, ymin);      //上端（Y最小値）
                    }
                    if(x < (xmin+150) && i < (ymin+150))
                    {
                        xmax = std::max(x, xmax);    //右端（X最大値）
                        ymax = std::max(i, ymax);      //下端（Y最大値）
                    }
		    
                    else
                    {
                       break;
                    }
                }
                else
                {
                    //フラグを物体検知有りにする
                    detection = true;

                    //左端、右端のX座標、上端、下端のY座標を導く
                    //今回の値と今までの値を比較し、最小値、最大値を調べる
                    xmin = std::min(x, xmin);    //左端（X最小値）
                    xmax = std::max(x, xmax);    //右端（X最大値）
                    ymin = std::min(i, ymin);      //上端（Y最小値）
                    ymax = std::max(i, ymax);      //下端（Y最大値）
                }
		//detection = true;
	/*	int ok_area[4] = {};
             	for(int loop = 0; loop < 4; loop++)
		{
			ok_area[loop] = is_my_area(&ar_areas_[loop], x, i);
		}
		int num_of_my_area = 0;
		int my_area_id = -1;
             	for(int loop = 0; loop < 4; loop++)
		{
			if(ok_area[loop] == true)
			{
				num_of_my_area++;
				my_area_id = loop;
			}
		}
		if(num_of_my_area > 1)
		{
			//ROS_INFO("num_of_my_area=%d",num_of_my_area);
		/*	if(pre_ar_area_id != -1)
			{
				my_area_id = pre_ar_area_id;
			}*/
		/*}
		if(my_area_id != -1)
		{
			// update xmin, ymin, xmax, ymax
			ar_areas_[my_area_id].flag = true;
			ar_areas_[my_area_id].xmin = std::min(x, ar_areas_[my_area_id].xmin);
			ar_areas_[my_area_id].xmax = std::max(x, ar_areas_[my_area_id].xmax);
			ar_areas_[my_area_id].ymin = std::min(i, ar_areas_[my_area_id].ymin);
			ar_areas_[my_area_id].ymax = std::max(i, ar_areas_[my_area_id].ymax);
			//pre_ar_area_id = my_area_id;
			//ROS_INFO("update x = %d, y = %d, xmin = %d, xmax = %d, ymin = %d, ymax = %d",x,i,ar_areas_[my_area_id].xmin, ar_areas_[my_area_id].xmax, ar_areas_[my_area_id].ymin, ar_areas_[my_area_id].ymax);
			//ROS_INFO("my_area_id = %d",my_area_id);
		}*/
            }
        }
    }

    return detection;
}

void adjust_bright(cv_bridge::CvImagePtr cv_msg, double brightScale)
{
    double b, g, r;

    int scan_start_y = 80;
    int sca_end_y = 80;

    for(int i = scan_start_y; i < cv_msg->image.rows-sca_end_y; i++) 
    {
        int y = i*cv_msg->image.step;
        int index = 0;

        for(int x = 0; x < cv_msg->image.cols; x++)
        {
            index = y+(x*3);
            b = cv_msg->image.data[index+2];
            g = cv_msg->image.data[index+1];
            r = cv_msg->image.data[index+0];
	    cv_msg->image.data[index+2] = CLAMP(b + (sqrt(b) - b) * brightScale, 0, 255);
            cv_msg->image.data[index+1] = CLAMP(g + (sqrt(g) - g) * brightScale, 0, 255);
            cv_msg->image.data[index+0] = CLAMP(r + (sqrt(r) - r) * brightScale, 0, 255);
        }
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("get image");
  /* ImageトピックをCvImage型に変換 */
  cv_bridge::CvImagePtr cv_msg = cv_bridge::toCvCopy(msg, "bgr8");

  //////////////////////////////////////////
  // ここにイメージ処理を実装する
    bool detection = false;
    int x = 0, y = 0;

    xmin = cv_msg->image.cols;
    ymin = cv_msg->image.rows;
    xmax = 0;
    ymax = 0;
/*
	for(int loop = 0; loop < 4; loop++)
	{
		ar_areas_[loop].flag = false;	
		ar_areas_[loop].xmin = cv_msg->image.cols;
		ar_areas_[loop].ymin = cv_msg->image.rows;
		ar_areas_[loop].xmax = 0;
		ar_areas_[loop].ymax = 0;
	}
*/
    adjust_bright(cv_msg, -0.75);

    // ARコードの枠色を探す
    detection = sakuAR(cv_msg, targetColor[0]);
    if(detection == false)
    {
        detection = sakuAR(cv_msg, targetColor[1]);
    }

    if(detection == true)
    {
	//for(int i = 0; i < 4; i++)
	//{
	//ROS_INFO("id = %d, flag = %d", i, ar_areas_[i].flag);
	/*	if(ar_areas_[i].flag == true)
		{
			xmin = ar_areas_[i].xmin;
			xmax = ar_areas_[i].xmax;
			ymin = ar_areas_[i].ymin;
			ymax = ar_areas_[i].ymax;*/
			if(xmax-xmin > 0 && ymax-ymin > 0)
			{
			//ROS_INFO("id = %d , xmin = %d, xmax = %d, ymin = %d, ymax = %d",i,xmin,xmax,ymin,ymax);
				

				cv::Mat subImg(cv_msg->image, cv::Rect(xmin, ymin, xmax-xmin, ymax-ymin));
				cv::resize(subImg, subImg, cv::Size(AR_RESIZE_WIDTH, AR_RESIZE_HEIGHT));
			 
				// ここにsubImgの補正処理を入れる
				//cv::threshold(subImg, subImg, 100, 255, CV_THRESH_BINARY);
				/*if(i==3){
				cv::Mat roi_dst1 = cv_msg->image(cv::Rect(0,0,AR_RESIZE_WIDTH, AR_RESIZE_HEIGHT));
subImg.copyTo(roi_dst1);
				}
				else if(i==2){
				cv::Mat roi_dst2 = cv_msg->image(cv::Rect(0,300,AR_RESIZE_WIDTH, AR_RESIZE_HEIGHT));
subImg.copyTo(roi_dst2);
				}
				else if(i==1){
				cv::Mat roi_dst3 = cv_msg->image(cv::Rect(350,0,AR_RESIZE_WIDTH, AR_RESIZE_HEIGHT));
subImg.copyTo(roi_dst3);
				}*/
				cv::Mat roi_dst = cv_msg->image(cv::Rect((cv_msg->image.cols - AR_RESIZE_WIDTH)/2, (cv_msg->image.rows - AR_RESIZE_HEIGHT)/2, AR_RESIZE_WIDTH, AR_RESIZE_HEIGHT));
				subImg.copyTo(roi_dst);

				//pub_.publish(cv_msg->toImageMsg());				
			}
		//}
	  //}
     }

	pub_.publish(cv_msg->toImageMsg());
#if 0
    if(detection == true && xmax-xmin > 0 && ymax-ymin > 0)
    {
        ROS_INFO("detection=%d, x=%d, y=%d, width=%d, height=%d", detection, xmin, ymin, xmax-xmin, ymax-ymin);

        cv::Mat subImg(cv_msg->image, cv::Rect(xmin, ymin, xmax-xmin, ymax-ymin));
	cv::resize(subImg, subImg, cv::Size(AR_RESIZE_WIDTH, AR_RESIZE_HEIGHT));
 
	// ここにsubImgの補正処理を入れる
	//cv::threshold(subImg, subImg, 100, 255, CV_THRESH_BINARY);

	cv::Mat roi_dst = cv_msg->image(cv::Rect((cv_msg->image.cols - AR_RESIZE_WIDTH)/2, (cv_msg->image.rows - AR_RESIZE_HEIGHT)/2, AR_RESIZE_WIDTH, AR_RESIZE_HEIGHT));
	subImg.copyTo(roi_dst);
/*
        int resize_width = AR_RESIZE_WIDTH;
        int resize_height = AR_RESIZE_HEIGHT;

        std::vector<cv::Point2f> src, dst;
        src.push_back(cv::Point2f(0, 0));
        src.push_back(cv::Point2f(xmax-xmin, 0));
        src.push_back(cv::Point2f(xmax-xmin, ymax-ymin));
        dst.push_back(cv::Point2f((cv_msg->image.cols - resize_width)/2, (cv_msg->image.rows - resize_height)/2));
        dst.push_back(cv::Point2f((cv_msg->image.cols - resize_width)/2 + resize_width, (cv_msg->image.rows - resize_height)/2));
        dst.push_back(cv::Point2f((cv_msg->image.cols - resize_width)/2 + resize_width, (cv_msg->image.rows - resize_height)/2 + resize_height));
        cv::Mat mat = cv::getAffineTransform(src, dst);

//        cv::warpAffine(subImg, cv_msg->image, mat, cv_msg->image.size(), CV_INTER_LINEAR, cv::BORDER_TRANSPARENT);
	cv::warpAffine(subImg, cv_msg->image, mat, cv_msg->image.size(), CV_INTER_LANCZOS4);
 //cv::BORDER_TRANSPARENT);
*/

 //pub_image_.publish(cv_msg->toImageMsg());
    }
        /* CvImage型をトピックに変換してpublish */
        pub_.publish(cv_msg->toImageMsg());
#endif

  // サンプルとして 赤で円をオーバーレイ描画する
  //cv::circle(cv_msg->image, cv::Point(100, 100), 100, CV_RGB(255,0,0), -1);

  //////////////////////////////////////////

}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "bla_sniper");
    ros::NodeHandle n("~");
    std::string param = "";
    n.getParam("robot_name", param);
    ROS_INFO("exec ar_sniper");

    // callback関数の登録
    sub_ = n.subscribe("in_image", 10, &imageCallback);
// sub_ = n.subscribe("/" + param + "/image_raw", 10, &imageCallback);

    // publishserオブジェクトの生成
    // サンプルではImageデータをpublishする
    image_transport::ImageTransport it(n);
    pub_ = it.advertise("out_image", 1);
	//pub_image_ = it.advertise("/" + param + "/out_image", 1);

    // loop処理
    ros::spin();
    return 0;
}
