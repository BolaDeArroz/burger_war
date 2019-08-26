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
int xmin = 0, ymin = 0;
int xmax = 0, ymax = 0;

inline double get_red(CvScalar color){return color.val[2];}
inline double get_green(CvScalar color){return color.val[1];}
inline double get_blue(CvScalar color){return color.val[0];}

bool sakuAR(cv_bridge::CvImagePtr cv_msg, CvScalar searchColor)
{
    bool detection = false;

    int scan_start_y = 80;
    int sca_end_y = 80;
    
    for(int i = scan_start_y; i < cv_msg->image.rows-sca_end_y; i++)
    {
        int y = i*cv_msg->image.step;

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
            }
        }
    }

    return detection;
}

void adjust_bright(cv_bridge::CvImagePtr cv_msg, double brightScale)
{
	double b ,g, r;

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

    adjust_bright(cv_msg, -0.7);

    // ARコードの枠色を探す
    detection = sakuAR(cv_msg, targetColor[1]);
    if(detection == false)
    {
        detection = sakuAR(cv_msg, targetColor[0]);
    }

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
