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
image_transport::Publisher  pub_image_;
cv_bridge::CvImage bridge_;
ros::Publisher pub_;
std_msgs::Float32MultiArray array;

#define nTARGETCOLOR    1
CvScalar targetColor[nTARGETCOLOR] = {
    CV_RGB(255,0,0),    // 赤玉の色(red)
};

int tolerance = 60;             // 色許容値
int xmin = 0, ymin = 0;
int xmax = 0, ymax = 0;

inline double get_red(CvScalar color){return color.val[2];}
inline double get_green(CvScalar color){return color.val[1];}
inline double get_blue(CvScalar color){return color.val[0];}

bool sakuteki(cv_bridge::CvImagePtr cv_msg, CvScalar searchColor)
{
    bool detection = false;

    for(int i = 0; i < cv_msg->image.rows; i++)
    {
        int y = i*cv_msg->image.step;
        int index = 0;

        for(int x = 0; x < cv_msg->image.cols; x++)
        {
            index = y+(x*3);
            int difRed = abs(get_red(searchColor)-cv_msg->image.data[index+2]);
            int difGreen = abs(get_green(searchColor)-cv_msg->image.data[index+1]);
            int difBlue = abs(get_blue(searchColor)-cv_msg->image.data[index+0]);

            //ROS_INFO("difRed=%d, difGreen= %d, difBlue=%d", difRed, difGreen, difBlue);

            //RGB各色が許容値以内の場合（近似色である場合）
            if(difRed < tolerance && difGreen < tolerance && difBlue < tolerance)
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

    return detection;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //ROS_INFO("get image");
  /* ImageトピックをCvImage型に変換 */
//  cv_bridge::CvImagePtr cv_origin_msg = cv_bridge::toCvCopy(msg, "bgr8");
  cv_bridge::CvImagePtr cv_msg = cv_bridge::toCvCopy(msg, "bgr8");

  //////////////////////////////////////////
  // ここにイメージ処理を実装する
  
  // サンプルとして 赤で円をオーバーレイ描画する
  //cv::circle(cv_msg->image, cv::Point(100, 100), 100, CV_RGB(255,0,0), -1);

    /*********************************************************/
    // ここからOpenCVを使用せずに赤玉の位置を特定するコード
    bool detection = false;
    int x = 0, y = 0;

    xmin = cv_msg->image.cols;
    ymin = cv_msg->image.rows;
    xmax = 0;
    ymax = 0;

    // 赤玉を探す
    detection = sakuteki(cv_msg, targetColor[0]);
    array.data[0] = detection;

    //ROS_INFO("detection=%d", detection);

    if(detection == true)
    {
        int x = (xmin + xmax)/2;
        int y = (ymin + ymax)/2;
        // デバッグ用描画(Black Circle)
//        cv::circle(cv_msg->image, cv::Point(x, y), xmax-xmin, CV_RGB(0,0,0), -1);

        // 敵と正面に向き合うための幅()
        int move_x = x-(cv_msg->image.cols/2);

        // detection, move_x, xmax-xminをmsgパラーメタとして送信
        array.data[1] = move_x;
        array.data[2] = xmax-xmin;
        array.data[3] = cv_msg->image.cols;
        array.data[4] = cv_msg->image.rows;
    }

    pub_.publish(array);

    // ここまでOpenCVを使用せずに赤玉の位置を特定するコード
    /*********************************************************/
  //////////////////////////////////////////
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "burger_sakuteki");
    ros::NodeHandle n("~");
    std::string param = "";
    n.getParam("robot_name", param);
    ROS_INFO("exec buger_sakuteki");

    array.data.resize(5);

    pub_ = n.advertise<std_msgs::Float32MultiArray>("/" + param + "/array", 5);

    // callback関数の登録
    sub_ = n.subscribe("/" + param + "/image_raw", 10, &imageCallback);

    // publishserオブジェクトの生成
    // サンプルではImageデータをpublishする
    image_transport::ImageTransport it(n);
    pub_image_ = it.advertise("/" + param + "/out_image", 1);

    // loop処理
    ros::spin();
    return 0;
}
