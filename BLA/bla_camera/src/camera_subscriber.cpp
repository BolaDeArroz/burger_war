#include <boost/function.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

ros::Subscriber sub_;
image_transport::Publisher  pub_;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("get image");
  /* ImageトピックをCvImage型に変換 */
  cv_bridge::CvImagePtr cv_msg = cv_bridge::toCvCopy(msg, "bgr8");

  //////////////////////////////////////////
  // ここにイメージ処理を実装する
  
  // サンプルとして 赤で円をオーバーレイ描画する
  cv::circle(cv_msg->image, cv::Point(100, 100), 100, CV_RGB(255,0,0), -1);

  //////////////////////////////////////////


  
  /* CvImage型をトピックに変換してpublish */
  pub_.publish(cv_msg->toImageMsg());
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "bla_camera");
  ros::NodeHandle n("~");
  ROS_INFO("exec bla_camra");

  // callback関数の登録
  sub_ = n.subscribe("/in_image", 10, &imageCallback);

  // publishserオブジェクトの生成
  // サンプルではImageデータをpublishする
  image_transport::ImageTransport it(n);
  pub_ = it.advertise("out_image", 1);

  // loop処理
  ros::spin();
  return 0;
}
