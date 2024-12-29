#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <math.h>
#include <cmath>
#include <opencv4/opencv2/core/mat.hpp>

//Ros2

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

enum Color {
	RED = 0,
	BLUE = 1
};

// class ImagePublisher : public rclcpp::Node
// {
// public:
// 	ImagePublisher() : Node("Image_publisher_result"){
// 		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("buff_decect_result", 10);
// 	}

// 	void PublishResultFrame(const Mat &result_frame){
// 		if(!result_frame.empty()){
// 			Mat bgr8_frame;
// 			cvtColor(result_frame, bgr8_frame, COLOR_HSV2BGR);
// 			cv_bridge::CvImage cv_image;
// 			cv_image.image = bgr8_frame;
// 			cv_image.encoding = "bgr8";
// 			cv_image.header.stamp = this->get_clock()->now();
//             cv_image.header.frame_id = "result_frame";
// 			auto img_msg = cv_image->toImageMsg();
//             publisher_.publish(img_msg);
		
// 		}else{
// 			RCLCPP_WARN(this->get_logger(), "Can't publish result frame!");
// 		}
// 	}

// private:
// 	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

// };

class Image : public rclcpp::Node
{
public:
    Image() : Node("buff_decect")
    {
		RCLCPP_INFO(this->get_logger(), "buff_detect_node_is running");
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&Image::imageCallback, this, std::placeholders::_1));
		
		//set level of logger
		this->get_logger().set_level((rclcpp::Logger::Level)10);

		//param
		//color change
		this->declare_parameter("Color(0-RED, 1-BLUE)", 0);
		this->get_parameter("Color(0-RED, 1-BLUE)", mask_color);

		rcl_interfaces::msg::ParameterDescriptor upper_blue_h;
        upper_blue_h.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        upper_blue_h.integer_range.push_back(rcl_interfaces::msg::IntegerRange());
        upper_blue_h.integer_range[0].from_value = 0;
        upper_blue_h.integer_range[0].to_value = 180;
        upper_blue_h.integer_range[0].step = 1;

		rcl_interfaces::msg::ParameterDescriptor upper_blue_s;
        upper_blue_s.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        upper_blue_s.integer_range.push_back(rcl_interfaces::msg::IntegerRange());
        upper_blue_s.integer_range[0].from_value = 0;
        upper_blue_s.integer_range[0].to_value = 255;
        upper_blue_s.integer_range[0].step = 1;

		rcl_interfaces::msg::ParameterDescriptor upper_blue_v;
        upper_blue_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        upper_blue_v.integer_range.push_back(rcl_interfaces::msg::IntegerRange());
        upper_blue_v.integer_range[0].from_value = 0;
        upper_blue_v.integer_range[0].to_value = 255;
        upper_blue_v.integer_range[0].step = 1;

		rcl_interfaces::msg::ParameterDescriptor lower_blue_h;
        lower_blue_h.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        lower_blue_h.integer_range.push_back(rcl_interfaces::msg::IntegerRange());
        lower_blue_h.integer_range[0].from_value = 0;
        lower_blue_h.integer_range[0].to_value = 255;
        lower_blue_h.integer_range[0].step = 1;

		rcl_interfaces::msg::ParameterDescriptor lower_blue_s;
        lower_blue_s.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        lower_blue_s.integer_range.push_back(rcl_interfaces::msg::IntegerRange());
        lower_blue_s.integer_range[0].from_value = 0;
        lower_blue_s.integer_range[0].to_value = 255;
        lower_blue_s.integer_range[0].step = 1;

		rcl_interfaces::msg::ParameterDescriptor lower_blue_v;
        lower_blue_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        lower_blue_v.integer_range.push_back(rcl_interfaces::msg::IntegerRange());
        lower_blue_v.integer_range[0].from_value = 0;
        lower_blue_v.integer_range[0].to_value = 255;
        lower_blue_v.integer_range[0].step = 1;

        // 声明参数
		this->declare_parameter("upper_blue_h", 110, upper_blue_h);
		this->declare_parameter("lower_blue_h", 90, lower_blue_h);
		this->declare_parameter("upper_blue_s", 255, upper_blue_s);
		this->declare_parameter("lower_blue_s", 140, lower_blue_s);
		this->declare_parameter("upper_blue_v", 255, upper_blue_v);
		this->declare_parameter("lower_blue_v", 140, lower_blue_v);
	}

	Scalar GetBlueLower(){
		Scalar lower = Scalar(this->get_parameter("lower_blue_h").get_parameter_value().get<int>(),
				this->get_parameter("lower_blue_s").get_parameter_value().get<int>(),
				this->get_parameter("lower_blue_v").get_parameter_value().get<int>());
		return lower;
	}

	Scalar GetBlueUpper(){
		Scalar upper = Scalar(this->get_parameter("upper_blue_h").get_parameter_value().get<int>(),
				this->get_parameter("upper_blue_s").get_parameter_value().get<int>(),
				this->get_parameter("upper_blue_v").get_parameter_value().get<int>());
		return upper;
	}

	Mat GetFrame() const{
		return image_mat_;rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_result_;
	}
	
	void PrintError()
	{
		RCLCPP_ERROR(this->get_logger(), "Can't get frame from camera");
	}
	
	int MaskColor(){
		this->get_parameter("Color(0-RED, 1-BLUE)", mask_color);
		return mask_color;
	}
private:
	Mat image_mat_;
	int mask_color;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
			//RCLCPP_DEBUG(this->get_logger(), "Frame is change to OpenCV Mat Successfully");
            // 使用cv_bridge将ROS图像消息转换为OpenCV的Mat类型
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			image_mat_ = cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

void dilateAndclose(const Mat input, Mat& output, int dilateNum) {
	Mat temp;
	Mat kernel = getStructuringElement(MORPH_RECT, Size(6, 6));
	dilate(input, temp, kernel, Point(-1, -1), dilateNum);
	//形态学处理
	morphologyEx(temp, output, MORPH_CLOSE, kernel);
}

void Erode(const Mat input, Mat& output) {
	Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	erode(input, output, kernel, Point(-1, -1), 1, BORDER_CONSTANT, 0);
}

void drawMinAreaRect(Mat& dst, const vector<Point>& contours, const Scalar color, int thickness, int linetype) {
	RotatedRect rect = minAreaRect(contours);
	Point2f point_rect[4];
	rect.points(point_rect);
	for (int i = 0; i <= 3; i++) {
		line(dst, point_rect[i], point_rect[(i + 1) % 4], color, thickness, linetype);
	}
}

void findContoursCenter(const vector<Point>& contours, Point2f& center) {
	Moments mom = moments(contours);
	center = Point2f(mom.m10 / mom.m00, mom.m01 / mom.m00);
}

void getTarget(Point2f& target, const Point2f& center_R, const Point2f& center_rect, const double rate) {
	//double radiu_buff = 0;
	if (center_R.x >= center_rect.x && center_R.y >= center_rect.y) {
		target = Point2f(center_R.x - (center_R.x - center_rect.x) * rate, center_R.y - (center_R.y - center_rect.y) * rate);
	}
	if (center_R.x <= center_rect.x && center_R.y >= center_rect.y) {
		target = Point2f(center_R.x + (center_rect.x - center_R.x) * rate, center_R.y - (center_R.y - center_rect.y) * rate);
	}
	if (center_R.x >= center_rect.x && center_R.y <= center_rect.y) {
		target = Point2f(center_R.x - (center_R.x - center_rect.x) * rate, center_R.y + (center_rect.y - center_R.y) * rate);
	}
	if (center_R.x <= center_rect.x && center_R.y <= center_rect.y) {
		target = Point2f(center_R.x + (center_rect.x - center_R.x) * rate, center_R.y + (center_rect.y - center_R.y) * rate);
	}
}

int main(int argc, char **argv) {
	Mat image;
	Mat image_hsv, mask_color, mask_dilate, mask_close;
	Mat mask_erode;
	int color = 0; // 0 READ, 1 BLUE
	Scalar blue_lower, blue_upper;
	Mat src;
	vector<vector<Point> > contours, contours_close;
	vector<Vec4i> hierarchies, hierarchies_close;
	//ros2
	rclcpp::init(argc, argv);
    auto node = std::make_shared<Image>();
	//auto node_publisher = std::make_shared<ImagePublisher>();
	rclcpp::spin_some(node);
	//rclcpp::spin(node_publisher);        

	while(1){
		rclcpp::spin_some(node);
		src = node->GetFrame();
		color = node->MaskColor();

		if(!src.empty()){
			image = src.clone();
			cvtColor(src, image_hsv, COLOR_BGR2HSV);
			blue_lower = node->GetBlueLower();
			blue_upper = node->GetBlueUpper();
			if (color == 1) {
				//Color blue
				//Scalar(90, 140, 140), Scalar(110, 255, 255)
				inRange(image_hsv, blue_lower, blue_upper, mask_color);
				dilateAndclose(mask_color, mask_close, 2);
				Erode(mask_color, mask_erode);
				findContours(mask_color, contours, hierarchies, RETR_TREE, CHAIN_APPROX_NONE);
				findContours(mask_close, contours_close, hierarchies_close, RETR_TREE, CHAIN_APPROX_NONE);
			}
			else if (color == 0) {
				Mat temp_mask_low, temp_mask_hight;
				inRange(image_hsv, Scalar(0, 50, 60), Scalar(10, 90, 255), temp_mask_low);
				inRange(image_hsv, Scalar(170, 50, 70), Scalar(180, 90, 255), temp_mask_hight);
				mask_color = temp_mask_hight | temp_mask_low;

				dilateAndclose(mask_color, mask_close, 2);
				findContours(mask_color, contours, hierarchies, RETR_TREE, CHAIN_APPROX_NONE);
				findContours(mask_close, contours_close, hierarchies_close, RETR_TREE, CHAIN_APPROX_NONE);
			}
			//imshow("Test", mask_color);

			int area_min = 1, area_max = 10000;
			int minID = 0;
			Point2f center_R;
			float radius;
			double minArea = 10000;
			for (size_t i = 0; i < contours.size(); i++) {
				if (contours.empty() || hierarchies.empty()) {
					cout << "No contours find!!" << endl;
					break;
				}
				double area = contourArea(contours[i]);

				if (area < area_min || area > area_max) {
					continue;
				}

				if (hierarchies[i][3] == -1 && hierarchies[i][2] != -1) {
					int childid = hierarchies[i][2];
					if (childid >= 0 && childid < contours.size()) {
						double area_child = contourArea(contours[childid]);
						if (hierarchies[childid][0] == -1 && hierarchies[childid][2] == -1) {
							if (minArea >= area_child) {
								minArea = area_child;
								minID = childid;
								//drawContours(image, contours, i, Scalar(0, 0, 255), 1, 8);
							}
						}
						else {
							continue;
						}
					}

				}
			}
			if ((minID >= 0 && minID < contours.size())) {
				if(hierarchies[minID][3] >= 0){
					minEnclosingCircle(Mat(contours[hierarchies[minID][3]]), center_R, radius);
					cv::circle(image, center_R, radius, Scalar(0, 0, 255), 1, 8);
				}
			}
			double maxArea = 1;
			int maxID = -1;
			Point2f center_rect;
			Point2f center_rect2;
			for (size_t i = 0; i < contours_close.size(); i++) {
				if (contours_close.empty() || hierarchies_close.empty()) {
					cout << "No contours!" << endl;
					break;
				}

				double area = contourArea(contours_close[i]);
				if (area < area_min || area > 1000000) {
					continue;
				}

				if (hierarchies_close[i][3] == -1 && hierarchies_close[i][2] != -1) {
					if (hierarchies_close[hierarchies_close[i][2]][0] != -1 && hierarchies_close[hierarchies_close[i][2]][0] < contours_close.size()) {
						double childarea = contourArea(contours_close[hierarchies_close[i][2]]);
						if (childarea >= 1000) {
							continue;
						}
						else if (area >= maxArea) {
							maxArea = area;
							maxID = i;
							//drawContours(image, contours_close, i, Scalar(0, 0, 0), 3, 8);
						}
					}
				}
				//drawContours(image, contours_close, i, Scalar(0, 0, 0), 3, 8);
			}
			if (contours_close.empty() || hierarchies_close.empty()) {
				cout << "No contours!" << endl;
			}
			else {
				if (maxID >= 0 && maxID < contours_close.size()) {
				
					drawContours(image, contours_close, maxID, Scalar(0, 255, 0), 2, 8);

					findContoursCenter(contours_close[maxID], center_rect);

					RotatedRect minRect = minAreaRect(contours_close[maxID]);
					Point2f point_pos[4];
					minRect.points(point_pos);
					vector<Point> buff_min_rect;
					for (int i = 0; i <= 3; i++) {
						buff_min_rect.push_back(point_pos[i]);
					}

					findContoursCenter(buff_min_rect, center_rect2);
					drawMinAreaRect(image, contours_close[maxID], Scalar(0, 255, 0), 1, 8);
					//cv::circle(image, center_rect, 1, Scalar(0, 0, 255), 2, 8);
				}
			}

			Point2f target, target2;
			double rate = 1.09;
			getTarget(target, center_R, center_rect, rate);
			getTarget(target2, center_R, center_rect2, 1.32);
			//point target

			//circle(image, target, 50, Scalar(0, 255, 255), 1, 8);
			circle(image, target, 1, Scalar(0, 0, 255), 2, 8);
			//drawContours(image, contours_close, -1, Scalar(0, 255, 0), 1, 8);
			if(image.empty() || mask_color.empty()){
				continue;
			}
			cv::imshow("image", image);
			cv::imshow("mask color", mask_color);
			//node_publisher->PublishResultFrame(image);
			cv::waitKey(1);
		}else{
			node->PrintError();		
			waitKey(1000);
		}
	}
	rclcpp::shutdown();
	return 0;
}
