#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <k4a/k4a.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <omp.h>
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/imgproc/imgproc_c.h"
#include<opencv2/imgproc/types_c.h>

//onnxruntime
#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>
#include <cuda_provider_factory.h>
// 命名空间
using namespace std;
using namespace cv;
using namespace Ort;



// 自定义配置结构
struct Configuration
{
	public: 
	float confThreshold; // Confidence threshold置信度阈值
	float nmsThreshold;  // Non-maximum suppression threshold非最大抑制阈值
	float objThreshold;  //Object Confidence threshold对象置信度阈值
	string modelpath;
};

// 定义BoxInfo结构类型

typedef struct BoxInfo
{
	float x1;
	float y1;
	float x2;
	float y2;
	float score;
	int label;
} BoxInfo;

class YOLOv5
{
public:
	YOLOv5(Configuration config);
	vector<BoxInfo> detect(Mat& frame);
private:
	float confThreshold;
	float nmsThreshold;
	float objThreshold;
	int inpWidth;
	int inpHeight;
	int nout;
	int num_proposal;
	int num_classes;
	string classes[1] = {"tower"};

	const bool keep_ratio = true;
	vector<float> input_image_;		// 输入图片
	void normalize_(Mat img);		// 归一化函数
	void nms(vector<BoxInfo>& input_boxes);  
	Mat resize_image(Mat srcimg, int *newh, int *neww, int *top, int *left);

	Env env = Env(ORT_LOGGING_LEVEL_ERROR, "yolov5-6.1"); // 初始化环境
	Session *ort_session = nullptr;    // 初始化Session指针选项
	SessionOptions sessionOptions = SessionOptions();  //初始化Session对象
	//SessionOptions sessionOptions;
	vector<char*> input_names;  // 定义一个字符指针vector
	vector<char*> output_names; // 定义一个字符指针vector
	vector<vector<int64_t>> input_node_dims; // >=1 outputs  ，二维vector
	vector<vector<int64_t>> output_node_dims; // >=1 outputs ,int64_t C/C++标准
};
// YOLOv5::YOLOv5(Configuration config);
// Mat YOLOv5::resize_image(Mat srcimg, int *newh, int *neww, int *top, int *left);
// void YOLOv5::normalize_(Mat img);
// void YOLOv5::nms(vector<BoxInfo>& input_boxes);
// void YOLOv5::detect(Mat& frame);

class Tower
{
public:
    vector<BoxInfo> Tower_box;
    int Find_tower(vector<BoxInfo>& Tower_box,Mat &src_no_alpha,Mat &transformed_depth_frame,int right_or_left); //对得到的塔方框进行排序并瞄准中间柱子
	float Find_depth(vector<BoxInfo>& Tower_box,Mat &transformed_depth_frame,Mat &src_no_alpha);
};