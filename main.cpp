#pragma comment(lib, "k4a.lib")

#include "serial_port.h"
#include "yolo.h"
#include <thread>  //管理多线程

double catch_moudle=0;
extern int left_tower_x,right_tower_x,angle;
extern bool mocalun;
void proc1()
{
	while(true)
	{
	int catch_m=catch_moudle;
	catch_moudle=read_data();
	// send_data(1,catch_moudle);
	// double catch_moudle_1=catch_moudle-1;
	// send_data(6,catch_moudle_1);
	//cout<<catch_moudle;
	if(catch_moudle==0)
		catch_moudle=catch_m;
	// for(int i=0;i<100;i++)
	// 	send_data(10,catch_moudle);
	}
}

void fafafa()
{
	send_data(1,20);
}


int main(int argc,char *argv[])
{
    left_tower_x=0;
    right_tower_x=0;
	bool catch_tower=false;
	bool left_tower=false;
	bool right_tower=false;
	thread th1(proc1);
	th1.detach();
	
	/*

		找到并打开 Azure Kinect 设备
	*/
	k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
	k4a::capture capture;

		/**********************摄像头***********************/
	const uint32_t device_count = k4a::device::get_installed_count();
	if (device_count == 0)
	{
		printf("No K4A devices found\n");
		return 0;
	}

	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	//config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	config.synchronized_images_only = true;
	device.start_cameras(&config);//开启摄像头

	/*校准深度和颜色*/
	k4a_calibration_t calibration = device.get_calibration(K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_720P);

	k4a::transformation transformation = k4a::transformation(calibration);
    // k4a_calibration_camera_t camera = calibration.color_camera_calibration;
    // double fx = camera.intrinsics.parameters.param.fx;
    // double fy = camera.intrinsics.parameters.param.fy;
    // double cx = camera.intrinsics.parameters.param.cx;
    // double cy = camera.intrinsics.parameters.param.cy;

    // cout << "Intrinsic matrix: " << endl;
    // cout << fx << " 0 " << cx << endl;
    // cout << "0 " << fy << " " << cy << endl;
    // cout << "0 0 1" << endl;
	/*定义image变量，接收相机传过来的原始数据*/
	k4a::image rgbImage;
	k4a::image depthImage;

	k4a::image transformed_depth_image = NULL;
	/*将image转化为Mat类型的接收的容器*/
	//cv::Mat color_frame;
	cv::Mat depth_frame;
	cv::Mat transformed_depth_frame;
	Mat src,src_no_alpha; 


	Configuration yolo_nets = { 0.3, 0.5, 0.7,"/home/xue/picture/modle/best.onnx" }; //初始化属性
	YOLOv5 yolo_model(yolo_nets);
	Tower Tower;
	clock_t startTime,endTime; //计算时间
	while(true)
	{
		if (device.get_capture(&capture, std::chrono::milliseconds(0)))
		{
			// fafafa();
			// send_data(6,33);
			//send_data(10,catch_moudle);
			//cout<<"catch_moudle"<<catch_moudle<<endl;
			/*捕获颜色和深度数据*/
			//cout<<catch_moudle<<endl;
			//send_data(10,15);
			rgbImage = capture.get_color_image();
			//depthImage = capture.get_depth_image();
			/*转换颜色和深度图像*/
			//transformed_depth_image = transformation.depth_image_to_color_camera(depthImage);
			/*将颜色和深度图像装入Mat容器中*/
			src = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4, rgbImage.get_buffer());
			//depth_frame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U, depthImage.get_buffer());

			//transformed_depth_frame = cv::Mat(transformed_depth_image.get_height_pixels(), transformed_depth_image.get_width_pixels(), CV_16U, transformed_depth_image.get_buffer());

			///

			depthImage = capture.get_depth_image();
			transformed_depth_image = transformation.depth_image_to_color_camera(depthImage);
			depth_frame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U, depthImage.get_buffer());
			transformed_depth_frame = cv::Mat(transformed_depth_image.get_height_pixels(), transformed_depth_image.get_width_pixels(), CV_16U, transformed_depth_image.get_buffer());
			

			///

			cv::cvtColor(src, src_no_alpha, cv::COLOR_BGRA2BGR);

			double timeStart = (double)getTickCount();
			startTime = clock();//计时开始	
			vector<BoxInfo> Tower_box=yolo_model.detect(src_no_alpha);
			endTime = clock();//计时结束
			double nTime = ((double)getTickCount() - timeStart) / getTickFrequency();
			// cout << "clock_running time is:" <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
			// cout << "The run time is:" << (double)clock() /CLOCKS_PER_SEC<< "s" << endl;
			// cout << "getTickCount_running time :" << nTime << "sec\n" << endl;
			// static const string kWinName = "Deep learning object detection in ONNXRuntime";

			int right_or_left=0;  //0时为中，1时为右，2时为左
			if(catch_moudle>0.5&&catch_moudle<6)
			{
				if(catch_moudle==1)		 //开柱
				{
					catch_tower=true;
				} 
				else if(catch_moudle==2)
				{
					left_tower=false;
					right_tower=false;
					left_tower_x=0;
					right_tower_x=0;
					
					// depthImage = capture.get_depth_image();
					// transformed_depth_image = transformation.depth_image_to_color_camera(depthImage);
					// depth_frame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16U, depthImage.get_buffer());
					// transformed_depth_frame = cv::Mat(transformed_depth_image.get_height_pixels(), transformed_depth_image.get_width_pixels(), CV_16U, transformed_depth_image.get_buffer());
					float revolving_speed=Tower.Find_depth(Tower_box,transformed_depth_frame,src_no_alpha);
					angle=0;
					//cout<<"转速"<<revolving_speed<<endl;
						
					send_data(3,revolving_speed);  //得到的转速
					//cout<<"水平深度"<<Tower.Find_depth(Tower_box,transformed_depth_frame)<<endl;
				}
				else if(catch_moudle==3) //左柱
				{
					right_or_left=2;
					mocalun=false;
					//send_data(4.0,0);
					send_data(3,0);  //得到的转速
				}
				else if(catch_moudle==4) //右柱
				{
					right_or_left=1;
					mocalun=false;
					//send_data(4.0,0);
					send_data(3,0);  //得到的转速
				}
				else if(catch_moudle==5) //关柱
				{
					mocalun=false;
					catch_tower=false;
					send_data(4.0,0);
					send_data(3,0);  //得到的转速
				}
			}

			if(right_or_left==2||left_tower==true)
			{
				catch_moudle=0;
				right_or_left=2;
				left_tower=true;
			}
			else if(right_or_left==1||right_tower==true)
			{
				catch_moudle=0;
				right_or_left=1;
				right_tower=true;
			}


			if(catch_moudle!=2&&catch_tower==true)
			{
				cout<<"right_or_left"<<right_or_left<<endl;
				int deviate = Tower.Find_tower(Tower_box,src_no_alpha,transformed_depth_frame,right_or_left);
				if(deviate == 1000)
				{
					send_data(2.0,0.0); //未捕捉到柱
					cout<<"未捕捉到柱"<<endl;
				}
				else 
				{
					if(deviate==0)
					{
						cout<<"fale0"<<endl;
						send_data(1,0);
					}
					send_data(6,deviate); //偏差
					cout<<"偏差为"<<deviate<<endl;
				}
			}
			cv::namedWindow("color", CV_WINDOW_NORMAL);
			cv::imshow("color",src_no_alpha);

			src.release();
			src_no_alpha.release();
			capture.reset();

			if (cv::waitKey(1) == ' ')
					{//按键采集，用户按下' ',跳出循环,结束采集
						std::cout << "----------------------------------" << std::endl;
						std::cout << "------------- closed -------------" << std::endl;
						std::cout << "----------------------------------" << std::endl;
						break;
					}
		}
			
	}
	
    // Shut down the camera when finished with application logic
	cv::destroyAllWindows();
	device.close();
    return 0;

}

