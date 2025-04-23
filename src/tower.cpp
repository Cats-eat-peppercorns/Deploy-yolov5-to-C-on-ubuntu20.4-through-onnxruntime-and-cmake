#include "yolo.h"
#include "serial_port.h"
#include <vector>
#include <algorithm>
int src_middle=640;
int src_middle_y=360;
/*****左柱坐标*****/
int left_tower_x;
/*****右柱坐标*****/
int	right_tower_x;

const float g=-9.8066;
const float tan_a=0.795;
const float cos_a=0.783;

bool mocalun=false;
float revolving_speed=0;
Mat K = (Mat_<double>(3, 3) << 605.719, 0, 639.149, 0, 605.785, 368.511, 0, 0, 1);

typedef struct Point_with_wide
{
    int x;
    int y;
    int wide;
	int hight;
}Point_with_wide;
bool Point_sorting(Point_with_wide a,Point_with_wide b) //依靠点的x坐标从小到大排序
{
    return a.x<b.x;
};

// int Tower::Find_tower(vector<BoxInfo>& Tower_box,Mat &src_no_alpha,Mat& transformed_depth_frame,int right_or_left)  //像素偏差 发现偏差减一更接近相机坐标的偏差，如果后续需要可以用像素减一然后传递给电控
// {
//     // vector<int> Tower_box_x;   //****注意，这里不知道Tower_box里的顺序到底是怎样的，如果不是从左往右的顺序的话可能需要排个序
//     // for(int i=0;i<Tower_box.size();i++) //获得所有柱子的横坐标
//     // {
//     //     Tower_box_x.push_back((int)(Tower_box[i].x1+Tower_box[i].x2)/2);
//     // }
//     // sort(Tower_box_x.begin(),Tower_box_x.end()); //对横坐标做一个排序，防止横坐标是乱序的
//     vector<Point_with_wide> Tower_box_x;   //****注意，这里不知道Tower_box里的顺序到底是怎样的，如果不是从左往右的顺序的话可能需要排个序

//     for(int i=0;i<Tower_box.size();i++) //获得所有柱子的横坐标
//     {
//         Tower_box_x.push_back(Point_with_wide{(int)(Tower_box[i].x1+Tower_box[i].x2)/2,(int)(Tower_box[i].y1+Tower_box[i].y2)/2,(int)abs(Tower_box[i].x2-Tower_box[i].x1),(int)abs(Tower_box[i].y2-Tower_box[i].y1)});
//     }
//     sort(Tower_box_x.begin(),Tower_box_x.end(),Point_sorting); //对横坐标做一个排序，防止横坐标是乱序的

//     if(right_or_left==0) //识别中间柱
// 	{
// 		if (Tower_box.size() > 0) //若有柱子
// 		{	int j=0;
// 			int min=abs(Tower_box_x[0].x-src_middle);
// 			for (int i = 0; i < Tower_box.size(); i++) //找到离中心点最近的框
// 			{
// 				int ifmin=abs(Tower_box_x[i].x-src_middle);
// 				if(ifmin<min)  
// 				{
// 					min=ifmin;
// 					j=i;  //第几个框
// 				}
// 			}
// 			int Find_X_D_X=Tower_box_x[j].x-src_middle; //像素偏差
		
// 		double u = (float)Tower_box_x[j].x, v = (float)(Tower_box_x[j].y-Tower_box_x[j].hight/2);

// 		float depth_value = transformed_depth_frame.at<ushort>((Tower_box_x[j].y-Tower_box_x[j].hight/2+5),Tower_box_x[j].x);;
// 		// 转化为相机坐标
// 		Mat pixelPoint = (Mat_<double>(3, 1) << u, v, 1);
// 		Mat cameraPoint = K.inv() * pixelPoint;
// 		double X = cameraPoint.at<double>(0, 0)*depth_value;
// 		double Y = cameraPoint.at<double>(1, 0)*depth_value;
// 		double Z = cameraPoint.at<double>(2, 0)*depth_value;
// 		cout<<"X"<<X<<"Y"<<Y<<"Z"<<Z<<"像素偏差"<<Find_X_D_X<<endl;

// 			return Find_X_D_X;
// 		}
// 		else
// 		{
// 			return 1000;
// 			cout << "未识别到立柱" << endl;
// 		}
// 	}
// 	else if(right_or_left==2) //识别左边
// 	{
// 		if (Tower_box.size() > 0)
// 		{	
//             if(left_tower_x==0) //如果第一次进左柱
//             {
//                 int j=0;
//                 int min=abs(Tower_box_x[0].x-src_middle);
//                 for (int i = 0; i < Tower_box.size(); i++) //找到离中心点最近的框
//                 {
// 				int ifmin=abs(Tower_box_x[i].x-src_middle);
// 				if(ifmin<min)  
// 				{
// 					min=ifmin;
// 					j=i;  //第几个框
// 				}
//                 }
//                 if(j==0)  //若只识别到一个柱子或者该柱子已是最左侧则不切换柱子且返回3（不转）
//                 {
//                     return 0;
//                 }
//                 else  //若左边有柱子，则直接把左柱子坐标赋给left_tower_x
//                 {
//                     left_tower_x=Tower_box_x[j-1].x;
//                 }
// 			}
// 			int j=0;
// 			int min=abs(Tower_box_x[Tower_box.size()-1].x-left_tower_x);  
// 			for (int i = 0; i < Tower_box.size(); i++)   //此循环用于找到最接近left_tower_x的柱子，且作差要是正的
// 			{
//                 int ifmin=Tower_box_x[i].x-left_tower_x;
// 				if(ifmin<min&&ifmin>=0)  
// 				{
// 					min=ifmin;
// 					j=i;  //j就是离left_tower_x最近的柱子
// 				}
// 				else if(ifmin<min&&ifmin>=-10&&ifmin<0) //防止超调后找不到最近的柱子 回转像素值不能超过10
// 				{
// 					min=abs(ifmin);
// 					j=i;
// 				}
// 			}
// 			left_tower_x=Tower_box_x[j].x;  //更新left_tower_x
// 			int Find_X_D_X=Tower_box_x[j].x-src_middle;
// 			return Find_X_D_X;
// 		}
// 		else
// 		{
// 			return 1000;
// 			cout << "未识别到立柱" << endl;
// 		}
// 	}
// 	else if(right_or_left==1) //识别右边
// 	{
// 		if (Tower_box.size() > 0)
// 		{	
//             if(right_tower_x==0) //如果第一次进左柱
//             {
//                 int j=0;
//                 int min=abs(Tower_box_x[0].x-src_middle);
//                 for (int i = 0; i < Tower_box.size(); i++) //找到离中心点最近的框
//                 {
// 				int ifmin=abs(Tower_box_x[i].x-src_middle);
// 				if(ifmin<min)  
// 				{
// 					min=ifmin;
// 					j=i;  //第几个框
// 				}
//                 }
//                 if(j==Tower_box.size()-1)  //若只识别到一个柱子或者该柱子已是最右侧则不切换柱子且返回3（不转）
//                 {
//                     return 0;
//                 }
//                 else  //若右边有柱子，则直接把左柱子坐标赋给right_tower_x
//                 {
//                     right_tower_x=Tower_box_x[j+1].x;
//                 }
// 			}
// 			int j=0;
// 			int min=abs(Tower_box_x[0].x-right_tower_x);
// 			for (int i = 0; i < Tower_box.size(); i++)   //此循环用于找到最接近right_tower_x的柱子
// 			{
//                 int ifmin=abs(Tower_box_x[i].x-right_tower_x);
// 				if(ifmin<min)  
// 				{
// 					min=ifmin;
// 					j=i;  //j就是离right_tower_x最近的柱子
// 				}
// 			}
// 			right_tower_x=Tower_box_x[j].x;  //更新right_tower_x
// 			int Find_X_D_X=Tower_box_x[j].x-src_middle;
// 			return Find_X_D_X;
// 		}
// 		else
// 		{
// 			return 1000;
// 			cout << "未识别到立柱" << endl;
// 		}
// 	}
// 	return 1000;
// }
double angle =0;
int Tower::Find_tower(vector<BoxInfo>& Tower_box,Mat &src_no_alpha,Mat& transformed_depth_frame,int right_or_left)  //角度偏差，第一次进入时发几次角度偏差，然后开始发距离最近的柱子的角度偏差或像素偏差
{
    // vector<int> Tower_box_x;   //****注意，这里不知道Tower_box里的顺序到底是怎样的，如果不是从左往右的顺序的话可能需要排个序
    // for(int i=0;i<Tower_box.size();i++) //获得所有柱子的横坐标
    // {
    //     Tower_box_x.push_back((int)(Tower_box[i].x1+Tower_box[i].x2)/2);
    // }
    // sort(Tower_box_x.begin(),Tower_box_x.end()); //对横坐标做一个排序，防止横坐标是乱序的
    vector<Point_with_wide> Tower_box_x;   //****注意，这里不知道Tower_box里的顺序到底是怎样的，如果不是从左往右的顺序的话可能需要排个序

    for(int i=0;i<Tower_box.size();i++) //获得所有柱子的横坐标
    {
        Tower_box_x.push_back(Point_with_wide{(int)(Tower_box[i].x1+Tower_box[i].x2)/2,(int)(Tower_box[i].y1+Tower_box[i].y2)/2,(int)abs(Tower_box[i].x2-Tower_box[i].x1),(int)abs(Tower_box[i].y2-Tower_box[i].y1)});
    }
    sort(Tower_box_x.begin(),Tower_box_x.end(),Point_sorting); //对横坐标做一个排序，防止横坐标是乱序的

    if(right_or_left==0) //识别中间柱
	{
		if (Tower_box.size() > 0) //若有柱子
		{	int j=0;
			int min=abs(Tower_box_x[0].x-src_middle);
			for (int i = 0; i < Tower_box.size(); i++) //找到离中心点最近的框
			{
				int ifmin=abs(Tower_box_x[i].x-src_middle);
				if(ifmin<min)  
				{
					min=ifmin;
					j=i;  //第几个框
				}
			}
			int Find_X_D_X=Tower_box_x[j].x-src_middle; //像素偏差
		
		double u = (float)Tower_box_x[j].x, v = (float)(Tower_box_x[j].y);

		float depth_value = transformed_depth_frame.at<ushort>(Tower_box_x[j].y,Tower_box_x[j].x);;
		// 转化为相机坐标
		Mat pixelPoint = (Mat_<double>(3, 1) << u, v, 1);
		Mat cameraPoint = K.inv() * pixelPoint;
		double X = cameraPoint.at<double>(0, 0)*depth_value;
		double Y = cameraPoint.at<double>(1, 0)*depth_value;
		double Z = cameraPoint.at<double>(2, 0)*depth_value;
		
		angle = atan(X / Z) * 180 / M_PI; //计算出角度
		//angle=angle;
		//cout<<"X"<<X<<"Y"<<Y<<"Z"<<Z<<"角度"<<angle<<endl;
			send_data(1,(float)angle);  //发角度
			Find_X_D_X++;
			return Find_X_D_X;
		}
		else
		{
			return 1000;
			cout << "未识别到立柱" << endl;
		}
	}
	else if(right_or_left==2) //识别左边
	{
		if (Tower_box.size() > 0)
		{	
            if(left_tower_x==0) //如果第一次进左柱
            {
                int j=0;
                int min=abs(Tower_box_x[0].x-src_middle);
                for (int i = 0; i < Tower_box.size(); i++) //找到离中心点最近的框
                {
				int ifmin=abs(Tower_box_x[i].x-src_middle);
				if(ifmin<min)  
				{
					min=ifmin;
					j=i;  //第几个框
				}
                }
                if(j==0)  //若只识别到一个柱子或者该柱子已是最左侧则不切换柱子且返回3（不转）
                {   cout<<"jiaodu"<<endl;
					//send_data(1,0);  //发角度
                    return 0;
                }
                else  //若左边有柱子，则直接把左柱子偏差发给单片机
                {
                    left_tower_x=Tower_box_x[j-1].x;
					double u = (float)Tower_box_x[j-1].x, v = (float)(Tower_box_x[j-1].y);

					float depth_value = transformed_depth_frame.at<ushort>(Tower_box_x[j-1].y,Tower_box_x[j-1].x);;
					// 转化为相机坐标
					Mat pixelPoint = (Mat_<double>(3, 1) << u, v, 1);
					Mat cameraPoint = K.inv() * pixelPoint;
					if(depth_value==0)
					{
						depth_value=8000;
					}
					double X = cameraPoint.at<double>(0, 0)*depth_value;
					double Y = cameraPoint.at<double>(1, 0)*depth_value;
					double Z = cameraPoint.at<double>(2, 0)*depth_value;
					// cout<<X<<"   "<<Z<<endl;
					// if(Z==0){
					// 	Z=8000;
					// }
					angle = atan(X / Z) * 180 / M_PI; //计算出角度
					//angle=angle-1;
					//cout<<"X"<<X<<"Y"<<Y<<"Z"<<Z<<"角度为"<<angle<<endl;
					// for(int i=0;i<=300;i++){
						
					// }
					
                }
			}
			//send_data(1,20);
			send_data(1,(float)angle);  //发角度
			cout<<"角度为"<<(float)angle<<endl;
			int j=0;
			int min=abs(Tower_box_x[0].x-src_middle);
			for (int i = 0; i < Tower_box.size(); i++) //找到离中心点最近的框
			{
				int ifmin=abs(Tower_box_x[i].x-src_middle);
				if(ifmin<min)  
				{
					min=ifmin;
					j=i;  //第几个框
				}
			}
			int Find_X_D_X=Tower_box_x[j].x-src_middle; //像素偏差
			Find_X_D_X++;
			return Find_X_D_X;
		}
		else
		{
			return 1000;
			cout << "未识别到立柱" << endl;
		}
	}
	else if(right_or_left==1) //识别右边
	{
		if (Tower_box.size() > 0)	
		{	
            if(right_tower_x==0) //如果第一次进左柱
            {
               int j=0;
                int min=abs(Tower_box_x[0].x-src_middle);
                for (int i = 0; i < Tower_box.size(); i++) //找到离中心点最近的框
                {
				int ifmin=abs(Tower_box_x[i].x-src_middle);
				if(ifmin<min)  
				{
					min=ifmin;
					j=i;  //第几个框
				}
                }
                if(j==Tower_box.size()-1)  //若只识别到一个柱子或者该柱子已是最右侧则不切换柱子且返回3（不转）
                {
					//send_data(1,0);  //发角度
                    return 0;
                }
                else  //若左边有柱子，则直接把左柱子偏差发给单片机
                {
                    right_tower_x=Tower_box_x[j+1].x;
					double u = (float)Tower_box_x[j+1].x, v = (float)(Tower_box_x[j+1].y);

					float depth_value = transformed_depth_frame.at<ushort>(Tower_box_x[j+1].y,Tower_box_x[j+1].x);
					// 转化为相机坐标
					Mat pixelPoint = (Mat_<double>(3, 1) << u, v, 1);
					Mat cameraPoint = K.inv() * pixelPoint;
					if(depth_value==0)
					{
						depth_value=8000;
					}
					double X = cameraPoint.at<double>(0, 0)*depth_value;
					double Y = cameraPoint.at<double>(1, 0)*depth_value;
					double Z = cameraPoint.at<double>(2, 0)*depth_value;
					//cout<<X<<"   "<<Z<<endl;
					angle = atan(X / Z) * 180 / M_PI; //计算出角度
					//angle=angle-1;
					//cout<<"X"<<X<<"Y"<<Y<<"Z"<<Z<<"角度为"<<angle<<endl;
					// for(int i=0;i<=300;i++)
					// {
						
					// }
						
                }
			}
			send_data(1,(float)angle);  //发角度
			cout<<"角度为"<<angle<<endl;
			int j=0;
			int min=abs(Tower_box_x[0].x-src_middle);
			for (int i = 0; i < Tower_box.size(); i++) //找到离中心点最近的框
			{
				int ifmin=abs(Tower_box_x[i].x-src_middle);
				if(ifmin<min)  
				{
					min=ifmin;
					j=i;  //第几个框
				}
			}
			int Find_X_D_X=Tower_box_x[j].x-src_middle; //像素偏差
			Find_X_D_X++;
			return Find_X_D_X;
		}
		else
		{
			return 1000;
			cout << "未识别到立柱" << endl;
		}
	}
	return 1000;
}



float Tower::Find_depth(vector<BoxInfo>& Tower_box,Mat &transformed_depth_frame,Mat& src_no_alpha)//
{
    vector<Point_with_wide> Tower_box_x;   //****注意，这里不知道Tower_box里的顺序到底是怎样的，如果不是从左往右的顺序的话可能需要排个序

    for(int i=0;i<Tower_box.size();i++) //获得所有柱子的横坐标
    {
        Tower_box_x.push_back(Point_with_wide{(int)(Tower_box[i].x1+Tower_box[i].x2)/2,(int)(Tower_box[i].y1+Tower_box[i].y2)/2,(int)abs(Tower_box[i].x2-Tower_box[i].x1),(int)abs(Tower_box[i].y2-Tower_box[i].y1)});
    }
    sort(Tower_box_x.begin(),Tower_box_x.end(),Point_sorting); //对横坐标做一个排序，防止横坐标是乱序的

    if (Tower_box.size() > 0) //若有柱子
    {	
		int j=0;
        int min=abs(Tower_box_x[0].x-src_middle);
        for (int i = 0; i < Tower_box.size(); i++) //找到离中心点最近的框
        {
            int ifmin=abs(Tower_box_x[i].x-src_middle);
            if(ifmin<min)  
            {
                min=ifmin;
                j=i;  //第几个框
            }
        }
		//cout<<mocalun<<endl;
		if(mocalun==false)
		{
			// 像素坐标 正中心顶部坐标
		double u = (float)Tower_box_x[j].x, v = (float)(Tower_box_x[j].y-Tower_box_x[j].hight/2);

		float depth_value = transformed_depth_frame.at<ushort>((Tower_box_x[j].y-Tower_box_x[j].hight/2+5),Tower_box_x[j].x);
		// 转化为相机坐标
		Mat pixelPoint = (Mat_<double>(3, 1) << u, v, 1);
		Mat cameraPoint = K.inv() * pixelPoint;
		if(depth_value==0)
		{
			depth_value=8000;
		}
		double X = cameraPoint.at<double>(0, 0)*depth_value;
		double Y = cameraPoint.at<double>(1, 0)*depth_value;
		double Z = cameraPoint.at<double>(2, 0)*depth_value;
		//cout<<"u"<<u<<"v"<<v<<"depth_value"<<depth_value<<endl;
		cv::Point pt(u, v); // 点的坐标
		cv::Scalar color(255, 255, 255); // 颜色，这里使用红色
		int radius = 5; // 圆的半径
		cv::circle(src_no_alpha, pt, radius, color, -1); // 在图像上绘制圆
		cout<<"Y"<<Y<<"Z"<<Z<<endl;
		float R=Z/1000.0;
		
		float H=Y/1000.0-0.225;
		float v_x=pow(g*R*R/2*(H-R*tan_a),0.5);
		float v_true=v_x/cos_a;
		revolving_speed=(v_true+11.717)/0.0026;
		//double angle_h = atan(Y / Z) * 180 / M_PI; //计算出角度
		//cout<<"angle_h"<<angle_h<<endl;
		cout<<"before_add"<<revolving_speed<<endl;
		//if(revolving_speed<=10000)
			//revolving_speed=revolving_speed+3000/R*R*R;
		if(revolving_speed>29500)
			revolving_speed=29500;
		cout<<"revolving_speed"<<revolving_speed<<endl;
		mocalun=true;
		//send_data(3,revolving_speed); 
		//cout << "Camera coordinates: (" << X << ", " << Y << ", " << Z << ")"<< angle_h<< endl;
		}
		
		return revolving_speed;
    }
    else
    {
        return 0;
        cout << "未识别到立柱" << endl;
    }
    return 0;
}


// bool ring_is_red(int i, int j, Mat& src_hsv, Mat& src_new, Mat& transformed_depth_frame)
// {
// 	if (src_hsv.at<Vec3b>(i, j)[0] >= ring_r_h_low && src_hsv.at<Vec3b>(i, j)[0] <= ring_r_h_high)
// 	{
// 		if (src_hsv.at<Vec3b>(i, j)[2] >= 50 && src_hsv.at<Vec3b>(i, j)[2] <= 240)
// 		{
// 			//transformed_depth_frame.at<Vec3b>(i, j) = 0;
// 			return true;
// 		}

// 	}
// 	else
// 		return false;
// 	return false;
// }
// bool ring_is_blue(int i, int j, Mat& src_hsv, Mat& src_new)
// {

// 	if (src_hsv.at<Vec3b>(i, j)[0] >= ring_b_h_low && src_hsv.at<Vec3b>(i, j)[0] <= ring_b_h_high)
// 	{
// 		if (src_hsv.at<Vec3b>(i, j)[1] >= ring_b_s_low && src_hsv.at<Vec3b>(i, j)[1] <= ring_b_s_high)
// 		{
// 			if (src_hsv.at<Vec3b>(i, j)[2] >= ring_b_v_low && src_hsv.at<Vec3b>(i, j)[2] <= ring_b_v_high)
// 			{
// 				//transformed_depth_frame.at<Vec3b>(i, j) = (255, 255,255);
// 				return true;
// 			}
// 		}
// 	}
// 	else
// 		return false;
// 	return false;
// }

// bool ring_depth(int i, int j, int depth, Mat& transformed_depth_frame)
// {
// 	if ((depth - transformed_depth_frame.at<ushort>(top_l + i, left_l + j)) < 200 && (depth - transformed_depth_frame.at<ushort>(top_l + i, left_l + j)) > -200)
// 		return true;
// 	else return false;
// }
// //***********************************
// //************得到环的存在和颜色信息
// int Find_ring(int src_x, int Find_distance, int depth, Mat& transformed_depth_frame, Mat& src_hsv, Mat& src_new)   //输入的为柱左端横坐标和柱体图中宽度
// {
// 	bool ring_out = false; //退出循环的flag
// 	int ring_color = 0;  //0为未找到，1为红色，2为蓝色
// 	//cout << "11" << endl;
// 	for (int i = src_hsv.rows / 2; ring_out == false; i--)
// 	{
// 		int k = 0;
// 		int k2 = 0;
// 		int is_red = 0;
// 		int is_blue = 0;
// 		bool f_c = false;
// 		if (src_x - Find_distance > 0)
// 		{
// 			for (int j = src_x - Find_distance; j < src_x + Find_distance; j++)
// 			{
// 				if (ring_is_red(i, j, src_hsv, src_new, transformed_depth_frame) && ring_depth(i, j, depth, transformed_depth_frame))
// 				{
// 					is_red++;
// 					//ring_color = 1;
// 					f_c = true;
// 				}
// 				else if (ring_is_blue(i, j, src_hsv, src_new) && ring_depth(i, j, depth, transformed_depth_frame))
// 				{
// 					is_blue++;
// 					//ring_color = 2;
// 					f_c = true;
// 				}
// 			}
// 		}

// 		if (is_red > 3 || is_blue > 3)
// 		{
// 			if (is_red >= is_blue)
// 				ring_color = 1;
// 			else if (is_red < is_blue)
// 				ring_color = 2;
// 		}
// 		if (f_c == false && ring_color != 0 && k <= 30)  //若已经检测出环的颜色且在最后一次检测到后的30次内
// 		{
// 			k++;
// 		}
// 		else if (f_c == false && ring_color == 0 && k2 <= 150)  //若未检测出环的颜色且持续了150次
// 		{
// 			k2++;
// 		}
// 		if (k > 30 || k2 > 150 || i < 20)
// 			ring_out = true;
// 	}
// 	//cout << "22" << endl;
// 	return ring_color;
// }