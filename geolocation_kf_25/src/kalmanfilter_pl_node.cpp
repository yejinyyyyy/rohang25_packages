/*
date : 250331
name : Yejin
text:  
 - Geolocation reference code (raw)
 - altitude 5m pub condition temp (5m  kalman data using---> not using)
*/

#include <iostream>
#include <std_msgs/Char.h>
#include <std_msgs/Int32.h>
#include <math.h>	//HJ temp

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <kalmanfilter.h> // --> "kalmanfilter.h" ?
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>   // ws
#include <gazebo_msgs/LinkStates.h>//ws add
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
//#include <mavros_msgs/Altitude.h>  // ws add
#include <sensor_msgs/Imu.h>//ws add


#define dt 0.05		//1/2.0
#define posQ 1
#define velQ 1
#define posR 10

/// ver8
void callback_cnn(const gazebo_msgs::ModelStates::ConstPtr& msg);  // not used?

void callback_yolo(const geometry_msgs::Point32::ConstPtr& msg);
void callback_yolo2(const geometry_msgs::Point32::ConstPtr& msg);

void callback_fcc_lla(const geometry_msgs::Point::ConstPtr& msg);
void callback_fcc_rpy(const geometry_msgs::Point::ConstPtr& msg);
void callback_fcc_vel_ned(const geometry_msgs::Point::ConstPtr& msg);

void callback_kalmanfilter_pl_node_control_flag(const std_msgs::Int32::ConstPtr& msg);

void callback_homeposition(const geometry_msgs::Point::ConstPtr& msg);

Eigen::Matrix<double, 3, 3> rotm_gimbal_3d(double roll, double pitch, double yaw); // ws add
Eigen::Matrix<double, 3, 3> rotm_3d(double roll, double pitch, double yaw); // ws add

geometry_msgs::Point32 CNNDetectionList_ws;
geometry_msgs::Point32 CNNDetectionList2_ws; 

geometry_msgs::Point fcc_lla_msg;
geometry_msgs::Point fcc_rpy_msg;
geometry_msgs::Point fcc_vel_ned_msg;

geometry_msgs::Point homeposition_msg;

std_msgs::Int32 kalmanfilter_pl_node_control_flag_msg;
std_msgs::Int32 kalmanfilter_pl_data_valid_flag_msg;

int main(int argc, char* argv[])
{
	/////////////////////////////////////////////////
	/////////////////ROS Init////////////////////////
	/////////////////////////////////////////////////

	ros::init(argc, argv, "kalmanfilter_1");
	ros::NodeHandle nh;
	ros::Rate rate(1/dt);

	/// sub qkesmsqjq 
	ros::Subscriber yolo_sub = nh.subscribe<geometry_msgs::Point32>("yolo/center", 1, callback_yolo);   
    ros::Subscriber yolo2_sub = nh.subscribe<geometry_msgs::Point32>("yolo/box_size", 1, callback_yolo2);     //      ws change
	ros::Subscriber fcc_lla_pub = nh.subscribe<geometry_msgs::Point>("fcc_lla", 1, callback_fcc_lla);
	ros::Subscriber fcc_rpy_pub = nh.subscribe<geometry_msgs::Point>("fcc_rpy", 1, callback_fcc_rpy);
	ros::Subscriber fcc_vel_ned_pub = nh.subscribe<geometry_msgs::Point>("fcc_vel_ned", 1, callback_fcc_vel_ned);
    ros::Subscriber kalmanfilter_pl_node_control_flag_sub = nh.subscribe<std_msgs::Int32>("kalmanfilter_pl_node_control_flag", 1, callback_kalmanfilter_pl_node_control_flag);
	ros::Subscriber homeposition_sub = nh.subscribe<geometry_msgs::Point>("/waypoint/homeposition", 1, callback_homeposition);

	/// pub 	
	ros::Publisher landing_pad_pub = nh.advertise<geometry_msgs::Point>("pad_landing", 1);
	ros::Publisher landing_pad_measure_pub = nh.advertise<geometry_msgs::Point>("pad_landing_measure", 1);
	ros::Publisher landing_pad_lla_pub = nh.advertise<geometry_msgs::Point>("landing_pad_lla", 1);
	ros::Publisher kalmanfilter_pl_data_valid_flag_pub = nh.advertise<std_msgs::Int32>("kalmanfilter_pl_data_valid_flag", 1);


	geometry_msgs::Point landing_pad_msg;
	geometry_msgs::Point landing_pad_measure_msg;
	geometry_msgs::Point landing_pad_lla_msg; 

	double ref_position_lla[2] ={0.0};  //home pose

	ref_position_lla[0] =  homeposition_msg.x; // uv --> change need
	ref_position_lla[1] =  homeposition_msg.y;
	
	/*
	float x_trim = 0.3;		// gazebo trim value
	float y_trim = -0.1;
	*/

	//// !!!!!!!!!!!!!!!! check here before build !!!!!!!!!!!!!!!! ////

	float x_trim = 0.0;		// hanseo trim value
	float y_trim = 0.0;

	//typedef struct reserved_argument kalman_arg;
	
	/////////////////////////////////////////////////
	//////////////Kalman Init////////////////////////
	/////////////////////////////////////////////////

	Eigen::Matrix<double, 3, 3> A;
	Eigen::Matrix<double, 3, 3> H;
	Eigen::Matrix<double, 3, 3> Q;
	Eigen::Matrix<double, 3, 3> R;

	Eigen::Matrix<double, 3, 3> P;

	Eigen::Matrix<double, 3, 1> x;
	Eigen::Matrix<double, 3, 1> u;
	Eigen::Matrix<double, 3, 1> u1; // add
	

	///////////////      HJ temp      ///////////////
	Eigen::Matrix<double, 3, 3> K;
	Eigen::Matrix<double, 3, 3> B;
	/////////////////////////////////////////////////

	A << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;

	B << dt, 0, 0,
		0, dt, 0,
		0, 0, dt;

	H << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;

	Q << 0.2, 0, 0,
	     0, 0.2, 0,
		 0, 0, 0.2;

	R << 1.0, 0, 0,
		0, 1.0, 0,
		0, 0, 1.0;
		
	P << 2000, 0, 0,
		0, 2000, 0,
		0, 0, 1000;

	x << 0, 0, 0;

	/*
	K << 1395.459, 0, 969.03,
		0, 1397.915, 540.563,
		0, 0, 1;	
	

	K << 1383.4, 0, 960.16,
		0, 1384.6, 571.0516,
		0, 0, 1;
	*/
	/*
	K << 649.2953, 0, 294.8190,
		0, 651.8373 ,240.0295,
		0, 0 , 1 ;
	*/
	/* 
	K <<  635.8167, 0, 290.4553,	// original
    0, 638.8100, 247.1302,
    0, 0, 1;
	*/
	/*
	K <<  815.7886, 0, 320.5806,	// 0831 cali version
    0, 690.1654, 240.5677,
    0, 0, 1;
	*/

	//// !!!!!!!!!!!!!!!! check here before build !!!!!!!!!!!!!!!! ////
	
	/*
	K <<  696.219, 0, 319.05,	// CW version 0815 vali
    0, 562.312, 244.665,
    0, 0, 1;
	*/

	K <<  815.7886, 0, 320.5806,
    0, 690.1654, 240.5677,
    0, 0, 1;
	

	
	
	
	Eigen::Matrix<double, 3, 1> z;	

	Eigen::Matrix<double, 3, 1> z_NED;	
	

	/////////////////////////////////////////////////

	kalmanfilter kf1(A, H, Q, R, P, x, B);
	//uav_status uav(nh, "", 1);///////// ws check

	Eigen::Quaterniond q;
	
	Eigen::Matrix3d R_body_to_ned;
	

	std::string HeliPad_label("HeliPad");


	float altitude;
	float altitude_f;
	
	int is_new1 = 1;
	int isFirst = 1;
	int isInitialized = 0;
	int length = 0;
	int numofdection = 0;
	int class_idx= 0;

	double yolo_lat_diff = 0.0;
	double yolo_lon_diff = 0.0;
	double yolo_diff = 0.0;

	double Obj_Lat = 0.0;
	double Obj_Lon = 0.0;

	double Obj_Lat_list[20] = {0.0};
	double Obj_Lon_list[20] = {0.0};
	
	double Obj_Lat_except[10] = {0.0};
	double Obj_Lon_except[10] = {0.0};

	double pre_Obj_Lat = 0.0;
	double pre_Obj_Lon = 0.0;
	
	double lat2m = 110961.7060516;
	double lon2m = 89476.51124;
	double m2lat = 1/lat2m;
	double m2lon = 1/lon2m;
	
	double deg2rad = 3.141592/180;

	double lat_trim = 0.0;
	double lon_trim = 0.0;

	while(ros::ok())
	{
		
		ros::spinOnce();

		Eigen::Matrix<double, 3, 1> detections;
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////Exception/////////////////////////////////////////////////////
		while((is_new1)&&(ros::ok()) && kalmanfilter_pl_node_control_flag_msg.data)
		//while((is_new1)&&(ros::ok()))
		{	
			
			/*
			while((uav.velocity_ned.norm() > 3.0)&&(ros::ok()))
			{
				std::cout<<uav.velocity_ned.norm()<<std::endl;
				ros::spinOnce();
				rate.sleep();		

			}
			*/			

			ros::spinOnce();
			//std::cout<<"Update!!"<<std::endl;


			std::vector<Eigen::Matrix<double, 3, 1>> init_vector_set;
			std::cout<<"rpy_msg :: "<<fcc_rpy_msg<<std::endl;
			std::cout<<"---------------"<<std::endl;
			std::cout<<"CNNDetection info :: "<<CNNDetectionList_ws<<std::endl;

			init_vector_set.clear();

			if((float)CNNDetectionList_ws.x >= 1.0 && (float)CNNDetectionList_ws.y >= 1.0)                /////////////// center point
			{
				
				detections << (float)CNNDetectionList_ws.x, (float)CNNDetectionList_ws.y, 1.0000;
						
				///////////////////////////////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////// fcc data 0812 //////////////////////////////////////////////
				
				double roll_fcc = fcc_rpy_msg.x * deg2rad;
				double pitch_fcc = fcc_rpy_msg.y* deg2rad;
				double yaw_fcc = fcc_rpy_msg.z* deg2rad; 

				altitude_f = fcc_lla_msg.z;                  //// fcc alt data  
				float depth_f;								
				depth_f= altitude_f; 
				std::cout<<"depth_f       ::    "<<depth_f<<std::endl;

				////// gimbal euler state   ///////////////
				double roll_gimbal_f =0.0;
				double pitch_gimbal_f = -3.141592/2;//0.0; //3.141592/2; // 90 deg -  FRD Frame 
				double yaw_gimbal_f = 0.0;
				
				//R_body_to_ned = rotm_3d(roll_fcc,  pitch_fcc,  yaw_fcc);
				R_body_to_ned = rotm_gimbal_3d(roll_fcc,  pitch_fcc,  yaw_fcc);

				Eigen::Matrix<double, 3, 1> x_normal_coordinate_f = K.inverse() * detections;	
							
				Eigen::Matrix<double, 3, 1> x_normal_coordinate_FRD_unit;
				x_normal_coordinate_FRD_unit << x_normal_coordinate_f(2, 0), x_normal_coordinate_f(0, 0), x_normal_coordinate_f(1, 0);  // gimbal pitch 0 deg ver
				
				Eigen::Matrix<double, 3, 1> x_normal_coordinate_FRD;
				x_normal_coordinate_FRD = x_normal_coordinate_FRD_unit *  myAbs(depth_f/x_normal_coordinate_FRD_unit(0,0));

				Eigen::Matrix<double, 3, 3> R_gimbal2fcc_f = rotm_gimbal_3d(roll_gimbal_f, pitch_gimbal_f, yaw_gimbal_f).normalized();
				
				Eigen::Matrix<double, 3, 1> x_normal_coordinate_ned_f = R_body_to_ned*R_gimbal2fcc_f*x_normal_coordinate_FRD;
				
				z_NED << x_normal_coordinate_ned_f(0, 0), x_normal_coordinate_ned_f(1, 0), x_normal_coordinate_ned_f(2, 0);   
				
				///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

				std::cout<<"z_N(raw) 			::    "<<z_NED(0)<<std::endl;
				std::cout<<"z_E(raw) 			::    "<<z_NED(1)<<std::endl;
				std::cout<<"z_D(raw) 			::    "<<z_NED(2)<<std::endl;
				

				init_vector_set.push_back(z_NED); // init_vector_set.at(~);	
				
			}
			else
			{
				std::cout<<"CNNDetection info :: Nothing~! "<<std::endl;
			}


			for(int i = 0; i < init_vector_set.size(); i++)
			{
				Obj_Lat_list[i] = init_vector_set.at(i)[0];
				Obj_Lon_list[i] = init_vector_set.at(i)[1]; 

			}
			
			//std::cout<<"pre_Obj_Lat(up)   	::    "<<pre_Obj_Lat<<std::endl;
			//std::cout<<"Obj_Lat_list[0]       ::    "<<Obj_Lat_list[0]<<std::endl;
			//std::cout<<"Obj_Lat_list[1]       ::    "<<Obj_Lat_list[1]<<std::endl;
			//std::cout<<"Obj_Lat_list[2]       ::    "<<Obj_Lat_list[2]<<std::endl;
			//std::cout<<"Obj_Lat_list[3]       ::    "<<Obj_Lat_list[3]<<std::endl;
			//std::cout<<"Obj_Lat_list[4]       ::    "<<Obj_Lat_list[4]<<std::endl;
			//std::cout<<"Obj_Lat_list[5]       ::    "<<Obj_Lat_list[5]<<std::endl;
			//std::cout<<"Obj_Lat_list[6]       ::    "<<Obj_Lat_list[6]<<std::endl;
			//std::cout<<"Obj_Lat_list[7]       ::    "<<Obj_Lat_list[7]<<std::endl;
			//std::cout<<"Obj_Lat_list[8]       ::    "<<Obj_Lat_list[8]<<std::endl;
			//std::cout<<"Obj_Lat_list[9]       ::    "<<Obj_Lat_list[9]<<std::endl;
			//std::cout<<"Obj_Lat_list[10]      ::    "<<Obj_Lat_list[10]<<std::endl;
			
								
			//////////////////////////////
			
			double yolo_min_val = 100000.0;
			int yolo_min_idx = 0;
		
			for(int i=0; i < init_vector_set.size(); ++i)
			{
				yolo_lat_diff = (Obj_Lat_list[i] - pre_Obj_Lat)*(Obj_Lat_list[i] - pre_Obj_Lat);
				yolo_lon_diff = (Obj_Lon_list[i] - pre_Obj_Lon)*(Obj_Lon_list[i] - pre_Obj_Lon);

				yolo_diff = sqrt(yolo_lat_diff + yolo_lon_diff);

				if(yolo_min_val > yolo_diff)
				{
					yolo_min_val = yolo_diff;
					yolo_min_idx = i;
				}
			}

			init_vector_set.clear();  // clear set

			Obj_Lat = Obj_Lat_list[yolo_min_idx];  // save min value
			Obj_Lon = Obj_Lon_list[yolo_min_idx];


			for( int j = 0 ; j < 9 ; j++ )  // saves the last 10 values 
			{
				Obj_Lat_except[j] = Obj_Lat_except[j+1];
				Obj_Lon_except[j] = Obj_Lon_except[j+1];
			}
			Obj_Lat_except[9] = Obj_Lat;  // update last value
			Obj_Lon_except[9] = Obj_Lon;
			/*		
			std::cout<<"Obj_Lat_except_1       ::    "<<Obj_Lat_except[0]<<std::endl;	
			std::cout<<"Obj_Lat_except_2       ::    "<<Obj_Lat_except[1]<<std::endl;
			std::cout<<"Obj_Lat_except_3       ::    "<<Obj_Lat_except[2]<<std::endl;	
			std::cout<<"Obj_Lat_except_4       ::    "<<Obj_Lat_except[3]<<std::endl;
			std::cout<<"Obj_Lat_except_5       ::    "<<Obj_Lat_except[4]<<std::endl;	
			std::cout<<"Obj_Lat_except_6       ::    "<<Obj_Lat_except[5]<<std::endl;
			std::cout<<"Obj_Lat_except_7       ::    "<<Obj_Lat_except[6]<<std::endl;	
			std::cout<<"Obj_Lat_except_8       ::    "<<Obj_Lat_except[7]<<std::endl;
			std::cout<<"Obj_Lat_except_9       ::    "<<Obj_Lat_except[8]<<std::endl;
			std::cout<<"Obj_Lat_except_10      ::    "<<Obj_Lat_except[9]<<std::endl;
			*/

			//Outlier decision logic
			unsigned int except_cnt = 0;
			for (int j = 0; j < 10; j++)  //  0.5 seconds data
			{
				float x_diff = (Obj_Lat_except[j] - Obj_Lat);
				float y_diff = (Obj_Lon_except[j] - Obj_Lon);

				float dist_diff = sqrt(x_diff*x_diff + y_diff*y_diff);

				if( dist_diff > 0.25 )   // 20hz    4m/s
				{					
					except_cnt = except_cnt + 1;
				}									
			}

		
			if( except_cnt > 5 )
			{
		
				Obj_Lat = pre_Obj_Lat;
				Obj_Lon = pre_Obj_Lon;	
			}
			else
			{
		
				Obj_Lat = Obj_Lat;
				Obj_Lon = Obj_Lon;
			}


			pre_Obj_Lat = Obj_Lat;
			pre_Obj_Lon = Obj_Lon;	
			
	
			std::cout<<"------------------------"<<std::endl;	

			///////////////////////////////////////////////exception////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////////////////////////


			/////////////////////////////////////////////////////Kalman///////////////////////////////////////////////////////

			z << Obj_Lat, Obj_Lon, altitude_f;	
			u1 << fcc_vel_ned_msg.x, fcc_vel_ned_msg.y, fcc_vel_ned_msg.z ; // static model input	
			
			kf1.update(z, -u1, dt);	
			//kf1.update(z, -u1, dt);
			/////////////////////////////////////////////////////Kalman///////////////////////////////////////////////////////
			
			////////// data printf

			std::cout<<"z(N)       			::    "<<z(0)<<std::endl;
			std::cout<<"z(E)       			::    "<<z(1)<<std::endl;
			std::cout<<"z(D)       			::    "<<z(2)<<std::endl;

			std::cout<<"kalman(N)       	::    "<<kf1.x(0)<<std::endl;
			std::cout<<"kalman(E)       	::    "<<kf1.x(1)<<std::endl;
			std::cout<<"kalman(D)  			::    "<<kf1.x(2)<<std::endl;

			//std::cout<<"Local/Positon(N)       		::    "<<-uav.position_ned[0]<<std::endl;  //  reference
			//std::cout<<"Local/Positon(E)       		::    "<<-uav.position_ned[1]<<std::endl; //  reference
			//std::cout<<"Local/Positon(D) 				::    "<<uav.position_ned[2]<<std::endl; //ws

			//// pub msg generate
			lat_trim = x_trim*cos(fcc_rpy_msg.z)-y_trim*sin(fcc_rpy_msg.z);
			lon_trim = x_trim*sin(fcc_rpy_msg.z)+y_trim*cos(fcc_rpy_msg.z);
			std::cout<<"lat_trim(m) :: "<<lat_trim<<std::endl;
			std::cout<<"lon_trim(m) :: "<<lon_trim<<std::endl;

			if (z(2) > 5.0){
				landing_pad_lla_msg.x = fcc_lla_msg.x + kf1.x(0)* m2lat + lat_trim*m2lat;
				landing_pad_lla_msg.y = fcc_lla_msg.y + kf1.x(1)* m2lon + lon_trim*m2lon;
			}


			printf("lat(pub) ::  %7f\n",landing_pad_lla_msg.x);
			printf("lon(pub) ::  %7f\n",landing_pad_lla_msg.y);
			//std::cout<<"lat(pub)::    "<<landing_pad_lla_msg.x<<std::endl;  //  pub confirm
			//std::cout<<"lon(pub)::    "<<landing_pad_lla_msg.y<<std::endl<<std::endl;  //  pub confirm
						
			double distance_ref;  // distance from home point(== landing pad) -> not during mission
			distance_ref = sqrt ((landing_pad_lla_msg.x - ref_position_lla[0])*lat2m *(landing_pad_lla_msg.x - ref_position_lla[0])*lat2m + (landing_pad_lla_msg.y - ref_position_lla[1])*lon2m*(landing_pad_lla_msg.y - ref_position_lla[1])*lon2m);
			
			printf("Relaitive Distance_ref:: %f \n\n", distance_ref);

			landing_pad_msg.x = kf1.x(0);
			landing_pad_msg.y = kf1.x(1);
			landing_pad_msg.z = altitude_f;

			landing_pad_measure_msg.x = Obj_Lat;
			landing_pad_measure_msg.y = Obj_Lon;
			landing_pad_measure_msg.z = altitude_f;

			xp = A * x + B * u  // ?????? error 
			printf("P(0)       		        ::    %f\n", kf1.P(0));
			/*
			printf("P(1)     		        ::    %f\n", kf1.P(1));
			printf("P(2)      		        ::    %f\n", kf1.P(2));
			printf("P(3)      		        ::    %f\n", kf1.P(3));
			printf("P(4)       		        ::    %f\n", kf1.P(4));
			printf("P(5)       		        ::    %f\n", kf1.P(5));
			printf("P(6)       		        ::    %f\n", kf1.P(6));
			printf("P(7)       		        ::    %f\n", kf1.P(7));
			printf("P(8)       		        ::    %f\n", kf1.P(8));
			*/
            if(kf1.P(0)>0.5)
			{
				kalmanfilter_pl_data_valid_flag_msg.data = 0;
			}
			else
			{
				kalmanfilter_pl_data_valid_flag_msg.data = 1;
			}


			std::cout<<"-----------------------------------------------"<<std::endl;
			std::cout<<"-----------------------------------------------"<<std::endl;


			// msg publish
			landing_pad_pub.publish(landing_pad_msg);
			landing_pad_measure_pub.publish(landing_pad_measure_msg);	
			landing_pad_lla_pub.publish(landing_pad_lla_msg);
            kalmanfilter_pl_data_valid_flag_pub.publish(kalmanfilter_pl_data_valid_flag_msg);

			rate.sleep();
		
		}

	}	
	
	return 0;
}



void callback_yolo(const geometry_msgs::Point32::ConstPtr& msg)
{
  CNNDetectionList_ws = *msg;

}
void callback_yolo2(const geometry_msgs::Point32::ConstPtr& msg)
{
  CNNDetectionList2_ws = *msg;
}  

void callback_fcc_lla(const geometry_msgs::Point::ConstPtr& msg)
{
	fcc_lla_msg = *msg;
}

void callback_fcc_rpy(const geometry_msgs::Point::ConstPtr& msg)
{
	fcc_rpy_msg = *msg;
}

void callback_fcc_vel_ned(const geometry_msgs::Point::ConstPtr& msg)
{
	fcc_vel_ned_msg = *msg;
}

void callback_kalmanfilter_pl_node_control_flag(const std_msgs::Int32::ConstPtr& msg)
{
	kalmanfilter_pl_node_control_flag_msg = *msg;
}

void callback_homeposition(const geometry_msgs::Point::ConstPtr& msg){
  homeposition_msg = *msg;
}


Eigen::Matrix<double, 3, 3> rotm_gimbal_3d(double roll, double pitch, double yaw)
{
	Eigen::Matrix<double, 3, 3> Rx;
	Eigen::Matrix<double, 3, 3> Ry;
	Eigen::Matrix<double, 3, 3> Rz;
	Eigen::Matrix<double, 3, 3> R;

	Rx << 1, 0, 0,
	0, cos(roll), sin(roll),
	0, -sin(roll), cos(roll);
	Ry << cos(pitch), 0, -sin(pitch),
	0, 1, 0,
	sin(pitch), 0, cos(pitch);

	Rz << cos(yaw), sin(yaw), 0,
	-sin(yaw), cos(yaw), 0,
	0, 0, 1;
	R = Ry*Rx*Rz;
	return R.transpose();
}

Eigen::Matrix<double, 3, 3> rotm_3d(double roll, double pitch, double yaw)
{
	
	double m11;
	double m12;
	double m13;
	double m21;
	double m22;
	double m23;
	double m31;
	double m32;
	double m33;

	m11 = cos(pitch) * cos(yaw);
	m12 = cos(yaw)   * sin(roll) * sin(pitch) - cos(roll) * sin(yaw);
	m13 = sin(roll)  * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch);

	m21 = cos(pitch) * sin(yaw);
	m22 = cos(roll)  * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw);
	m23 = cos(roll)  * sin(pitch) * sin(yaw) - cos(yaw) * sin(roll);

	m31 = -sin(pitch);
	m32 = cos(pitch) * sin(roll);
	m33 = cos(roll)  * cos(pitch);
	
	Eigen::Matrix<double, 3, 3> A;

	A << m11, m21, m31,
		m12, m22, m32,
		m13, m23, m33;
		
	return A.transpose();
	
}
