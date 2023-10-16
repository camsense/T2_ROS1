
//#include "base/hclidar.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include"sensor_msgs/PointCloud.h"
#include"laser_geometry/laser_geometry.h"

#include <stdio.h>
#include <iostream>
#include <string> 
#include <sstream>
#include <algorithm>
#include <fstream>
#include<thread>

#include "base/HcData.h"
#include "base/HcSDK.h"
#include "base/hchead.h"

std::string  g_strLidarID = "";


#define  NOISE_FILTER_DEMO      0 //0,noise filter demo close,1 noise filter demo open
#define  PI		3.14159
#define DEG2RAD(DEG) ((DEG)*0.017453f)

int64_t hl_toMSec(const timespec & t)
{
    int64_t msec ;
    msec = t.tv_sec * 1000L ;
    msec += t.tv_nsec / 1000000L ;
    return msec ;
}


void sdkCallBackFunErrorCode(int iErrorCode)
{
	char buff[128] = { 0 };
	sprintf(buff, "[ERROR] Error code=%d\n",iErrorCode);
	printf(buff);
}

void sdkCallBackFunSecondInfo(tsSDKStatistic sInfo)
{
	/*printf("Main: sdkCallBackFunSecondInfo time=%lld s,points=%d,GrayBytes=%d,FPS=%d,speed=%0.2f,PPS=%d,valid=%d,invalid=%d,ErrorPacket=%d\n",
		sInfo.u64TimeStampS, sInfo.iNumPerPacket, sInfo.iGrayBytes, sInfo.u64FPS
		, sInfo.dRMS, sInfo.iPacketPerSecond, sInfo.iValid, sInfo.iInvalid
		, sInfo.u64ErrorPacketCount);*/

	std::string strFile = "";
	if(g_strLidarID.size() > DEFAULT_ID_LEN)
		strFile = "FPS_" + g_strLidarID.substr(g_strLidarID.size() - DEFAULT_ID_LEN, DEFAULT_ID_LEN) + ".csv";
	else
		strFile = "FPS_" + g_strLidarID + ".csv";
	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);

	char buff[128] = { 0 };
	sprintf(buff, "%lld,%d,%d,%lld,%0.2f,%d,%d,%d,%lld\n",
		sInfo.u64TimeStampS, sInfo.iNumPerPacket, sInfo.iGrayBytes, sInfo.u64FPS
		, sInfo.dRMS,  sInfo.iValid, sInfo.iInvalid, sInfo.iPacketPerSecond
		, sInfo.u64ErrorPacketCount);

	outFile.write(buff, strlen(buff));
	outFile.close();

//	printf(buff);
	
}

bool comp(const tsPointCloud ps1, const tsPointCloud ps2)
{
    return ps1.dAngleRaw < ps2.dAngleRaw;
}

void sdkCallBackFunPointCloud(LstPointCloud lstG, sensor_msgs::LaserScan &scan)
{

	ros::Time scan_time = ros::Time::now();
	timespec start_time,end_time;
	start_time.tv_sec=lstG[0].u64TimeStampMs;
	start_time.tv_nsec=lstG[0].u64TimeStampNs;
	int pointCount = lstG.size();
	end_time.tv_sec=lstG[pointCount - 1].u64TimeStampMs;
	end_time.tv_nsec=lstG[pointCount - 1].u64TimeStampNs;
	float scanDuration = hl_toMSec(end_time) - hl_toMSec(start_time);

	sort(lstG.begin(), lstG.end(), comp);
	std::string strFile;
	if (g_strLidarID.size() > DEFAULT_ID_LEN)
		strFile = "Raw_" + g_strLidarID.substr(g_strLidarID.size() - DEFAULT_ID_LEN, DEFAULT_ID_LEN) + ".csv";
	else
		strFile = "Raw_" + g_strLidarID + "msec.csv";

	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);

	char buff[128] = { 0 };
	sprintf(buff, "----------- Get PointCloud => Size=%zu\n", lstG.size());
	printf("%s", buff);

	scan.header.frame_id = "map";
	scan.header.stamp = scan_time;
	scan.scan_time = scanDuration;
    scan.angle_min = DEG2RAD(lstG[0].dAngleRaw);
    scan.angle_max = DEG2RAD(lstG[pointCount - 1].dAngleRaw);
	scan.angle_increment = 2 * PI / pointCount;
    scan.time_increment = scanDuration / pointCount; 
    scan.range_min = 0.0;  
    scan.range_max = 15.0; 
    scan.ranges.resize(pointCount);
    scan.intensities.resize(pointCount);
    for(unsigned int i = 0; i < pointCount; ++i)
    {
            scan.ranges[i] = lstG[i].u16Dist / 1000.0f; 
            scan.intensities[i] = lstG[i].u16Gray;
    }

    for(auto sInfo : lstG)
    {
		//if (sInfo.dAngle > 310 || sInfo.dAngle < 50)
		//{

			memset(buff, 0, 128);
			sprintf(buff, "%lld,%0.3f,%0.3f,%d,%d,%d,%d\n",
				sInfo.u64TimeStampNs, 
				sInfo.dAngle, 
				sInfo.dAngleRaw, 
				sInfo.u16Dist,
				sInfo.bValid,
				sInfo.u16Speed, 
				sInfo.u16Gray);


			outFile.write(buff, strlen(buff));

			//printf(buff);
		//}

		
    }
		
	outFile.close();
	
}

void readFileOfIfstream(LstPointCloud& lstG)
{
	
	std::ifstream csv_file("RawData.ini", std::ios::in);
	std::string line;

	if (!csv_file.is_open())
	{
		std::cout << "Error: opening file fail" << std::endl;
		return;//std::exit(1);
	}

	std::istringstream sin;         //将整行字符串line读入到字符串istringstream中
	std::vector<std::string> words; //声明一个字符串向量
	std::string word;

	// 读取标题行
	//std::getline(csv_data, line);
	// 读取数据
	while (std::getline(csv_file, line))
	{
		sin.clear();
		sin.str(line);
		words.clear();
		//while (std::getline(sin, word, ',')) 
		while (std::getline(sin, word, ' ')) 
		{
			words.push_back(word); 
			//std::cout << word;
			// std::cout << atol(word.c_str());
		}
		if (words.size() >= 2)
		{
			tsPointCloud sInfo;
			sInfo.bValid = true;
			std::string str = words.at(0);
			sInfo.dAngle = atof(str.c_str());

			str = words.at(1);
			sInfo.u16Dist = (UINT16)atof(str.c_str());
			lstG.push_back(sInfo);
		}
		
		std::cout << std::endl;
		// do something。。。
	}
	csv_file.close();

}


int main(int argc, char** argv)
{
	
    ros::init(argc, argv, "scan_publisher");
    ros::NodeHandle n;
	laser_geometry::LaserProjection projector;
    
    ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 500);
	ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud",1000);

	std::string strPort="/dev/ttyUSB0";
	int iBaud = 230400;
	bool bPollMode = true;
	std::string strLidarModel = "T2A1";

    int rtn = 0;
    bool bLoop = false;
	std::string strVer = getSDKVersion();
    std::cout << "Main: SDK version=" << strVer.c_str()<< std::endl;

    auto funErrorCode = std::bind(sdkCallBackFunErrorCode, std::placeholders::_1);
	setSDKCallBackFunErrorCode(funErrorCode);

    auto funSecondInfo = std::bind(sdkCallBackFunSecondInfo, std::placeholders::_1);
    setSDKCallBackFunSecondInfo(funSecondInfo);

    if(!bPollMode)//call back
    {
        auto funPointCloud = std::bind(sdkCallBackFunPointCloud, std::placeholders::_1, std::placeholders::_2);
        setSDKCallBackFunPointCloud(funPointCloud);

    }

    int iRotationalSpeed {360};//设置转速
    bool isGetCircleData {true};//按圈读取
	int iReadTimeoutMs = 2;//读取串口数据超时

	setSDKAngOffset(true);//启用零度角修正，需要配合雷达上电获取属性包，部分型号支持（D2系列）。
    if (isGetCircleData){
        setSDKCircleDataMode();//按圈获取点云
    }
	rtn = hcSDKInitialize(strPort.c_str(), strLidarModel.c_str(), iBaud, iReadTimeoutMs, false, bLoop, bPollMode);

    if (rtn != 1)
    {
		hcSDKUnInit();
		printf("Main: Init sdk failed!\n");
		getchar();
		exit(0);
		return 0;
        
    }

    if (strLidarModel == T200){
        setSDKT2LidarStart(false);
        std::cout << "Please input any char to start the T2 lidar" << std::endl;
        setSDKT2LidarStart(true);
    }else {
        setSDKLidarPowerOn(true);//通知camsense SDK 雷达已经上电
        setSDKPointCloudLattice(true);
    }

	if (strLidarModel == "X2MF")
	{
		setSDKLidarLowSpeed(true);
	}else if (strLidarModel == T200){
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        setSDKT2RotationalSpeed(iRotationalSpeed);
    }
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	g_strLidarID = getSDKLidarID();
	printf( "Main:Lidar ID=%s\n" , getSDKLidarID());

	UINT64 u64LastTimeNs = HCHead::getCurrentTimestampNs();
	int iCount = 0;
    while (true)
    {

        if(bPollMode)
        {
			LstPointCloud lstG;

			if (getSDKRxPointClouds(lstG))
			{

				if (lstG.size() > 0)
				{
					UINT64 u64CurrentTimeNs = HCHead::getCurrentTimestampNs();
					double fDeltaMs = (u64CurrentTimeNs - u64LastTimeNs)/1e6;
					u64LastTimeNs = u64CurrentTimeNs;

					printf("Main: Delta time=%f\n", fDeltaMs);

					if(fDeltaMs>500.0)
						printf("Main: -----------------------------------------------\n");

					sensor_msgs::LaserScan scan; 
					sdkCallBackFunPointCloud(lstG,scan);
					scan_pub.publish(scan);

					sensor_msgs::PointCloud cloud;
					projector.projectLaser(scan,cloud);
					pointcloud_pub.publish(cloud);

				}

			}

			int iError = getSDKLastErrCode();
			if (iError != LIDAR_SUCCESS)
			{
				printf("Main: Poll Rx Points error code=%d\n", iError);
				switch (iError)
				{
				case ERR_SHARK_MOTOR_BLOCKED://堵转消息
					break;
				case ERR_SHARK_INVALID_POINTS://雷达被遮挡
					break;
				case ERR_DISCONNECTED://连接丢失
					break;
				case ERR_RX_CONTINUE://持续接收校验错误包
					break;
				case ERR_LIDAR_FPS_INVALID:
					break;
				default:
					break;
				}
			}
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::this_thread::yield();
    }

	hcSDKUnInit();
    return 0;

}
