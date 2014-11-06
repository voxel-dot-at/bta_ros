/******************************************************************************
 * Copyright (c) 2014
 * VoXel Interaction Design GmbH
 *
 * @author Angel Merino Sastre
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 ******************************************************************************/

/** @mainpage Bta ROS driver
 *
 * @section intro_sec Introduction
 *
 * This software defines a interface for working with all ToF cameras from 
 * Bluetechnix GmbH supported by their API.
 *
 * @section install_sec Installation
 *
 * We encorage you to follow the instruction we prepared in:
 *
 * ROS wiki: http://wiki.ros.org/bta_ros
 * Github repository: https://github.com/voxel-dot-at/bta_ros
 *
 */

#include <bta_ros/bta_ros.hpp>

namespace bta_ros 
{

	BtaRos::BtaRos(ros::NodeHandle nh_camera, 
                ros::NodeHandle nh_private, 
	              std::string nodeName) : 
	  nh_(nh_camera),
	  nh_private_(nh_private),
	  it_(nh_camera),
	  cim_tof_(nh_camera),
	  nodeName_(nodeName),
	  config_init_(false) 
	{
	
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
		    ros::console::levels::Debug) ) {
   		ros::console::notifyLoggerLevelsChanged();
		}
		
	}
	
	BtaRos::~BtaRos() 
	{
		printf("clossing!!!!!");
		close();
	}
	

	void BtaRos::close() 
	{
		ROS_DEBUG("Close called");
		if (BTAisConnected(handle_)) {
			ROS_DEBUG("Clossing..");
			BTA_Status status;
			status = BTAclose(&handle_);
			printf("done: %d \n", status);
	 	}
		
		return;
	}

	
	void BtaRos::callback(bta_ros::bta_rosConfig &config_, uint32_t level)
	{
		BTA_Status status;
		// Check the configuretion parameters with those given in the initialization
		int it;
		double fr;
		uint32_t usValue;

		if (!config_init_) {
			if (nh_private_.getParam(nodeName_+"/integrationTime",it)) {
				usValue = (uint32_t)it;
				status = BTAsetIntegrationTime(handle_, usValue);
				if (status != BTA_StatusOk)
					ROS_WARN_STREAM("Error setting IntegrationTime:: " << status << "---------------");	
			} else {
                status = BTAgetIntegrationTime(handle_, &usValue);
                if (status != BTA_StatusOk)
					ROS_WARN_STREAM("Error reading IntegrationTime: " << status << "---------------");
				else
					nh_private_.setParam(nodeName_+"/integrationTime", (int)usValue);
			}
			nh_private_.getParam(nodeName_+"/integrationTime",config_.Integration_Time);
			
			if (nh_private_.getParam(nodeName_+"/frameRate",fr)) {
				status = BTAsetFrameRate(handle_, fr);
				if (status != BTA_StatusOk)
					ROS_WARN_STREAM("Error setting FrameRate: " << status << "---------------");	
			} else {
				float fr_f;
				status = BTAgetFrameRate(handle_, &fr_f);
				fr = fr_f;
				if (status != BTA_StatusOk)
					ROS_WARN_STREAM("Error reading FrameRate: " << status << "---------------");
				else
					nh_private_.setParam(nodeName_+"/frameRate", fr);
			}
			nh_private_.getParam(nodeName_+"/frameRate",config_.Frame_rate);
            config_init_ = true;
            return;
		}
		
		
		nh_private_.getParam(nodeName_+"/integrationTime",it);
		if(it != config_.Integration_Time) {
			usValue = (uint32_t)config_.Integration_Time;
			status = BTAsetIntegrationTime(handle_, usValue);
				if (status != BTA_StatusOk)
					ROS_WARN_STREAM("Error setting IntegrationTime: " << status << "---------------");	
				else
					nh_private_.setParam(nodeName_+"/integrationTime", config_.Integration_Time);
		}
		
		nh_private_.getParam(nodeName_+"/frameRate",fr);
		if(fr != config_.Frame_rate) {
			usValue = (uint32_t)config_.Frame_rate;
			status = BTAsetFrameRate(handle_, usValue);
				if (status != BTA_StatusOk)
					ROS_WARN_STREAM("Error setting FrameRate: " << status << "---------------");	
				else
					nh_private_.setParam(nodeName_+"/frameRate", config_.Frame_rate);
		}

        if(config_.Read_reg) {
            try {
                std::stringstream ss;
                it = strtoul(config_.Reg_addr.c_str(), NULL, 0);
                status = BTAreadRegister(handle_, it, &usValue, 0);
                if (status != BTA_StatusOk) {
                    ROS_WARN_STREAM("Could not read reg: " << config_.Reg_addr << ". Status: " << status);
                }
                ss<<"";
                ss  << "0x"<< std::hex << usValue;
                ss >> config_.Reg_val;
                ROS_INFO_STREAM("Read register: " << config_.Reg_addr << ". Value: " << ss.str());
            } catch(const boost::bad_lexical_cast &) {
                ROS_WARN_STREAM("Wrong address: " << config_.Reg_addr << " " << it);
            }
            config_.Read_reg = false;
        }
        if(config_.Write_reg) {
            //std::stringstream ss;
            it = strtoul(config_.Reg_addr.c_str(), NULL, 0);
            usValue = strtoul(config_.Reg_val.c_str(), NULL, 0);

            status = BTAwriteRegister(handle_, it, &usValue, 0);
            if (status != BTA_StatusOk) {
                ROS_WARN_STREAM("Could not write reg: " <<
                    config_.Reg_addr <<
                    " with val: " <<
                    config_.Reg_val <<
                    ". Status: " << status);
            }
            ROS_INFO_STREAM("Written register: " << config_.Reg_addr << ". Value: " << config_.Reg_val);
            BTAgetIntegrationTime(handle_, &usValue);
            config_.Integration_Time = usValue;
            BTAsetFrameRate(handle_, fr);
            config_.Frame_rate = fr;
            config_.Write_reg = false;

        }

	}

	void BtaRos::publishData() 
	{

		BTA_Status status;

		BTA_Frame *frame;
    status = BTAgetFrame(handle_, &frame, 3000);
    if (status != BTA_StatusOk) {
       	return;
    }

    ROS_DEBUG("		frameArrived FrameCounter %d", frame->frameCounter);
		
		BTA_DataFormat dataFormat;
		BTA_Unit unit;
		uint16_t xRes, yRes;
		
		sensor_msgs::CameraInfoPtr ci_tof(new sensor_msgs::CameraInfo(cim_tof_.getCameraInfo()));
		ci_tof->header.frame_id = nodeName_+"/tof_camera";
		
		
	  uint16_t *distances;
	  status = BTAgetDistances(frame, (void **)&distances, &dataFormat, &unit, &xRes, &yRes);
	  if (status == BTA_StatusOk) {
	    sensor_msgs::ImagePtr dis (new sensor_msgs::Image);
      if (dataFormat == BTA_DataFormatUInt16) {
        if (unit == BTA_UnitMillimeter) {
        	dis->header.seq = frame->frameCounter;
        	dis->header.stamp.sec = frame->timeStamp;
        	dis->height = yRes;
        	dis->width = xRes;
        	dis->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        	dis->step = yRes*sizeof(uint16_t);
        	dis->data.resize(xRes*yRes*sizeof(uint16_t));
        	memcpy ( &dis->data[0], distances, xRes*yRes*sizeof(uint16_t) );
        }
      }
      dis->header.frame_id = nodeName_+"/tof_camera";
	    pub_dis_.publish(dis,ci_tof);
	  }
	
	  bool ampOk = false;
	  uint16_t *amplitudes;
	  status = BTAgetAmplitudes(frame, (void **)&amplitudes, &dataFormat, &unit, &xRes, &yRes);
	  if (status == BTA_StatusOk) {
	    sensor_msgs::ImagePtr amp (new sensor_msgs::Image);
      if (dataFormat == BTA_DataFormatUInt16) {
        if (unit == BTA_UnitUnitLess) {
          amp->header.seq = frame->frameCounter;
        	amp->header.stamp.sec = frame->timeStamp;
        	amp->height = yRes;
        	amp->width = xRes;
        	amp->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        	amp->step = yRes*sizeof(uint16_t);
        	amp->data.resize(xRes*yRes*sizeof(uint16_t));
        	memcpy ( &amp->data[0], amplitudes, xRes*yRes*sizeof(uint16_t) );
        }
      }
	    amp->header.frame_id = nodeName_+"/tof_camera";
	    pub_amp_.publish(amp,ci_tof);
	    ampOk = true;
	  }
		  	  
		int16_t *xCoordinates, *yCoordinates, *zCoordinates;
    printf("BTAgetXYZcoordinates()\n");
    status = BTAgetXYZcoordinates(frame, (void **)&xCoordinates, (void **)&yCoordinates, (void **)&zCoordinates, &dataFormat, &unit, &xRes, &yRes);
    if (status == BTA_StatusOk) {
      sensor_msgs::PointCloud2Ptr xyz (new sensor_msgs::PointCloud2);
      if (dataFormat == BTA_DataFormatSInt16) {
        if (unit == BTA_UnitMillimeter) {
          printf("Got 3D data\n");
          pcl::PointCloud<pcl::PointXYZI> cloud;
          for (size_t i = 0; i < yRes*xRes; ++i) {
            pcl::PointXYZI temp_point;
            temp_point.x = xCoordinates[i];
            temp_point.y = yCoordinates[i];
            temp_point.z = zCoordinates[i];
            if (ampOk)
              temp_point.intensity = amplitudes[i];
            else
              temp_point.intensity = 255;
            cloud.push_back(temp_point);
            
          }
          cloud.width = xRes;
          cloud.height = yRes;
          cloud.is_dense = true;
          pcl::toROSMsg(cloud, *xyz);
        }
      }
      xyz->header.seq = frame->frameCounter;
      xyz->header.stamp.sec = frame->timeStamp;
      xyz->header.frame_id = nodeName_+"/tof_camera";
      pub_xyz_.publish(xyz);
    } 
		/*
		 * Publishing the messages
		 */
		/*sensor_msgs::CameraInfoPtr ci_tof(new sensor_msgs::CameraInfo(cim_tof_.getCameraInfo()));
		ci_tof->header.frame_id = nodeName_+"/tof_camera";
		dis->header.frame_id = nodeName_+"/tof_camera";
		amp->header.frame_id = nodeName_+"/tof_camera";*/
	
		BTAfreeFrame(&frame);
	
	  //pub_amp_.publish(amp,ci_tof);
		//pub_dis_.publish(dis,ci_tof);

	}
	
  /*void BtaRos::ampCb(const sensor_msgs::ImagePtr& amp)
  {
  	sensor_msgs::CameraInfoPtr ci_tof(new sensor_msgs::CameraInfo(cim_tof_.getCameraInfo()));
		ci_tof->header.frame_id = nodeName_+"/tof_camera";
		amp->header.frame_id = nodeName_+"/tof_camera";
	
		pub_amp_.publish(amp,ci_tof);
  }
    
  void BtaRos::disCb(const sensor_msgs::ImagePtr& dis)
  {
  	sensor_msgs::CameraInfoPtr ci_tof(new sensor_msgs::CameraInfo(cim_tof_.getCameraInfo()));
		ci_tof->header.frame_id = nodeName_+"/tof_camera";
		dis->header.frame_id = nodeName_+"/tof_camera";
	
		pub_dis_.publish(dis,ci_tof);
  }*/

	void BtaRos::parseConfig() {
		int iusValue;
		nh_private_.param(nodeName_+"/udpDataIpAddrLen",iusValue,4);
		config_.udpDataIpAddrLen = (uint8_t)iusValue;
		
		ROS_DEBUG_STREAM("config_.udpDataIpAddrLen: " << (int)config_.udpDataIpAddrLen);
		if (nh_private_.hasParam(nodeName_+"/udpDataIpAddr")) {
			ROS_DEBUG_STREAM("config_.udpDataIpAddr:");
			for (int i = 1; i <= config_.udpDataIpAddrLen; i++) {
				std::ostringstream to_string;
				to_string << "";
				to_string << nodeName_ << "/udpDataIpAddr/n" << i;
				nh_private_.getParam(to_string.str(),iusValue);
				udpDataIpAddr_[i-1] = (uint8_t)iusValue;
				ROS_DEBUG_STREAM((int)udpDataIpAddr_[i-1] << ","); 
			}
			config_.udpDataIpAddr = udpDataIpAddr_;
		}
		nh_private_.param(nodeName_+"/udpDataPort",iusValue,10002);
		config_.udpDataPort = (uint16_t)iusValue;
		ROS_DEBUG_STREAM("config_.udpDataPort: " << config_.udpDataPort);  
		
	 	if(nh_private_.getParam(nodeName_+"/udpControlOutIpAddrLen",iusValue))
	 		config_.udpControlOutIpAddrLen = (uint8_t)iusValue;
		ROS_DEBUG_STREAM("config_.udpControlOutIpAddrLen: " << (int)config_.udpControlOutIpAddrLen);
		if (nh_private_.hasParam(nodeName_+"/udpControlOutIpAddr")) {
			ROS_DEBUG_STREAM("config_.udpControlOutIpAddr:");
			for (int i = 1; i <= config_.udpControlOutIpAddrLen; i++) {
				std::ostringstream to_string;
				to_string << "";
				to_string << nodeName_ << "/udpControlOutIpAddr/n" << i;
				nh_private_.getParam(to_string.str(),iusValue);
				udpControlOutIpAddr_[i-1] = (uint8_t)iusValue;
				ROS_DEBUG_STREAM((int)udpControlOutIpAddr_[i-1] << ","); 
			}
			config_.udpControlOutIpAddr = udpControlOutIpAddr_;
		}
		if(nh_private_.getParam(nodeName_+"/udpControlOutPort",iusValue))
			config_.udpControlOutPort = (uint16_t)iusValue; 
		ROS_DEBUG_STREAM("config_.udpControlOutPort: " << (int)config_.udpControlOutPort);
		
		if(nh_private_.getParam(nodeName_+"/udpControlInIpAddrLen",iusValue))
			config_.udpControlInIpAddrLen = (uint8_t)iusValue;
		ROS_DEBUG_STREAM("config_.udpControlInIpAddrLen: " << (int)config_.udpControlInIpAddrLen);
		if (nh_private_.hasParam(nodeName_+"/udpControlInIpAddr")) {
			
			ROS_DEBUG_STREAM("config_.udpControlInIpAddr:");
			for (int i = 1; i <= config_.udpControlInIpAddrLen; i++) {
				std::ostringstream to_string;
				to_string << "";
				to_string << nodeName_ << "/udpControlInIpAddr/n" << i;
				nh_private_.getParam(to_string.str(),iusValue);
				udpControlInIpAddr_[i-1] = (uint8_t)iusValue; 
				ROS_DEBUG_STREAM((int)udpControlInIpAddr_[i-1] << ","); 
			}
			config_.udpControlInIpAddr = udpControlInIpAddr_;
		}
		if(nh_private_.getParam(nodeName_+"/udpControlInPort",iusValue))
		  	config_.udpControlInPort = (uint16_t)iusValue; 
		ROS_DEBUG_STREAM("config_.udpControlInPort: " << (int)config_.udpControlInPort);
	  
		nh_private_.param(nodeName_+"/tcpDeviceIpAddrLen",iusValue,4);
		config_.tcpDeviceIpAddrLen = (uint8_t)iusValue;
		ROS_DEBUG_STREAM("config_.tcpDeviceIpAddrLen: " << (int)config_.tcpDeviceIpAddrLen);
		if (nh_private_.hasParam(nodeName_+"/tcpDeviceIpAddr")) {
			ROS_DEBUG_STREAM("TCP address:");
			for (int i = 1; i <= config_.tcpDeviceIpAddrLen; i++) {
				std::ostringstream to_string;
				to_string << "";
				to_string << nodeName_ << "/tcpDeviceIpAddr/n" << i;
				nh_private_.getParam(to_string.str(),iusValue);
				tcpDeviceIpAddr_[i-1] = (uint8_t)iusValue;
				ROS_DEBUG_STREAM((int)tcpDeviceIpAddr_[i-1] << ",");  
			}
			config_.tcpDeviceIpAddr = tcpDeviceIpAddr_;
		}
		nh_private_.param(nodeName_+"/tcpControlPort",iusValue,10001);
		config_.tcpControlPort = (uint16_t)iusValue;
		ROS_DEBUG_STREAM("config_.tcpControlPort: " << config_.tcpControlPort);
		
		if (nh_private_.getParam(nodeName_+"/tcpDataPort",iusValue))  
			config_.tcpDataPort = (uint16_t)iusValue;
	
		nh_private_.getParam(nodeName_+"/uartPortName",uartPortName_);
		config_.uartPortName = (int8_t *)uartPortName_.c_str();
	
		if(nh_private_.getParam(nodeName_+"/uartBaudRate",iusValue))
			config_.uartBaudRate = (uint32_t)iusValue;
		if(nh_private_.getParam(nodeName_+"/uartDataBits",iusValue))
			config_.uartDataBits = (uint8_t)iusValue;
		if(nh_private_.getParam(nodeName_+"/uartStopBits",iusValue))
			config_.uartStopBits = (uint8_t)iusValue;
		if(nh_private_.getParam(nodeName_+"/uartParity",iusValue))
			config_.uartParity = (uint8_t)iusValue;
		if(nh_private_.getParam(nodeName_+"/uartTransmitterAddress",iusValue))
			config_.uartTransmitterAddress = (uint8_t)iusValue;
		if(nh_private_.getParam(nodeName_+"/uartReceiverAddress",iusValue))
			config_.uartReceiverAddress = (uint8_t)iusValue;
		if(nh_private_.getParam(nodeName_+"/serialNumber",iusValue))
			config_.serialNumber = (uint32_t)iusValue;

		nh_private_.getParam(nodeName_+"/calibFileName",calibFileName_);
		config_.calibFileName = (uint8_t *)calibFileName_.c_str();
	
		int32_t frameMode;
		nh_private_.param(nodeName_+"/frameMode",frameMode,(int)BTA_FrameModeDistAmp);
		config_.frameMode = (BTA_FrameMode)frameMode;
	
		nh_private_.param(nodeName_+"/verbosity",iusValue,5);
		config_.verbosity = (uint8_t)iusValue;
	
		nh_private_.param(nodeName_+"/frameQueueLength",iusValue,1);
		config_.frameQueueLength = (uint16_t)iusValue;
	
		int32_t frameQueueMode;
		nh_private_.param(nodeName_+"/frameQueueMode",frameQueueMode,(int)BTA_QueueModeDropOldest);
		config_.frameQueueMode = (BTA_QueueMode)frameQueueMode;
	
		//config_.frameArrived = &frameArrived;
		config_.infoEvent = &infoEventCb;
	}



	int BtaRos::connectCamera() {
		BTA_Status status;
		BTA_DeviceInfo *deviceInfo;
		
		// Init camera connection
		//ros::Duration().sleep();
		status = (BTA_Status)-1;
		for (int i=0; i<10; i++) {
			ROS_DEBUG_STREAM("Connecting... try " << i+1);
			status = BTAopen(&config_, &handle_);
			if (status != BTA_StatusOk) {
				if (!nh_private_.ok() && ros::isShuttingDown())
					return -1;
				ROS_WARN_STREAM("Could not connect to the camera. status: " << status);
				continue;
			}
			break;
		}
		if (!BTAisConnected(handle_)) {
		 		ROS_WARN_STREAM("Could not connect to the camera.");
		 		return -1;
		}
		
		ROS_INFO_STREAM("Camera connected sucessfully. status: " << status);
		status = BTAgetDeviceInfo(handle_, &deviceInfo);
		if (status != BTA_StatusOk) {
			ROS_WARN_STREAM("Could not get device info. status: " << status);
		    return status;
		}
		
		ROS_DEBUG_STREAM("Retrieved device info: \n" 
			<< "deviceType: " << deviceInfo->deviceType << "\n"
			<< "serialNumber: " << deviceInfo->serialNumber << "\n"
			<< "firmwareVersionMajor: " << deviceInfo->firmwareVersionMajor << "\n"
			<< "firmwareVersionMinor: " << deviceInfo->firmwareVersionMinor << "\n"
			<< "firmwareVersionNonFunc: " << deviceInfo->firmwareVersionNonFunc 
			<< "\n");

		ROS_INFO_STREAM("Service running: " << (int)BTAisRunning(handle_));
		ROS_INFO_STREAM("Connection up: " << (int)BTAisConnected(handle_));
		
		BTAfreeDeviceInfo(deviceInfo);
		return 1;
	}
	
	int BtaRos::initialize() 
	{

		/*
		 * Camera config_
		 */
		
		BTAinitConfig(&config_);  
	
		parseConfig();
		
		//int_amp = nh_private_.advertise<sensor_msgs::Image>("bta_node_amp", 0);
		//int_dis = nh_private_.advertise<sensor_msgs::Image>("bta_node_dis", 0);
		
		/*
		 * Camera Initialization
		 */
		ROS_DEBUG_STREAM("Config Readed sucessfully");
		
		BTA_Status status;
		if (connectCamera() < 0)
			return -1;
		
		reconfigure_server_.reset(new ReconfigureServer(nh_private_));
		reconfigure_server_->setCallback(boost::bind(&BtaRos::callback, this, _1, _2));
		
		while (!config_init_)
		{
			ROS_DEBUG("Waiting for dynamic reconfigure configuration.");
			boost::this_thread::sleep(boost::posix_time::seconds(0.1));
		}
		ROS_DEBUG("Dynamic reconfigure configuration received.");
		
		/*
		 * ROS Node Initialization
		 */
		 {
		// Advertise all published topics
			cim_tof_.setCameraName(nodeName_);
			if(cim_tof_.validateURL(
					/*camera_info_url_*/"package://bta_ros/calib.yml")) {
				cim_tof_.loadCameraInfo("package://bta_ros/calib.yml");
				ROS_INFO_STREAM("Loaded camera calibration from " << 
					"package://bta_ros/calib.yml"	);
			} else {
				ROS_WARN_STREAM("Camera info at: " <<
					"package://bta_ros/calib.yml" <<
					" not found. Using an uncalibrated config_.");
			} 	
		
			pub_amp_ = it_.advertiseCamera(nodeName_ + "/tof_camera/image_raw", 1);
			pub_dis_ = it_.advertiseCamera(nodeName_ + "/tof_camera/compressedDepth", 1);
			pub_xyz_ = nh_private_.advertise<sensor_msgs::PointCloud2> (nodeName_ + "/tof_camera/point_cloud_xyz", 1);
	
			//sub_amp_ = nh_private_.subscribe("bta_node_amp", 1, &BtaRos::ampCb, this);
			//sub_dis_ = nh_private_.subscribe("bta_node_dis", 1, &BtaRos::disCb, this);
		}
		
		while (nh_private_.ok() && !ros::isShuttingDown()) {
			if (!BTAisConnected(handle_)) {
		 		ROS_WARN_STREAM("The camera got disconnected." << BTAisConnected(handle_));
	 			if (connectCamera() < 0)
					break;
			}
	 			
			publishData();
			ros::spinOnce ();	
		}
		return 0;
	}
}

