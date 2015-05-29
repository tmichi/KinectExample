#pragma once 
#ifndef KINNECT_WRAPPER_V2_HPP
#define KINNECT_WRAPPER_V2_HPP 1

#include <Windows.h>
#include <Kinect.h>
#pragma comment(lib, "Kinect20.lib")
#include <iostream>
#include <vector>
#include <Eigen/Dense>


#define DEBUG_MSG std::cerr << __FILE__ << ":" << __LINE__ << std::endl;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


class KinectWrapper {
private:
	IKinectSensor* _sensor;
	IDepthFrameReader* _depthReader;
	IDepthFrameSource* _frameSource;
public:
	KinectWrapper(void) : _sensor(NULL), _depthReader(NULL),  _frameSource(NULL) {
		return;
	}

	~KinectWrapper(void) {
		SafeRelease(this->_frameSource);
		SafeRelease(this->_depthReader);
		if (this->_sensor) {
			this->_sensor->Close();
		}
		SafeRelease(this->_sensor);
		return;
	}

	bool init(void) {
		std::cerr << "initializing Kinect... ";
		if (FAILED(GetDefaultKinectSensor(&_sensor))){
			std::cerr << " default kinect sensor was not got." << std::endl;
			return false;
		}
		if (FAILED(this->_sensor->Open())) {
			return false;
		}
		if (FAILED(this->_sensor->get_DepthFrameSource(&_frameSource))) {
			std::cerr << "depth frame source" << std::endl;
			return false;
		}
		if ( FAILED(this->_frameSource->OpenReader(&_depthReader) ) ){
			std::cerr << "open reader" << std::endl;
			return false;
		}
		if (!_sensor ) {
			std::cerr << "failed" << std::endl;
			return false;
		}
		else {
			std::cerr << "ok" << std::endl;
			return true;
		}
	}


	bool getPointSet(std::vector<Eigen::Vector3d>& points){
	
		if (!this->_depthReader)
		{
			return false;
		}

		HRESULT result = S_OK;
		IFrameDescription* description;
		int width = 0;
		int height = 0;
		unsigned short minDepth = 0;
		unsigned short maxDepth = 0;

		unsigned int numBuffer = 0;
		INT64 nTime = 0;
		IDepthFrame* frame;
		if (FAILED(this->_depthReader->AcquireLatestFrame(&frame))){
			return false;
		}

		if (FAILED(frame->get_RelativeTime(&nTime))) {
			return false;
		}

		if ( FAILED (frame->get_FrameDescription(&description) ) ) {
			return false;
		}

		description->get_Width(&width);
		description->get_Height(&height);
		numBuffer = width * height;
		std::vector<UINT16> buf(numBuffer, 0);
		UINT16* buffer = &buf[0];

		if ( FAILED(this->_frameSource->get_DepthMinReliableDistance(&minDepth) ) ){
			return false;
		}
		if ( FAILED(this->_frameSource->get_DepthMaxReliableDistance(&maxDepth) ) ) {
			return false;
		}
		if ( FAILED ( frame->AccessUnderlyingBuffer(&numBuffer, &buffer) ) ) {
			return false;
		}

		ICoordinateMapper* mapper;
		if (FAILED(this->_sensor->get_CoordinateMapper(&mapper))) {
			return false;
		}
		for (int y = 0; y < height; ++y){
			for (int x = 0; x < width; ++x){
				CameraSpacePoint csp;
				DepthSpacePoint dsp = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = buffer[x + y*width];
				if (depth < minDepth) continue;
				if (maxDepth < depth) continue;
				mapper->MapDepthPointToCameraSpace(dsp, depth, &csp);
				if (isinf(csp.X)) continue;
				if (isinf(csp.Y)) continue;
				if (isinf(csp.Z)) continue;
				points.push_back(Eigen::Vector3d(csp.X, csp.Y, csp.Z));
			}
		}

		SafeRelease(description);
		SafeRelease(frame);
		return true;
	}
};

#endif //KINECT_WRAPPER_HPP