#pragma once 
#ifndef KINNECT_WRAPPER_V1_HPP
#define KINNECT_WRAPPER_V1_HPP 1

#include <Windows.h>
#include <NuiApi.h>
#pragma comment(lib, "Kinect10.lib")
#include <iostream>
#include <vector>
#include <Eigen/Dense>

class KinectWrapper {
public : 
	INuiSensor* _sensor;
	HANDLE hDepthPlayerEvent;
	HANDLE hDepthPlayerHandle;
	INuiCoordinateMapper* pCordinateMapper;
	NUI_IMAGE_RESOLUTION resolution;


	KinectWrapper ( void ) : _sensor (NULL) {
		this->hDepthPlayerEvent  = INVALID_HANDLE_VALUE;
		this->hDepthPlayerHandle = INVALID_HANDLE_VALUE;
		this->pCordinateMapper = NULL;
		this->resolution = NUI_IMAGE_RESOLUTION_640x480;
		return;
	}

	~KinectWrapper ( void ) {
		this->_sensor->NuiShutdown();
		if ( this->pCordinateMapper != NULL ) this->pCordinateMapper->Release();
		CloseHandle( hDepthPlayerEvent );
		return;
	}

	bool init( void ) {
		std::cerr<<"initializing Kinect.";
		int num_sensor;
		if ( FAILED ( ::NuiGetSensorCount(&num_sensor) ) )      return false;
		if ( num_sensor == 0 ) {
			std::cerr<<"no sensor found"<<std::endl;
			return false;
		}
		std::cerr<<".";
		if ( FAILED ( ::NuiCreateSensorByIndex(0, &(this->_sensor) ) ) ) return false;
		std::cerr<<".";
		if ( FAILED ( this->_sensor->NuiInitialize( NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX) ) ) return false;
		std::cerr<<".";
		this->hDepthPlayerEvent = CreateEvent( nullptr, true, false, nullptr );
		std::cerr<<".";
		if ( FAILED (this->_sensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, resolution, 0, 2, hDepthPlayerEvent, &hDepthPlayerHandle ) ) ) {
			std::cerr << "Error : NuiImageStreamOpen( DEPTH&PLAYER )" << std::endl;
			return false;
		}
		std::cerr<<".";	
		if (FAILED(this->_sensor->NuiGetCoordinateMapper( &pCordinateMapper ) ) ) {
			std::cerr << "Error : NuiGetCoordinateMapper" << std::endl;
			return false;
		}
		std::cerr<<"done."<<std::endl;
		return true;

	}

	bool getPointSet(std::vector<Eigen::Vector3d> & points ){
		int width;
		int height;
		this->getWindowSize(width, height);


		HANDLE hEvents[1] = { hDepthPlayerEvent };
		ResetEvent( this->hDepthPlayerEvent );
		WaitForMultipleObjects( 1, hEvents, true, INFINITE );
		
		// Depthセンサーからフレームを取得
		NUI_IMAGE_FRAME depthPlayerImageFrame = { 0 };
		if ( FAILED (this->_sensor->NuiImageStreamGetNextFrame( this->hDepthPlayerHandle, 0, &depthPlayerImageFrame ) ) ) {
			std::cerr << "Error : NuiImageStreamGetNextFrame( DEPTH&PLAYER )" << std::endl;
			return false;
		}

		// Depth&Playerデータの取得
		BOOL nearMode = false;

		INuiFrameTexture* pDepthPlayerFrameTexture = nullptr;
		this->_sensor->NuiImageFrameGetDepthImagePixelFrameTexture( hDepthPlayerHandle, &depthPlayerImageFrame, &nearMode, &pDepthPlayerFrameTexture );
		NUI_LOCKED_RECT rect = {0};
		pDepthPlayerFrameTexture->LockRect(0, &rect, 0, 0);
		NUI_DEPTH_IMAGE_PIXEL* depthPixel = (NUI_DEPTH_IMAGE_PIXEL *)rect.pBits;
		int npixel = width * height;
		points.reserve(npixel);
		for( int y = 0; y < height; y++ ){
			for( int x = 0; x < width; x++ ){
				unsigned int index = y * width + x;
				unsigned short depth = depthPixel[index].depth;
				Vector4 p = NuiTransformDepthImageToSkeleton(x, y, depth, NUI_IMAGE_RESOLUTION_640x480);
				points.push_back(Eigen::Vector3d(-p.x, p.y, -p.z));
			}
		}
		pDepthPlayerFrameTexture->UnlockRect( 0 );
		this->_sensor->NuiImageStreamReleaseFrame( hDepthPlayerHandle, &depthPlayerImageFrame );

		return true;
	}

	void getWindowSize( int& w , int& h) {
		unsigned long refWidth = 0;
		unsigned long refHeight = 0;
		NuiImageResolutionToSize( resolution, refWidth, refHeight );
		w = static_cast<int>( refWidth );
		h = static_cast<int>( refHeight );
		return;
	}
};


#endif //KINECT_WRAPPER_V1_HPP