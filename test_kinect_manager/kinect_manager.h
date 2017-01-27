#pragma once
#include <iostream>
#include "Kinect.h"

const unsigned int DEPTH_FLAG    = (1 << 0);
const unsigned int COLOR_FLAG    = (1 << 1);
const unsigned int INFRARED_FLAG = (1 << 2);
const unsigned int BODY_FLAG     = (1 << 3);

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

class KinectManager
{
public:
	KinectManager(const unsigned int source_flag) {
		HRESULT hr = GetDefaultKinectSensor(&kinect_sensor);
		if (FAILED(hr))
			std::cerr << "Error at GetDefaultKinectSensor : " << std::hex << hr << std::endl;

		if (FAILED(hr))
			std::cerr << "Error at get_CoordinateMapper : " << std::hex << hr << std::endl;

		hr = kinect_sensor->Open();
		if (FAILED(hr))
			std::cerr << "Error at Open: " << std::hex << hr << std::endl;

		// initialization of depth source
		if (source_flag & DEPTH_FLAG) {
			IDepthFrameSource* depth_frame_source;
			hr = kinect_sensor->get_DepthFrameSource(&depth_frame_source);
			if (FAILED(hr))
				std::cerr << "Error at get_DepthFrameSource: " << std::hex << hr << std::endl;

			hr = depth_frame_source->OpenReader(&depth_frame_reader);
			if (FAILED(hr))
				std::cerr << "Error at OpenReader: " << std::hex << hr << std::endl;

			hr = depth_frame_reader->SubscribeFrameArrived(&depth_frame_event);
			if (FAILED(hr))
				std::cerr << "Error at SubscribeFrameArrived: " << std::hex << hr << std::endl;

			IFrameDescription* depth_frame_description;
			hr = depth_frame_source->get_FrameDescription(&depth_frame_description);
			if (FAILED(hr))
				std::cerr << "Error at get_FrameDescription: " << std::hex << hr << std::endl;

			hr = depth_frame_description->get_Width(&depth_width);
			hr = depth_frame_description->get_Height(&depth_height);
			if (FAILED(hr))
				std::cerr << "Error at get_Width or get_Height: " << std::hex << hr << std::endl;

			SafeRelease(depth_frame_description);
			SafeRelease(depth_frame_source);
		}

		// initialization of color source
		if (source_flag & COLOR_FLAG)
			;// TODO : write color initialize procedure

	}

	int getDepthWidth() {
		return depth_width;
	}

	int getDepthHeight() {
		return depth_height;
	}

	bool getDepthData(UINT16* depth_buffer, INT64* relative_time) {
		DWORD result = WaitForSingleObjectEx(reinterpret_cast<HANDLE>(depth_frame_event), 0, FALSE);
		if (result != WAIT_OBJECT_0)
			return false;
		if (!depth_frame_event)
			return false;


		IDepthFrameArrivedEventArgs* depth_frame_arrived_event;
		HRESULT hr = depth_frame_reader->GetFrameArrivedEventData(depth_frame_event, &depth_frame_arrived_event);
		if (FAILED(hr)) {
			std::cerr << "Error at GetFrameArrivedEventData: " << std::hex << hr << std::endl;
			return false;
		}
			

		IDepthFrameReference* depth_frame_reference;
		hr = depth_frame_arrived_event->get_FrameReference(&depth_frame_reference);
		if (FAILED(hr)) {
			std::cerr << "Error at get_FrameReference: " << std::hex << hr << std::endl;
			return false;
		}

		IDepthFrame* depth_frame;
		hr = depth_frame_reference->AcquireFrame(&depth_frame);
		if (FAILED(hr)) {
			std::cerr << "Error at AcquireDepthFrame: " << std::hex << hr << std::endl;
			return false;
		}

		hr = depth_frame->get_RelativeTime(relative_time);
		hr = depth_frame->CopyFrameDataToArray(depth_width * depth_height, depth_buffer);
		if (FAILED(hr)) {
			std::cerr << "Error at CopyFrameDataToArray: " << std::hex << hr << std::endl;
			return false;
		}

		SafeRelease(depth_frame);
		SafeRelease(depth_frame_reference);
		SafeRelease(depth_frame_arrived_event);

		return true;
	}
	
	~KinectManager() {
		SafeRelease(kinect_sensor);
	}

	

private:

	IKinectSensor* kinect_sensor;

	ICoordinateMapper* coordinate_mapper;

	IDepthFrameReader* depth_frame_reader;
	WAITABLE_HANDLE depth_frame_event;
	int depth_width;
	int depth_height;

	IColorFrameReader* color_frame_reader;
	WAITABLE_HANDLE color_frame_event;
	int color_width;
	int color_height;

};
