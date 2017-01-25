#pragma once
#include <iostream>

#include "Kinect.h"

const unsigned int DEPTH_HEIGHT = 424;
const unsigned int DEPTH_WIDTH  = 512;

const unsigned int COLOR_HEIGHT = 1080;
const unsigned int COLOR_WIDTH  = 1920;



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

		WCHAR buffer_id[256];
		hr = kinect_sensor->get_UniqueKinectId(256, buffer_id);
		if (FAILED(hr))
			std::cerr << "Error at get_UniqueKinectId : " << std::hex << hr << std::endl;
		std::wstring wid(buffer_id);
		kinect_id.assign(wid.begin(), wid.end());

		hr = kinect_sensor->get_CoordinateMapper(&coordinate_mapper);
		if (FAILED(hr))
			std::cerr << "Error at get_CoordinateMapper : " << std::hex << hr << std::endl;

		hr = kinect_sensor->Open();
		if (FAILED(hr))
			std::cerr << "Error at Open : " << std::hex << hr << std::endl;

		CameraIntrinsics camera_intrinsics;
		hr = coordinate_mapper->GetDepthCameraIntrinsics(&camera_intrinsics);
		if (FAILED(hr))
			std::cerr << "Error at GetDepthCameraIntrinsics : " << std::hex << hr << std::endl;
		depth_camera_focal_length_x    = camera_intrinsics.FocalLengthX;
		depth_camera_focal_length_y    = camera_intrinsics.FocalLengthY;
		depth_camera_principal_point_x = camera_intrinsics.PrincipalPointX;
		depth_camera_principal_point_y = camera_intrinsics.PrincipalPointY;
		
		hr = kinect_sensor->OpenMultiSourceFrameReader(source_flag, &multi_source_frame_reader);
		if (FAILED(hr))
			std::cerr << "Error at OpenMultiSourceFrameReader : " << std::hex << hr << std::endl;

		hr = multi_source_frame_reader->SubscribeMultiSourceFrameArrived(&waitable_handle);
		if (FAILED(hr))
			std::cerr << "Error at SubscribeMultiSourceFrameArrived : " << std::hex << hr << std::endl;
		/* initialization of depth source
		if (source_flag & FrameSourceTypes_Depth) {
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
		if (source_flag & FrameSourceTypes_Color) {
			IColorFrameSource* color_frame_source;
			hr = kinect_sensor->get_ColorFrameSource(&color_frame_source);
			if (FAILED(hr))
				std::cerr << "Error at get_ColorFrameSource: " << std::hex << hr << std::endl;

			hr = color_frame_source->OpenReader(&color_frame_reader);
			if (FAILED(hr))
				std::cerr << "Error at OpenReader: " << std::hex << hr << std::endl;

			hr = color_frame_reader->SubscribeFrameArrived(&color_frame_event);
			if (FAILED(hr))
				std::cerr << "Error at SubscribeFrameArrived: " << std::hex << hr << std::endl;

			IFrameDescription* color_frame_description;
			hr = color_frame_source->get_FrameDescription(&color_frame_description);
			if (FAILED(hr))
				std::cerr << "Error at get_FrameDescription: " << std::hex << hr << std::endl;

			hr = color_frame_description->get_Width(&color_width);
			hr = color_frame_description->get_Height(&color_height);
			if (FAILED(hr))
				std::cerr << "Error at get_Width or get_Height: " << std::hex << hr << std::endl;

			SafeRelease(color_frame_description);
			SafeRelease(color_frame_source);
		}
		*/
	}

	int getDepthHeight() {
		return DEPTH_HEIGHT;
	}

	int getDepthWidth() {
		return DEPTH_WIDTH;
	}

	int getColorHeight() {
		return COLOR_HEIGHT;
	}

	int getColorWidth() {
		return COLOR_WIDTH;
	}

	bool isValidColorRange(int x, int y)
	{
		return ((0 <= x) && (x < COLOR_WIDTH)) && ((0 <= y) && (y < COLOR_HEIGHT));
	}

	bool isValidDepthRange(int index, const UINT16* depth_buffer)
	{
		return (500 <= depth_buffer[index]) && (depth_buffer[index] <= 8000);
	}

	bool getColoredDepthAndDepthImage(BYTE* colored_depth_buffer, UINT16* depth_buffer, INT64* relative_time) {
		DWORD result = WaitForSingleObjectEx(reinterpret_cast<HANDLE>(waitable_handle), 0, FALSE);
		if (result != WAIT_OBJECT_0)
			return false;
		if (!waitable_handle)
			return false;

		IMultiSourceFrameArrivedEventArgs* multi_source_frame_arrived_event;
		HRESULT hr = multi_source_frame_reader->GetMultiSourceFrameArrivedEventData(waitable_handle, &multi_source_frame_arrived_event);
		if (FAILED(hr))
			std::cerr << "Error at GetMultiSourceFrameArrivedEventData : " << std::hex << hr << std::endl;

		IMultiSourceFrameReference* multi_source_frame_reference;
		hr = multi_source_frame_arrived_event->get_FrameReference(&multi_source_frame_reference);
		if (FAILED(hr))
			std::cerr << "Error at get_FrameReference : " << std::hex << hr << std::endl;

		IMultiSourceFrame* multi_source_frame;
		hr = multi_source_frame_reference->AcquireFrame(&multi_source_frame);
		if (FAILED(hr))
			std::cerr << "Error at AcquireFrame : " << std::hex << hr << std::endl;

		IDepthFrameReference* depth_frame_reference;
		hr = multi_source_frame->get_DepthFrameReference(&depth_frame_reference);
		if (FAILED(hr)) {
			std::cerr << "Error at get_DepthFrameReference: " << std::hex << hr << std::endl;
			return false;
		}

		IDepthFrame* depth_frame;
		hr = depth_frame_reference->AcquireFrame(&depth_frame);
		if (FAILED(hr)) {
			std::cerr << "Error at AcquireFrame: " << std::hex << hr << std::endl;
			return false;
		}

		hr = depth_frame->get_RelativeTime(relative_time);
		hr = depth_frame->CopyFrameDataToArray(DEPTH_WIDTH * DEPTH_HEIGHT, depth_buffer);
		if (FAILED(hr)) {
			std::cerr << "Error at CopyFrameDataToArray: " << std::hex << hr << std::endl;
			return false;
		}

		SafeRelease(depth_frame);
		SafeRelease(depth_frame_reference);

		IColorFrameReference* color_frame_reference;
		hr = multi_source_frame->get_ColorFrameReference(&color_frame_reference);
		if (FAILED(hr)) {
			std::cerr << "Error at get_FrameReference: " << std::hex << hr << std::endl;
			return false;
		}


		IColorFrame* color_frame;
		hr = color_frame_reference->AcquireFrame(&color_frame);
		if (FAILED(hr)) {
			std::cerr << "Error at AcquireFrame: " << std::hex << hr << std::endl;
			return false;
		}

		const unsigned int color_buffer_size = COLOR_WIDTH * COLOR_HEIGHT * 4 * sizeof(BYTE);
		BYTE* color_buffer = (BYTE*)malloc(color_buffer_size);
		hr = color_frame->CopyConvertedFrameDataToArray(color_buffer_size, color_buffer, ColorImageFormat_Bgra);
		if (FAILED(hr)) {
			std::cerr << "Error at CopyConvertedFrameDataToArray: " << std::hex << hr << std::endl;
			return false;
		}

		const unsigned int depth_buffer_size = DEPTH_WIDTH * DEPTH_HEIGHT;
		std::vector<ColorSpacePoint> color_space_points(depth_buffer_size);
		hr = coordinate_mapper->MapDepthFrameToColorSpace(depth_buffer_size, depth_buffer, color_space_points.size(), &color_space_points[0]);
		if (FAILED(hr)) {
			std::cerr << "Error at MapDepthFrameToColorSpace: " << std::hex << hr << std::endl;
			return false;
		}

		for (int i = 0; i < depth_buffer_size; ++i) {
			int x = (int)color_space_points[i].X;
			int y = (int)color_space_points[i].Y;

			int src_index = ((y * COLOR_WIDTH) + x) * 4;
			int dest_index = i * 4;

			if (isValidColorRange(x, y) && isValidDepthRange(i, depth_buffer)) {
				colored_depth_buffer[dest_index + 0] = color_buffer[src_index + 0];
				colored_depth_buffer[dest_index + 1] = color_buffer[src_index + 1];
				colored_depth_buffer[dest_index + 2] = color_buffer[src_index + 2];
			}
			else {
				colored_depth_buffer[dest_index + 0] = 255;
				colored_depth_buffer[dest_index + 1] = 255;
				colored_depth_buffer[dest_index + 2] = 255;
			}
		}

		SafeRelease(depth_frame);
		SafeRelease(depth_frame_reference);
		SafeRelease(color_frame);
		SafeRelease(color_frame_reference);
		SafeRelease(multi_source_frame);
		SafeRelease(multi_source_frame_reference);
		SafeRelease(multi_source_frame_arrived_event);

		return true;

	}
	/*
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
			std::cerr << "Error at AcquireFrame: " << std::hex << hr << std::endl;
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

	bool getColoredDepthData(UINT16* depth_buffer, BYTE* colored_depth_buffer) {
		DWORD result = WaitForSingleObjectEx(reinterpret_cast<HANDLE>(color_frame_event), 0, FALSE);
		if (result != WAIT_OBJECT_0)
			return false;
		if (!color_frame_event)
			return false;

		IColorFrameArrivedEventArgs* color_frame_arrived_event;
		HRESULT hr = color_frame_reader->GetFrameArrivedEventData(color_frame_event, 
			&color_frame_arrived_event);
		if (FAILED(hr)) {
			std::cerr << "Error at GetFrameArrivedEventData: " << std::hex << hr << std::endl;
			return false;
		}


		IColorFrameReference* color_frame_reference;
		hr = color_frame_arrived_event->get_FrameReference(&color_frame_reference);
		if (FAILED(hr)) {
			std::cerr << "Error at get_FrameReference: " << std::hex << hr << std::endl;
			return false;
		}


		IColorFrame* color_frame;
		hr = color_frame_reference->AcquireFrame(&color_frame);
		if (FAILED(hr)) {
			std::cerr << "Error at AcquireFrame: " << std::hex << hr << std::endl;
			return false;
		}

		const unsigned int color_buffer_size = color_width * color_height * 4 * sizeof(BYTE);
		BYTE* color_buffer = (BYTE*)malloc(color_buffer_size);
		hr = color_frame->CopyConvertedFrameDataToArray(color_buffer_size, color_buffer, ColorImageFormat_Bgra);
		if (FAILED(hr)) {
			std::cerr << "Error at CopyConvertedFrameDataToArray: " << std::hex << hr << std::endl;
			return false;
		}

		std::vector<ColorSpacePoint> color_space_points(color_width * color_height);
		const unsigned int depth_buffer_size = depth_width * depth_height;
		hr = coordinate_mapper->MapDepthFrameToColorSpace(depth_buffer_size, depth_buffer, color_space_points.size(), &color_space_points[0]);
		if (FAILED(hr)) {
			std::cerr << "Error at MapDepthFrameToColorSpace: " << std::hex << hr << std::endl;
			return false;
		}

		for (int i = 0; i < depth_buffer_size; ++i) {
			int x = (int)color_space_points[i].X;
			int y = (int)color_space_points[i].Y;

			int src_index = ((y * color_width) + x) * 4;
			int dest_index = i * 4;

			if (isValidColorRange(x, y) && isValidDepthRange(i, depth_buffer)) {
				colored_depth_buffer[dest_index + 0] = color_buffer[src_index + 0];
				colored_depth_buffer[dest_index + 1] = color_buffer[src_index + 1];
				colored_depth_buffer[dest_index + 2] = color_buffer[src_index + 2];
			}
			else {
				colored_depth_buffer[dest_index + 0] = 255;
				colored_depth_buffer[dest_index + 1] = 255;
				colored_depth_buffer[dest_index + 2] = 255;
			}
		}

		return true;
	}
	*/
	~KinectManager() {
		SafeRelease(kinect_sensor);
	}

	

private:

	IKinectSensor* kinect_sensor;

	std::string kinect_id;

	ICoordinateMapper* coordinate_mapper;

	float depth_camera_focal_length_x;
	float depth_camera_focal_length_y;
	float depth_camera_principal_point_x;
	float depth_camera_principal_point_y;

	IMultiSourceFrameReader* multi_source_frame_reader;
	WAITABLE_HANDLE waitable_handle;

	//IDepthFrameReader* depth_frame_reader;
	//WAITABLE_HANDLE depth_frame_event;
	//int depth_width;
	//int depth_height;

	//IColorFrameReader* color_frame_reader;
	//WAITABLE_HANDLE color_frame_event;
	//int color_width;
	//int color_height;

};