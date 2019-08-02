#include "360VR_player.h"

// player class 
void VR360_Player::player_init(cv::Size omnidirectinalSize, float phi_i, float theta_i, float psi_i, float W_i, float H_i, float fov_i)
{
	//////////////////////////////////////
	// omnidirectional frame info 저장	//
	//////////////////////////////////////
	omnidirectionalW = omnidirectinalSize.width;
	omnidirectionalH = omnidirectinalSize.height;
	
	//////////////////////
	// viewport 초기화	//
	//////////////////////
	
	// 해상도
	viewportImage = cv::Mat(cv::Size((int)W_i, (int)H_i), CV_8UC3);
	viewportNormPlane = cv::Mat(cv::Size((int)W_i, (int)H_i), CV_32FC3);

	viewportTable = cv::Mat(cv::Size((int)W_i, (int)H_i), CV_32FC2);

	// initial position
	phi = phi_i* PI / 180;
	theta = theta_i * PI / 180;
	psi = psi_i * PI / 180;

	viewportW = W_i;
	viewportH = H_i;

	fov = fov_i;
	float fovx = fov_i * PI / 180.f;
	float fovy = atan2f(tanf(fov_i * PI / 360.f) * viewportH / viewportW, 1) * 2;

	float cx = viewportW / 2.f;
	float cy = viewportH / 2.f;

	float fx = (viewportW / 2) / tanf(fovx / 2);
	float fy = (viewportH / 2) / tanf(fovy / 2);

	K[0][0] = fx; K[0][1] = 0;  K[0][2] = cx;
	K[1][0] = 0;  K[1][1] = fy; K[1][2] = cy;
	K[2][0] = 0;  K[2][1] = 0;  K[2][2] = 1;

	std::cout << "intrinsic matix of viewport" << std::endl;
	std::cout << K[0][0] << "	" << K[0][1] << "	" << K[0][2] << std::endl;
	std::cout << K[1][0] << "	" << K[1][1] << "	" << K[1][2] << std::endl;
	std::cout << K[2][0] << "	" << K[2][1] << "	" << K[2][2] << std::endl;

	float K_inv[3][3];
	float cameraCoordinate[3];
	float ImageCoordinate[3];

	inverse_matrix3x3(K, K_inv);
	ImageCoordinate[0] = 0.f;		ImageCoordinate[1] = 0.f;		ImageCoordinate[2] = 1.f;

	for (int v = 0; v < viewportH; ++v)
	{
		ImageCoordinate[1] = (float)v + 0.5f;

		for (int u = 0; u < viewportW; ++u)
		{
			ImageCoordinate[0] = (float)u + 0.5f;
			multiply_3x3and3x1(K_inv, ImageCoordinate, cameraCoordinate);
			normalize(cameraCoordinate);

			viewportNormPlane.at<Vec3f>(v, u)[0] = cameraCoordinate[0];
			viewportNormPlane.at<Vec3f>(v, u)[1] = cameraCoordinate[1];
			viewportNormPlane.at<Vec3f>(v, u)[2] = cameraCoordinate[2];
		}
	}
	stopIndex = 0;
	frameIndex = 0;
	printf("\n...viewport player_initialized...\n");
}

void VR360_Player::player_reinit(float fov_i)
{
	
	fov = fov_i;
	float fovx = fov_i * PI / 180.f;
	float fovy = atan2f(tanf(fov_i * PI / 360.f) * viewportH / viewportW, 1) * 2;

	float cx = viewportW / 2.f;
	float cy = viewportH / 2.f;

	float fx = (viewportW / 2) / tanf(fovx / 2);
	float fy = (viewportH / 2) / tanf(fovy / 2);
	
	K[0][0] = fx; K[0][1] = 0;  K[0][2] = cx;
	K[1][0] = 0;  K[1][1] = fy; K[1][2] = cy; 
	K[2][0] = 0;  K[2][1] = 0;  K[2][2] = 1;
	
	std::cout << "intrinsic matix of viewport" << std::endl;
	std::cout << K[0][0] << "	" << K[0][1] << "	" << K[0][2] << std::endl;
	std::cout << K[1][0] << "	" << K[1][1] << "	" << K[1][2] << std::endl;
	std::cout << K[2][0] << "	" << K[2][1] << "	" << K[2][2] << std::endl;
	
	float K_inv[3][3];
	float cameraCoordinate[3];
	float ImageCoordinate[3];
	
	inverse_matrix3x3(K, K_inv);
	ImageCoordinate[0] = 0.f;		ImageCoordinate[1] = 0.f;		ImageCoordinate[2] = 1.f;
	
	for (int v = 0; v < viewportH; ++v)
	{
		ImageCoordinate[1] = (float)v + 0.5f;

		for (int u = 0; u < viewportW; ++u)
		{
			ImageCoordinate[0] = (float)u + 0.5f;
			multiply_3x3and3x1(K_inv, ImageCoordinate, cameraCoordinate);
			normalize(cameraCoordinate);

			viewportNormPlane.at<Vec3f>(v, u)[0] = cameraCoordinate[0];
			viewportNormPlane.at<Vec3f>(v, u)[1] = cameraCoordinate[1];
			viewportNormPlane.at<Vec3f>(v, u)[2] = cameraCoordinate[2];
		}
	}
	printf("\n...viewport player_reinitialized...\n");
}

void VR360_Player::getImage(cv::Mat src)
{	
	src.copyTo(imgOmnidirectional);
	imgOmnidirectionalShow = src.clone();
}

void VR360_Player::viewport_rotation()
{
	//ctheta = cosf(theta);
	cphi = cosf(phi);
	cpsi = cosf(psi);
	//stheta = sinf(theta);
	sphi = sinf(phi);
	spsi = sinf(psi);

	//Rx
	Rx[0][0] = 1;			Rx[0][1] = 0;		Rx[0][2] = 0;
	Rx[1][0] = 0;			Rx[1][1] = cphi;	Rx[1][2] = -sphi;
	Rx[2][0] = 0;			Rx[2][1] = sphi;	Rx[2][2] = cphi;
	/*
	//Ry
	Ry[0][0] = 1;		Ry[0][1] = 0;		Ry[0][2] = stheta;
	Ry[1][0] = 0;			Ry[1][1] = 1;		Ry[1][2] = 0;
	Ry[2][0] = -stheta;		Ry[2][1] = 0;		Ry[2][2] = ctheta;
	*/
	//Rz
	Rz[0][0] = cpsi;		Rz[0][1] = -spsi;		Rz[0][2] = 0;
	Rz[1][0] = spsi;		Rz[1][1] = cpsi;		Rz[1][2] = 0;
	Rz[2][0] = 0;			Rz[2][1] = 0;			Rz[2][2] = 1;

	multiply_3x3and3x3(Rz, Rx, R_wc);
	//ltiply_3x3and3x3(R1, , R);
}

void VR360_Player::key_input()
{

	c = cv::waitKey(1);
	if (c == 'w')
	{
		phi += 5.f *PI / 180.f;
	}
	else if (c == 's')
	{
		phi -= 5.f*PI / 180.f;
	}
	else if (c == '.') // 다음영상
	{
		if(frameIndex <100)
			frameIndex++;
		else
			printf("last frame\n");
	}
	else if (c == ',') // 이전영상
	{
		if (frameIndex >0)
			frameIndex--;
		else
			printf("first frame\n");
	}
	else if (c == 'a')
	{
		psi -= 10.f*PI / 180.f;
	}
	else if (c == 'd')
	{
		psi += 10.f*PI / 180.f;
	}
	else if (c == '[')
	{
		player_reinit(fov+= 10.f);
	}
	else if (c == ']')
	{
		player_reinit(fov -= 10.f);
	}
	else if (c == 32)
	{
		stopIndex = (stopIndex+1)%2;
		printf("stop Index: %d\n",stopIndex);
		if(stopIndex)
			printf("pause\n");
		else
			printf("start\n");
	}
	else if (c == 'i')
	{
		theta = 0;
		phi = 0;
		psi = 0;
	}
	else if (c == 't')
	{
		float R_cw[3][3];
		inverse_matrix3x3(R_wc, R_cw);

		float P[3][3];
		multiply_3x3and3x3(K, R_cw, P);

		std::cout << "3x4 projection metrix: " << std::endl;

		std::cout << P[0][0] << " " << P[0][1] << " " << P[0][2] << " 0 ";
		std::cout << P[1][0] << " " << P[1][1] << " " << P[1][2] << " 0 ";
		std::cout << P[2][0] << " " << P[2][1] << " " << P[2][2] << std::endl;		

		cv::String viewportFileName = "viewport" + std::to_string(stopIndex) + ".png";

		cv::imwrite(viewportFileName, viewportImage);

		stopIndex++;
	}

	if (c == 27)// 27: esc key
	{
		printf("finish\n");
		cv::destroyWindow("VIEWPORT");
	}

	// maintain degree within 360
	if (phi >= 2 * PI)
		phi -= 2 * PI;

	else if (phi <= -2 * PI)
		phi += 2 * PI;

	if (theta >= 2 * PI)
		theta -= 2 * PI;
	else if (theta <= -2 * PI)
		theta += 2 * PI;
}

void VR360_Player::mapping()
{
	
	float fisheyeCameraXYZ[3] = { 0,0,0 };
	float fisheyeCameraTP[2] = {0,0};

	Point2f X;
	float viewportXYZ[3];

	float* pviewportXYmap = (float*)viewportTable.data;
	float* pviewportNormPlane = (float*)viewportNormPlane.data;

	int viewportPixelNum = viewportH * viewportW;

	for (int i = 0; i < viewportPixelNum; ++i)
	{
		viewportXYZ[0] = pviewportNormPlane[0];
		viewportXYZ[1] = pviewportNormPlane[1]; 
		viewportXYZ[2] = pviewportNormPlane[2];	
		pviewportNormPlane += 3;
		multiply_3x3and3x1(R_wc, viewportXYZ, fisheyeCameraXYZ);
		
		xyz2tp(fisheyeCameraXYZ, fisheyeCameraTP);
	
		X = fisheyeCam.genericprojextented(fisheyeCameraTP[0], fisheyeCameraTP[1]);
		pviewportXYmap[0] = X.x;
		pviewportXYmap[1] = X.y;
		pviewportXYmap += 2;	

	}

	remap(imgOmnidirectional, viewportImage, viewportTable, Mat(), cv::INTER_CUBIC);
}

void VR360_Player::show(float sizeRatio, float o_time)
{
	char buf[255];
		
	int myFontFace = 1;
	double myFontSize = 0.9;

	cv::Mat viewportImage_buf = viewportImage.clone();

	//sprintf_s(buf, "(%dx%d), x: %.1f, y: %.1f", (int)viewportW, (int)viewportH, fovx * 180.f/PI, fovy * 180.f / PI);
	//cv::putText(imgViewport_buf, buf, cvPoint(viewportW/2- 50, viewportH - 20), myFontFace, myFontSize, CV_RGB(255, 0, 0));

	// print text on screen

	sprintf(buf, "<VIEWPORT> phi: %.1f, theta: %.1f", phi / PI * 180, theta / PI * 180);
	cv::putText(viewportImage_buf, buf, cvPoint(10, 10), myFontFace, myFontSize, CV_RGB(255, 0, 0));
	
	if (o_time != 0)
	{
		sprintf_s(buf, "Processing time: %.2f", o_time);
		cv::putText(viewportImage_buf, buf, cvPoint(10, 30), myFontFace, myFontSize, CV_RGB(255, 0, 0));

		sprintf_s(buf, "FPS: %.2f", 1 / o_time * 1000.);
		cv::putText(viewportImage_buf, buf, cvPoint(10, 45), myFontFace, myFontSize, CV_RGB(255, 0, 0));
	}
			
	cv::imshow("viewport", viewportImage_buf);
	cv::resize(imgOmnidirectionalShow, imgOmnidirectionalShow, Size(imgOmnidirectionalShow.size().width * sizeRatio, imgOmnidirectionalShow.size().height * sizeRatio));
	cv::imshow("Omnidirectional video", imgOmnidirectionalShow);
}

void VR360_Player::release()
{
	cv::destroyAllWindows();
}