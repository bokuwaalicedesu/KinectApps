#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <sstream>
#include <map>
#include<time.h>

#include <Kinect.h>
#include <opencv2\opencv.hpp>

#include <atlbase.h>


// 次のように使います
// ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );
// 書籍での解説のためにマクロにしています。実際には展開した形で使うことを検討してください。
#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
    }

class KinectApp
{
private:

	// Kinect SDK
	CComPtr<IKinectSensor> kinect = nullptr;
	CComPtr<ICoordinateMapper> coordinateMapper = nullptr;
	CComPtr<IColorFrameReader> colorFrameReader = nullptr;


	int colorWidth;
	int colorHeight;
	int count = 0,n = 0;
	int X[5], Y[5],color[5], targetColor,tag[5];
	bool touchCircle[5];
	bool wait = true;
	bool touch = false;
	int rightHandX, rightHandY,leftHandX,leftHandY,R=0,G=0,B=0,counter=0,R1 = 0, G2 = 0, B3 = 0;
	unsigned int colorBytesPerPixel;
	int itarator = 0,k=0;
	char time_c[256];
	int time2 = 0,time1=0;


	ColorImageFormat colorFormat = ColorImageFormat::ColorImageFormat_Bgra;

	// 表示部分
	std::vector<BYTE> colorBuffer;

	CComPtr<IBodyFrameReader> bodyFrameReader = nullptr;
	IBody* bodies[6];

public:

	~KinectApp()
	{
		// Kinectの動作を終了する
		if (kinect != nullptr) {
			kinect->Close();
		}
	}

	// 初期化
	void initialize()
	{
		// デフォルトのKinectを取得する
		ERROR_CHECK(::GetDefaultKinectSensor(&kinect));
		ERROR_CHECK(kinect->Open());
		kinect->get_CoordinateMapper(&coordinateMapper);

		// カラーリーダーを取得する
		CComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

		// デフォルトのカラー画像のサイズを取得する
		CComPtr<IFrameDescription> defaultColorFrameDescription;
		ERROR_CHECK(colorFrameSource->get_FrameDescription(&defaultColorFrameDescription));
		ERROR_CHECK(defaultColorFrameDescription->get_Width(&colorWidth));
		ERROR_CHECK(defaultColorFrameDescription->get_Height(&colorHeight));
		ERROR_CHECK(defaultColorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
		std::cout << "default : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

		// カラー画像のサイズを取得する
		CComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK(colorFrameSource->CreateFrameDescription(
			colorFormat, &colorFrameDescription));
		ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth));
		ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight));
		ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel));
		std::cout << "create  : " << colorWidth << ", " << colorHeight << ", " << colorBytesPerPixel << std::endl;

		// バッファーを作成する
		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);

		// ボディリーダーを取得する
		CComPtr<IBodyFrameSource> bodyFrameSource;
		ERROR_CHECK(kinect->get_BodyFrameSource(&bodyFrameSource));
		ERROR_CHECK(bodyFrameSource->OpenReader(&bodyFrameReader));

		for (auto& body : bodies) {
			body = nullptr;
		}
		unsigned int now = (unsigned int)time(0);
		srand(now);


		for (int i = 0; i < 5; i++)
		{
			
			touchCircle[i] = true;
			color[i] = (rand() % 3) +1;
			
			tag[i] = 0;
			//std::cout <<"\n"<< color[i];
			
		}
		sprintf(time_c, "Points = %2.0f", counter);
		targetColor = (rand() % 3) + 1;
		
	}

	void run()
	{
		while (1) {
			update();
			
			draw();
		
			auto key = cv::waitKey(10);
			if (key == 'q'||time2==120) {
				break;
			}

		}

	}

private:

	// データの更新処理
	void update()
	{
		
		updateColorFrame();
		updateBodyFrame();
	}

	// カラーフレームの更新
	void updateColorFrame()
	{
		// フレームを取得する
		CComPtr<IColorFrame> colorFrame;
		auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if (FAILED(ret)) {
			return;
		}

		// 指定の形式でデータを取得する
		ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(
			colorBuffer.size(), &colorBuffer[0], colorFormat));
	}

	void updateBodyFrame()
	{
		// フレームを取得する
		CComPtr<IBodyFrame> bodyFrame;
		auto ret = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
		if (FAILED(ret)) {
			return;
		}

		
		// 前回のBodyを解放する
		for (auto& body : bodies) {
			if (body != nullptr) {
				body->Release();
				body = nullptr;
			}
		}
		
		// データを取得する
		ERROR_CHECK(bodyFrame->GetAndRefreshBodyData(6, &bodies[0]));
	}
	// データの表示処理
	void draw()
	{
		drawColorFrame();
		//drawBodyFrame();

	}

	// カラーデータの表示処理
	void drawColorFrame()
	{
#if 0
		// カラーデータを表示する
		cv::Mat colorImage(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
		cv::imshow("Color Image", colorImage);
#else
		cv::Mat colorImage(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);

		sprintf(time_c, "Time = %d", time2);
		cv::putText(colorImage, time_c, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 10);
		
		
		time1++;
		if ( ((time1 )%60) == 0)
		{
			time2++;

		}
		if (time2 == 119)
		{
			cv::putText(colorImage, "GAME OVER", cv::Point(50, 500), cv::FONT_HERSHEY_SIMPLEX, 10.0, (0, 0, 0), 30);
		}

		if (!wait) {
			
			//X = rand() % 1920 / 2 + 480;
			//Y = rand() % 1040 / 2 + 260;
			
		

			//wait = true;
		
		}
	

		
			
			k = 0;
			
			if (wait)
			{
				if (counter % 5 == 0)
				{
					targetColor = (rand() % 3) + 1;
				}
			
				while (k < 5)
				{
				


					if (touchCircle[k])
					{
						std::cout << "target: " << targetColor << "\n";
						
					



						X[k] = rand() % 1920 / 2 + 480;
						Y[k] = rand() % 1040 / 2 + 260;
						color[k] = (rand() % 3) + 1;

						

						touchCircle[k] = false;

						
					}
				

				
					k++;
				}
				wait = false;

			}
		
			

					if (targetColor == 1)
					{
						R1 = 255;
						G2 = 255;
						B3 = 255;
						

					}
					else if (targetColor == 2)
					{
						R1= 255;
						G2 = 0;
						B3 = 0;
						

					}
					else if (targetColor == 3)
					{

						R1 = 76;
						G2 = 153;
						B3 = 0;
						

					}
					cv::circle(colorImage, cv::Point2d(1600, 200), 100, cv::Scalar(R1, G2, B3), -1);
					

				









					k = 0;
					
					while (k < 5)
					{
						
							if (color[k] == 1)
							{
								R = 255;
								G = 255;
								B = 255;
								cv::circle(colorImage, cv::Point2d(X[k], Y[k]), 50, cv::Scalar(R, G, B), -1);
								tag[k] = color[k];
							}
							else if (color[k] == 2)
							{
								R = 255;
								G = 0;
								B = 0;
								cv::circle(colorImage, cv::Point2d(X[k], Y[k]), 50, cv::Scalar(R, G, B), -1);
								tag[k] = color[k];
							}
							else if (color[k] == 3)
							{

								R = 76;
								G = 153;
								B = 0;
								cv::circle(colorImage, cv::Point2d(X[k], Y[k]), 50, cv::Scalar(R, G, B), -1);
								tag[k] = color[k];
							}

					
				
							
					k++;
						
					}
					

			
			
		
	
		
		
		for (auto body : bodies) {

			if (body == nullptr) {
				continue;
			}

			BOOLEAN isTracked = false;
			ERROR_CHECK(body->get_IsTracked(&isTracked));
			if (!isTracked) {
				continue;
			}

			// 関節の位置を表示する
			Joint joints[JointType::JointType_Count];
			body->GetJoints(JointType::JointType_Count, joints);
			for (auto joint : joints) {
				// 手の位置が追跡状態
				if (joint.TrackingState == TrackingState::TrackingState_Tracked) {
					ColorSpacePoint colorSpacePoint;
					coordinateMapper->MapCameraPointToColorSpace(joint.Position, &colorSpacePoint);
					cv::circle(colorImage, cv::Point2d(colorSpacePoint.X, colorSpacePoint.Y), 5, cv::Scalar(0, 0, 255), 4);
					//drawEllipse(bodyImage, joint, 10, cv::Scalar(255, 0, 0));
				}
				// 手の位置が推測状態
				else if (joint.TrackingState == TrackingState::TrackingState_Inferred) {
					//drawEllipse(bodyImage, joint, 10, cv::Scalar(255, 255, 0));
				}
				if (joint.JointType == JointType::JointType_HandRight) {
					ColorSpacePoint colorSpacePoint;
					coordinateMapper->MapCameraPointToColorSpace(joint.Position, &colorSpacePoint);
					rightHandX = colorSpacePoint.X;
					rightHandY = colorSpacePoint.Y;
				}
				if (joint.JointType == JointType::JointType_HandLeft) {
					ColorSpacePoint colorSpacePoint;
					coordinateMapper->MapCameraPointToColorSpace(joint.Position, &colorSpacePoint);
					leftHandX = colorSpacePoint.X;
					leftHandY = colorSpacePoint.Y;
				}
			}
		}
		k = 0;
		while (k < 5)
		{
			

			if (abs(leftHandX - X[k]) < 50 && abs(leftHandY - Y[k]) < 50) {
				touch = true;
				touchCircle[k] = true;
				
					if (touch&&(targetColor==tag[k]))
					{
						
						std::cout << "tag : " << tag[k];
						counter++;
					}
				

				
				
			}
			if (abs(rightHandX - X[k]) < 50 && abs(rightHandY - Y[k]) < 50) {
				touch = true;
				touchCircle[k] = true;
				
					if (touch && (targetColor == tag[k]))
					{

						std::cout << "tag : " << tag[k];
						counter++;
					}
				


			}
		
			

			if (!wait && touch) {
				wait = true;
				touch = false;
			}
			k++;
		}
	

		sprintf(time_c, "Points = %d", counter);
		
		cv::putText(colorImage, time_c, cv::Point(100, 200), cv::FONT_ITALIC, 2.0, cv::Scalar(0, 0, 0),10);


		cv::putText(colorImage, "Target", cv::Point(1000, 200), cv::FONT_HERSHEY_SIMPLEX, 5.0, cv::Scalar(0, 0, 0),10);

	



		

		cv::Mat harfImage;
		cv::resize(colorImage, harfImage, cv::Size(), 0.5, 0.5);
		cv::imshow("Harf Image", harfImage);
#endif
	}
	void drawBodyFrame()
	{
		// 関節の座標をDepth座標系で表示する
		cv::Mat bodyImage = cv::Mat::zeros(424, 512, CV_8UC4);

		for (auto body : bodies) {
			if (body == nullptr) {
				continue;
			}

			BOOLEAN isTracked = false;
			ERROR_CHECK(body->get_IsTracked(&isTracked));
			if (!isTracked) {
				continue;
			}

			// 関節の位置を表示する
			Joint joints[JointType::JointType_Count];
			body->GetJoints(JointType::JointType_Count, joints);
			for (auto joint : joints) {
				// 手の位置が追跡状態
				if (joint.TrackingState == TrackingState::TrackingState_Tracked) {
					//drawEllipse(bodyImage, joint, 10, cv::Scalar(255, 0, 0));
				}
				// 手の位置が推測状態
				else if (joint.TrackingState == TrackingState::TrackingState_Inferred) {
					//drawEllipse(bodyImage, joint, 10, cv::Scalar(255, 255, 0));
				}
			}
		}

		cv::imshow("Body Image", bodyImage);
	}
	/*void drawEllipse(cv::Mat& bodyImage, const Joint& joint, int r, const cv::Scalar& color)
	{
		// カメラ座標系をDepth座標系に変換する
		CComPtr<ICoordinateMapper> mapper;
		ERROR_CHECK(kinect->get_CoordinateMapper(&mapper));

		DepthSpacePoint point;
		mapper->MapCameraPointToDepthSpace(joint.Position, &point);

		cv::circle(bodyImage, cv::Point(point.X, point.Y), r, color, -1);
	}*/
};

void main()
{
	try {
		KinectApp app;
		app.initialize();
		app.run();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}
