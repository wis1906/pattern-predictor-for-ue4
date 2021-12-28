// Fill out your copyright notice in the Description page of Project Settings.

#include "panoramaReader.h"
#include "featurePredictor.h"
#include "FDetector.h"

// Sets default values
ApanoramaReader::ApanoramaReader()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	//Init classes
	imageData = new imageDatabase();
	detector = new FDetector();

	//Init Kalman class

	kalmanCenter.first = new Kalman();
	kalmanCenter.second = new Kalman();
	for (int i = 0; i < 4; i++)
	{
		kalmanCorner[i].first = new Kalman();
		kalmanCorner[i].second = new Kalman();
	}
}

// Called when the game starts or when spawned
void ApanoramaReader::BeginPlay()
{
	Super::BeginPlay();

	//Initialize
	initImageDatabase(imageData, panoramaWidth, panoramaHeight);
	initTexture(imageData->size);

	//Load File
	if (!loadFile(imageData))
	{
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("Image not found."));
	}

	if (detectionMode != DetectionMode::None)
	{
		detector->getObjFeaturePoint(&imageData->objImg);
	}
	//Processing for image
	if (isOpened && playMode == PlayMode::Image)
	{
		doProcessing(imageData);
	}

}

// Called every frame
void ApanoramaReader::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);


	//Process video frame per tick
	if (isOpened && playMode == PlayMode::Video)
	{
		if (!imageData->VCap.read(imageData->sourceImg))
		{	//Restart Video
			FString pathString = filePath + imagePath_Scene;
			imageData->VCap = VideoCapture(TCHAR_TO_UTF8(*pathString));
			imageData->VCap.set(CAP_PROP_FRAME_WIDTH, panoramaWidth);
			imageData->VCap.set(CAP_PROP_FRAME_HEIGHT, panoramaHeight);
			imageData->VCap.read(imageData->sourceImg);
		}

		doProcessing(imageData);

	}
}

//###############################
// Processing
//###############################
void ApanoramaReader::doProcessing(imageDatabase* imgData)
{
	//Get panorama image
	resize(imgData->sourceImg, imgData->panoramaImg, Size(panoramaWidth, panoramaHeight));

	//Create Cubemap
	createCubemap(imageData);


	//Feature detection
	if (detectionMode != DetectionMode::None)
	{
		if (detectionMode == DetectionMode::FeatureWithKalman)
		{
			if (notDetectedCnt > 10)
			{
				notDetectedCnt = 0;
				isFristDetect = false;
			}

			if (!isFristDetect)
			{
				findMostMatchedType(imgData);
				UE_LOG(featurePredictor, Warning, TEXT("%c Field is most matched"), detector->matchType);
			}
			else
			{
				checkField(detector->matchType);
			}


			if (newDetectCnt != 0)
			{
				checkWeightTwoField();

				if (newDetectCnt == 0)
				{
					if (newDetectWeight > newDetectCntNum / 2)
					{
						detector->matchType = newDetectType[1];

						newDetectType[0] = 0;
						newDetectType[1] = 0;
						newDetectCnt = 0;
						newDetectWeight = 0;
						kalmanInit = true;

						UE_LOG(featurePredictor, Warning, TEXT("Field change : %c"), detector->matchType);
					}
					else if (newDetectWeight == newDetectCntNum / 2)
					{
						newDetectCnt++;
					}
					else
					{
						newDetectType[0] = 0;
						newDetectType[1] = 0;
						newDetectCnt = 0;
						newDetectWeight = 0;

						UE_LOG(featurePredictor, Warning, TEXT("Field change : Not"));
					}
				}


			}
		}
		else
		{
			findMostMatchedType(imgData);
			UE_LOG(featurePredictor, Warning, TEXT("%c Field is most matched"), detector->matchType);
		}


		detectFeatureByMatchedType(detector->matchType, imgData);
	}

	//Update Texture
	updateTextureAll(imageData);

	//Update VideoFrame(For Blueprint)
	OnNextVideoFrame();
}

//###############################
// Feature Detection
//###############################
void ApanoramaReader::findMostMatchedType(imageDatabase* imgData)
{
	detector->matchType = 0;

	char type = 0;
	double max = 0;
	double curCnt = 0;


	curCnt = findMatchedType('T', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'T';
	}

	curCnt = findMatchedType('B', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'B';
	}

	curCnt = findMatchedType('F', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'F';
	}

	curCnt = findMatchedType('A', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'A';
	}

	curCnt = findMatchedType('L', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'L';
	}

	curCnt = findMatchedType('R', imgData);
	if (max < curCnt)
	{
		max = curCnt;
		type = 'R';
	}

	detector->matchType = type;
}
int ApanoramaReader::findMatchedType(char type, imageDatabase* imgData)
{
	Mat *Img_Scene = 0;
	switch (type)
	{
	case 'T':
		Img_Scene = &imgData->TopImg;
		break;
	case 'B':
		Img_Scene = &imgData->BottomImg;
		break;
	case 'A':
		Img_Scene = &imgData->RearImg;
		break;
	case 'L':
		Img_Scene = &imgData->LeftImg;
		break;
	case 'F':
		Img_Scene = &imgData->FrontImg;
		break;
	case 'R':
		Img_Scene = &imgData->RightImg;
		break;
	}

	drawLineDetectedImg(Img_Scene);

	IpPairVec matches = detector->matchImage(Img_Scene);
	//UE_LOG(featurePredictor, Warning, TEXT("%c  %d\n"), type, matches.size());

	return detector->matchWeight;
}
void ApanoramaReader::detectFeatureByMatchedType(char c, imageDatabase* imgData)
{
	switch (c)
	{
	case 'T':
		detectFeature('T', &imgData->objImg, &imgData->TopImg);
		break;
	case 'B':
		detectFeature('B', &imgData->objImg, &imgData->BottomImg);
		break;
	case 'A':
		detectFeature('A', &imgData->objImg, &imgData->RearImg);
		break;
	case 'L':
		detectFeature('L', &imgData->objImg, &imgData->LeftImg);
		break;
	case 'F':
		detectFeature('F', &imgData->objImg, &imgData->FrontImg);
		break;
	case 'R':
		detectFeature('R', &imgData->objImg, &imgData->RightImg);
		break;
	}
}
void ApanoramaReader::detectFeature(char type, Mat *Img_Obj, Mat *Img_Scene)
{
	vector<cv::Point2f> scene_corners(4);

	drawLineDetectedImg(Img_Scene);

	//Detect Features of each image and Match images
	IpPairVec matches = detector->matchImage(Img_Scene);

	if (matches.size() == 0)
		return;

	//Get and draw corners of outline
	scene_corners = detector->drawMatchedLine(Img_Obj, Img_Scene, matches);



	//Some is detected
	if (scene_corners[0].x != -999 && scene_corners[0].y != -999)
	{
		//Calculate Center Axis
		Point2f centerAxis;
		centerAxis.x = ((scene_corners[0].x + scene_corners[1].x + scene_corners[2].x + scene_corners[3].x) / 4);
		centerAxis.y = ((scene_corners[0].y + scene_corners[1].y + scene_corners[2].y + scene_corners[3].y) / 4);
		//Real Axis Center
		circle(*Img_Scene, Point((int)centerAxis.x, (int)centerAxis.y), 30, Scalar(0, 0, 255), 2);

		if (detectionMode == DetectionMode::FeatureWithKalman)
		{
			//Kalman
			updateKalman(scene_corners);
			drawKalman(Img_Scene);
		}

		notDetectedCnt = 0;
	}
	//Not detected
	else
	{
		if (detectionMode == DetectionMode::FeatureWithKalman)
		{
			if (isFristDetect && !kalmanInit)
			{
				//Repeat Kalman data
				addExpectedDataToKalman();
				drawKalman(Img_Scene);
			}
		}
		notDetectedCnt++;
	}
}
void ApanoramaReader::drawLineDetectedImg(Mat *img)
{
	line(*img, Point(0, 0), Point(img->cols, 0), cv::Scalar(0, 255, 0), 5);
	line(*img, Point(img->cols, 0), Point(img->cols, img->rows), cv::Scalar(0, 255, 0), 5);
	line(*img, Point(img->cols, img->rows), Point(0, img->rows), cv::Scalar(0, 255, 0), 5);
	line(*img, Point(0, img->rows), Point(0, 0), cv::Scalar(0, 255, 0), 5);
}

//###############################
// Kalman Fillter
//###############################
void ApanoramaReader::updateKalman(vector<cv::Point2f> scene_corners)
{
	//Calculate Center Axis
	Point2f centerAxis;
	centerAxis.x = ((scene_corners[0].x + scene_corners[1].x + scene_corners[2].x + scene_corners[3].x) / 4);
	centerAxis.y = ((scene_corners[0].y + scene_corners[1].y + scene_corners[2].y + scene_corners[3].y) / 4);

	if (!isFristDetect || kalmanInit)
	{
		UE_LOG(featurePredictor, Warning, TEXT("Initialize Kalman Data"));
		isFristDetect = true;
		kalmanInit = false;

		//init center
		kalmanCenter.first->initKalman((double)centerAxis.x, kalman_CenterQ, 1, kalman_CenterR, kalman_CenterVelocityAlpha);
		kalmanCenter.second->initKalman((double)centerAxis.y, kalman_CenterQ, 1, kalman_CenterR, kalman_CenterVelocityAlpha);

		//init corners
		for (int i = 0; i < 4; i++)
		{
			kalmanCorner[i].first->initKalman((double)scene_corners[i].x - (double)centerAxis.x, kalman_CornerQ, 1, kalman_CornerR, kalman_CornerVelocityAlpha);
			kalmanCorner[i].second->initKalman((double)scene_corners[i].y - (double)centerAxis.y, kalman_CornerQ, 1, kalman_CornerR, kalman_CornerVelocityAlpha);
		}
	}
	else
	{
		//Update center
		kalmanCenter.first->KalmanPredictUpdate((double)centerAxis.x);
		kalmanCenter.first->setVelocity(kalmanCenter.first->X);
		kalmanCenter.second->KalmanPredictUpdate((double)centerAxis.y);
		kalmanCenter.second->setVelocity(kalmanCenter.second->X);

		//Update corners
		for (int i = 0; i < 4; i++)
		{
			kalmanCorner[i].first->KalmanPredictUpdate((double)scene_corners[i].x - (double)centerAxis.x);
			kalmanCorner[i].first->setVelocity(kalmanCorner[i].first->X);
			kalmanCorner[i].second->KalmanPredictUpdate((double)scene_corners[i].y - (double)centerAxis.y);
			kalmanCorner[i].second->setVelocity(kalmanCorner[i].second->X);
		}
	}
}
void ApanoramaReader::addExpectedDataToKalman()
{
	//Update center
	kalmanCenter.first->KalmanPredictUpdate(kalmanCenter.first->X + kalmanCenter.first->velocityX);
	kalmanCenter.second->KalmanPredictUpdate(kalmanCenter.second->X + kalmanCenter.second->velocityX);

	//Update corners
	for (int i = 0; i < 4; i++)
	{
		kalmanCorner[i].first->KalmanPredictUpdate(kalmanCorner[i].first->X + kalmanCenter.first->velocityX);
		kalmanCorner[i].second->KalmanPredictUpdate(kalmanCorner[i].second->X + kalmanCenter.second->velocityX);
	}
}
void ApanoramaReader::drawKalman(Mat* img)
{
	//Kalman Center Point
	circle(*img, Point(kalmanCenter.first->X, kalmanCenter.second->X), 20, Scalar(255, 255, 255), 2);

	//Expected Next Center Point
	circle(*img, Point(kalmanCenter.first->X + kalmanCenter.first->velocityX, kalmanCenter.second->X + kalmanCenter.second->velocityX), 10, Scalar(255, 0, 0), -1);

	//Draw Kalman Line
	for (int i = 0; i < 4; i++)
	{
		line(*img,
			Point(kalmanCorner[i % 4].first->X + kalmanCenter.first->X, kalmanCorner[i % 4].second->X + kalmanCenter.second->X),
			Point(kalmanCorner[(i + 1) % 4].first->X + kalmanCenter.first->X, kalmanCorner[(i + 1) % 4].second->X + kalmanCenter.second->X),
			cv::Scalar(255, 255, 255),
			2);
	}
}

//###############################
// Check Field for New Detection
//###############################
void ApanoramaReader::checkWeightTwoField()
{
	char type = 0;
	int max = 0;
	int curCnt = 0;

	curCnt = findMatchedType(newDetectType[0], imageData);
	if (max < curCnt)
	{
		max = curCnt;
		type = newDetectType[0];
	}

	curCnt = findMatchedType(newDetectType[1], imageData);
	if (max < curCnt)
	{
		max = curCnt;
		type = newDetectType[1];
	}

	if (type != 0 && type != newDetectType[0])
	{
		newDetectWeight++;
	}
	newDetectCnt--;

	//UE_LOG(featurePredictor, Warning, TEXT("Count : %d, Weight : %d"), newDetectCnt, newDetectWeight);
}
void ApanoramaReader::checkField(char type)
{

	if (kalmanCenter.first->velocityX < 0)
	{
		int expectedPos = kalmanCenter.first->X + kalmanCenter.first->velocityX * 5;

		if (expectedPos < 0)
		{
			addNewDetectFieldType(type, 0, 0);
			return;
		}
	}
	else if (kalmanCenter.first->velocityX >= 0)
	{
		int expectedPos = kalmanCenter.first->X + kalmanCenter.first->velocityX * 5;

		if (expectedPos > imageData->TopImg.cols)
		{
			addNewDetectFieldType(type, 0, 1);
			return;
		}
	}
	if (kalmanCenter.second->velocityX < 0)
	{
		int expectedPos = kalmanCenter.second->X + kalmanCenter.second->velocityX * 5;

		if (expectedPos < 0)
		{
			addNewDetectFieldType(type, 1, 0);
			return;
		}
	}
	else if (kalmanCenter.second->velocityX >= 0)
	{
		int expectedPos = kalmanCenter.second->X + kalmanCenter.second->velocityX * 5;

		if (expectedPos > imageData->TopImg.rows)
		{
			addNewDetectFieldType(type, 1, 1);
			return;
		}
	}
}
void ApanoramaReader::addNewDetectFieldType(char type, int first, int second)
{
	char checkType[2] = { type , 0 };

	char fieldT[2][2] = { {'L','R' },{'A', 'F'} };
	char fieldB[2][2] = { {'L','R' },{'F', 'A'} };
	char fieldF[2][2] = { {'L','R' },{'T', 'B'} };
	char fieldA[2][2] = { {'R','L' },{'T', 'B'} };
	char fieldL[2][2] = { {'A','F' },{'T', 'B'} };
	char fieldR[2][2] = { {'F','A' },{'T', 'B'} };

	switch (type)
	{
	case 'T':
		checkType[1] = fieldT[first][second];
		break;
	case 'B':
		checkType[1] = fieldB[first][second];
		break;
	case 'A':
		checkType[1] = fieldA[first][second];
		break;
	case 'L':
		checkType[1] = fieldL[first][second];
		break;
	case 'F':
		checkType[1] = fieldF[first][second];
		break;
	case 'R':
		checkType[1] = fieldR[first][second];
		break;
	}

	if (newDetectType[0] == checkType[0] && newDetectType[1] == checkType[1])
	{
		return;
	}


	UE_LOG(featurePredictor, Warning, TEXT("Check : [ %c , %c ] Field"), checkType[0], checkType[1]);
	newDetectType[0] = checkType[0];
	newDetectType[1] = checkType[1];
	newDetectCnt = newDetectCntNum;
	newDetectWeight = 0;
}

//###############################
// Panorama to Cubemap
//###############################
void ApanoramaReader::createCubemap(imageDatabase* imgData)
{
	cubeToImg(imgData, &imgData->RearImg, 'A');
	cubeToImg(imgData, &imgData->LeftImg, 'L');
	cubeToImg(imgData, &imgData->FrontImg, 'F');
	cubeToImg(imgData, &imgData->RightImg, 'R');
	cubeToImg(imgData, &imgData->TopImg, 'T');
	cubeToImg(imgData, &imgData->BottomImg, 'B');
}
void ApanoramaReader::cubeToImg(imageDatabase* imgData, Mat* Img, char Type)
{
	int width = Img->cols;
	int height = Img->rows;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			Point3d xy = getPanoramaAxis(j, i, Type, width, imgData->panoramaImg.cols, imgData->panoramaImg.rows);


			int idx = j + (i * width);
			int xy_idx = (int)xy.x + ((int)xy.y * panoramaWidth);

			Img->data[idx * 3 + 0] = imgData->panoramaImg.data[xy_idx * 3 + 0];
			Img->data[idx * 3 + 1] = imgData->panoramaImg.data[xy_idx * 3 + 1];
			Img->data[idx * 3 + 2] = imgData->panoramaImg.data[xy_idx * 3 + 2];
		}
	}
}
Point3d ApanoramaReader::getPanoramaAxis(int x, int y, char Type, int CubeWidth, int Width, int Height)
{
	double u = 2 * (double(x) / CubeWidth - 0.5f);
	double v = 2 * (double(y) / CubeWidth - 0.5f);

	Point3d ThetaPhi;
	if (Type == 'F')
	{
		ThetaPhi = getThetaPhi(1, u, -v);
	}
	else if (Type == 'R')
	{
		ThetaPhi = getThetaPhi(-u, 1, -v);
	}
	else if (Type == 'L')
	{
		ThetaPhi = getThetaPhi(u, -1, -v);
	}
	else if (Type == 'A')
	{
		ThetaPhi = getThetaPhi(-1, -u, -v);
	}
	else if (Type == 'B')
	{
		ThetaPhi = getThetaPhi(-v, u, -1);
	}
	else if (Type == 'T')
	{
		ThetaPhi = getThetaPhi(v, u, 1);
	}

	double _x;
	double _y;
	_x = 0.5 + 0.5*(ThetaPhi.x / PI);
	_y = (ThetaPhi.y / PI);

	Point3d xy;
	xy.x = _x * Width;
	xy.y = _y * Height;
	if (xy.x >= Width)
	{
		xy.x = Width - 1;
	}
	if (xy.y >= Height)
	{
		xy.y = Height - 1;
	}
	return xy;
}
Point3d ApanoramaReader::getThetaPhi(double x, double y, double z)
{
	double dv = sqrt(x * x + y * y + z * z);
	x = x / dv;
	y = y / dv;
	z = z / dv;

	Point3d ThetaPhi;
	ThetaPhi.x = atan2(y, x);
	ThetaPhi.y = acos(z);
	return ThetaPhi;
}

//###############################
// File IO
//###############################
bool ApanoramaReader::loadFile(imageDatabase* imgData)
{
	FString pathString;

	isOpened = true;

	//Load object image
	pathString = filePath + imagePath_Obj;
	imgData->objImg = imread(TCHAR_TO_UTF8(*pathString));
	if (imgData->objImg.empty())
	{
		isOpened = false;
	}

	if (playMode == PlayMode::Video)
	{
		pathString = filePath + imagePath_Scene;
		imgData->VCap = VideoCapture(TCHAR_TO_UTF8(*pathString));
		if (!imgData->VCap.isOpened())
		{
			isOpened = false;
		}
		else
		{
			//Adjust Width and Height of Video
			imgData->VCap.set(CAP_PROP_FRAME_WIDTH, panoramaWidth);
			imgData->VCap.set(CAP_PROP_FRAME_HEIGHT, panoramaHeight);
		}
	}
	else
	{
		//Load scene image
		pathString = filePath + imagePath_Scene;
		imgData->sourceImg = imread(TCHAR_TO_UTF8(*pathString));
		if (imgData->sourceImg.empty())
		{
			isOpened = false;
		}
	}

	return isOpened;
}

//###############################
// Initialize
//###############################
void ApanoramaReader::initImageDatabase(imageDatabase* imgData, int width, int height)
{
	Size size = Size((int)width / 4, (int)height / 2);
	imgData->size = size;

	//Init Image
	imgData->TopImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
	imgData->BottomImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
	imgData->RearImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
	imgData->LeftImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
	imgData->FrontImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
	imgData->RightImg = Mat(Size(size), CV_8UC3, Scalar(0, 0, 0));
}
void ApanoramaReader::initTexture(Size size)
{
	//Init Texture
	TopTexture = UTexture2D::CreateTransient(size.width, size.height);
	TopTexture->UpdateResource();
	BottomTexture = UTexture2D::CreateTransient(size.width, size.height);
	BottomTexture->UpdateResource();
	RearTexture = UTexture2D::CreateTransient(size.width, size.height);
	RearTexture->UpdateResource();
	LeftTexture = UTexture2D::CreateTransient(size.width, size.height);
	LeftTexture->UpdateResource();
	FrontTexture = UTexture2D::CreateTransient(size.width, size.height);
	FrontTexture->UpdateResource();
	RightTexture = UTexture2D::CreateTransient(size.width, size.height);
	RightTexture->UpdateResource();
}

//###############################
// Texture Process
//###############################
void ApanoramaReader::updateTextureAll(imageDatabase* imgdata)
{
	//Update Texture
	updateTextureAtOneTime(imgdata->size, &imgdata->TopImg, TopTexture);
	updateTextureAtOneTime(imgdata->size, &imgdata->BottomImg, BottomTexture);
	updateTextureAtOneTime(imgdata->size, &imgdata->RearImg, RearTexture);
	updateTextureAtOneTime(imgdata->size, &imgdata->LeftImg, LeftTexture);
	updateTextureAtOneTime(imgdata->size, &imgdata->FrontImg, FrontTexture);
	updateTextureAtOneTime(imgdata->size, &imgdata->RightImg, RightTexture);
}
void ApanoramaReader::updateTextureAtOneTime(Size size, Mat* image, UTexture2D* texture)
{
	Data.Init(FColor(0, 0, 0, 255), size.width * size.height);
	if (image->data)
	{
		// Copy Mat data to Data array
		for (int y = 0; y < size.height; y++)
		{
			for (int x = 0; x < size.width; x++)
			{
				int i = x + (y * size.height);
				Data[i].B = image->data[i * 3 + 0];
				Data[i].G = image->data[i * 3 + 1];
				Data[i].R = image->data[i * 3 + 2];
			}
		}

		FTexture2DMipMap& Mip = texture->PlatformData->Mips[0];
		void* pData = Mip.BulkData.Lock(LOCK_READ_WRITE);
		FMemory::Memcpy(pData, Data.GetData(), Data.Num() * 4);
		Mip.BulkData.Unlock();
		texture->UpdateResource();
	}
}