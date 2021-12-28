// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "featurePredictor.h"
#include "Engine.h"
#include "GameFramework/Actor.h"
#include "Runtime/Engine/Classes/Engine/Texture2D.h"

#include "opencv2/opencv.hpp"
#include "Kalman.h"

#include <iostream>
#include <fstream>
#include <string>
#include <io.h>

#include "panoramaReader.generated.h"

using namespace cv;
using namespace std;

UENUM()
enum class DetectionMode : uint8
{
	None UMETA(DisplayName = "None"),
	FeatureDetection UMETA(DisplayName = "FeatureDetection"),
	FeatureWithKalman UMETA(DisplayName = "FeatureWithKalman"),
};

UENUM()
enum class PlayMode : uint8
{
	Image UMETA(DisplayName = "Image"),
	Video UMETA(DisplayName = "Video"),
};

UCLASS()
class FEATUREPREDICTOR_API ApanoramaReader : public AActor
{
	GENERATED_BODY()

public:
	ApanoramaReader();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

	//Editor Adjustable Field
public:
	UPROPERTY(EditAnywhere, Category = Cubemap)
		int panoramaWidth = 3000;
	UPROPERTY(EditAnywhere, Category = Cubemap)
		int panoramaHeight = 1500;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = Cubemap)
		float viewRatio = 1.0f;

	UPROPERTY(EditAnywhere, Category = Cubemap)
		FString filePath = "D:/Unreal_Project/4.19.2/featurePredictor/Content/Images/";
	UPROPERTY(EditAnywhere, Category = Cubemap)
		FString imagePath_Obj = "Pattern2_Low.jpg";
	UPROPERTY(EditAnywhere, Category = Cubemap)
		FString imagePath_Scene = "PatternVideo.mp4";

	UPROPERTY(EditAnywhere, Category = Cubemap)
		DetectionMode detectionMode;
	UPROPERTY(EditAnywhere, Category = Cubemap)
		PlayMode playMode;

	UPROPERTY(EditAnywhere, Category = KalmanValue)
		float kalman_CenterQ = 0.01f;
	UPROPERTY(EditAnywhere, Category = KalmanValue)
		float kalman_CenterR = 0.01f;
	UPROPERTY(EditAnywhere, Category = KalmanValue)
		float kalman_CenterVelocityAlpha = 0.6f;
	UPROPERTY(EditAnywhere, Category = KalmanValue)
		float kalman_CornerQ = 0.001f;
	UPROPERTY(EditAnywhere, Category = KalmanValue)
		float kalman_CornerR = 0.01f;
	UPROPERTY(EditAnywhere, Category = KalmanValue)
		float kalman_CornerVelocityAlpha = 0.6f;
	UPROPERTY(EditAnywhere, Category = KalmanValue)
		int newDetectCntNum = 3;


public:
	//flag
	bool isOpened = false;
	bool isFristDetect = false;


	//Image Database
	class imageDatabase* imageData;

	//Feature detector
	class FDetector* detector;

	//Kalman Fillter
	pair<class Kalman*, class Kalman*> kalmanCenter;
	pair<class Kalman*, class Kalman*> kalmanCorner[4];
	bool kalmanInit = false;

	//Check Field for New Detection
	int notDetectedCnt = 0;
	char newDetectType[2] = { 0 , 0 };
	int newDetectCnt = 0;
	int newDetectWeight = 0;

	//Texture Data
	//------------------------------
	// The current texture array
	TArray<FColor> Data;
	// Pointer to update texture region 2D struct
	FUpdateTextureRegion2D* VideoUpdateTextureRegion;
	// Image textures
	UPROPERTY(BlueprintReadOnly, Category = Cubemap)
		UTexture2D* TopTexture;
	UPROPERTY(BlueprintReadOnly, Category = Cubemap)
		UTexture2D* BottomTexture;
	UPROPERTY(BlueprintReadOnly, Category = Cubemap)
		UTexture2D* RearTexture;
	UPROPERTY(BlueprintReadOnly, Category = Cubemap)
		UTexture2D* LeftTexture;
	UPROPERTY(BlueprintReadOnly, Category = Cubemap)
		UTexture2D* FrontTexture;
	UPROPERTY(BlueprintReadOnly, Category = Cubemap)
		UTexture2D* RightTexture;

public:
	//For Init
	void initImageDatabase(imageDatabase* imageData, int width, int height);
	void initTexture(Size size);

	//For Processing
	void doProcessing(imageDatabase* imgData);

	//For feature detection
	void findMostMatchedType(imageDatabase* imgData);
	int findMatchedType(char type, imageDatabase* imgData);
	void detectFeatureByMatchedType(char c, imageDatabase* imgData);
	void detectFeature(char type, Mat *Img_Obj, Mat *Img_Scene);
	void drawLineDetectedImg(Mat *Img_Scene);

	//For Kalman Filter
	void updateKalman(vector<cv::Point2f> scene_corners);
	void addExpectedDataToKalman();
	void drawKalman(Mat* img);

	//Check Field for New Detection
	void checkWeightTwoField();
	void checkField(char type);
	void addNewDetectFieldType(char type, int first, int second);

	//For panorama to cubemap
	void createCubemap(imageDatabase* imgData);
	void cubeToImg(imageDatabase* imgData, Mat* Img, char Type);
	Point3d getPanoramaAxis(int x, int y, char Type, int CubeWidth, int Width, int Height);
	Point3d getThetaPhi(double x, double y, double z);

	//For File IO
	bool loadFile(imageDatabase* imgData);

	//For Texture Update
	void updateTextureAll(imageDatabase* imgdata);
	void updateTextureAtOneTime(Size size, Mat* image, UTexture2D* texture);

	//For blueprint functions
	UFUNCTION(BlueprintImplementableEvent, Category = Cubemap)
		void OnNextVideoFrame();
};

class imageDatabase
{

public:
	VideoCapture VCap;

	Size size;

	Mat objImg;
	Mat sourceImg;
	Mat panoramaImg;

	Mat TopImg;
	Mat BottomImg;
	Mat RearImg;
	Mat LeftImg;
	Mat FrontImg;
	Mat RightImg;
};

