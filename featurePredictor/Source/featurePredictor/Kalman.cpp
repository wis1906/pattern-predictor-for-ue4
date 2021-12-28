// Fill out your copyright notice in the Description page of Project Settings.

#include "Kalman.h"

void Kalman::initKalman(double _X, double _Q, double _P, double _R, double _velocityAlpha)
{
	prevX = _X;
	velocityAlpha = _velocityAlpha;

	X = _X;
	Q = _Q;
	P = _P;
	R = _R;
}


double Kalman::KalmanPredictUpdate(double NewData)
{
	double K;	// Kalman gain

	// Predict
	// Kalman->X �� 1�������� �׳� �������� �����ҰŶ� ������(?)
	P = P + Q;

	// Update
	K = P / (P + R);
	X = X + K * (NewData - X);
	P = (1 - K) * P;

	//setVelocity(X);

	return X;
}

void Kalman::setVelocity(double NewData)
{
	double curVelocity = NewData - prevX;

	velocityX = velocityAlpha * prevVelocityX + (1 - velocityAlpha) * curVelocity;

	prevX = NewData;
	prevVelocityX = velocityX;
}