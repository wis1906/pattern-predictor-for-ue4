#pragma once

class Kalman
{
public:
	//ID
	char type = 0;

	//Velocity
	double velocityAlpha = 0.4f;
	double prevX = 0;
	double prevVelocityX = 0;
	double velocityX = 0;

	//Kalman
	double	X = 0;  // ���� ���
	double	Q = 0.001f;  // ���� ������ ���л� ���
	double	P = 1;  // ���� ���л� ���
	double	R = 0.1f;  // ���� ���л� ���

public:
	void initKalman(double _X, double _Q, double _P, double _R, double _velocityAlpha);
	double KalmanPredictUpdate(double NewData);
	void setVelocity(double NewData);
};
