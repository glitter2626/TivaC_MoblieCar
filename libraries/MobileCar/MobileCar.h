#ifndef _MOBILECAR_H_
#define _MOBILECAR_H_

#include<Encoder.h>
#include<PID.h>

class MobileCar{

	private:
		double x;
		double y;
		double theta;

		double Vr_fbk;
		double Vl_fbk;

		int rightPwm;
		int leftPwm;
		
		const float pi = 3.1415;
		const float radius = 19.0;
		const float l = 11.0;

		const float MAX_VELOCITY = 5.0;

	public:
		
		MobileCar(double x, double y, double theta){

			this->x = x;
			this->y = y;
			this->theta = theta;

			Vr_fbk = 0;
			Vl_fbk = 0;

			rightPwm = 0;
			leftPwm = 0;
		};

		void execute(double v, double w, long leftTicks, long rightTicks, float dt, PID *leftPid, PID *rightPid){

			double Vr_obj = (2*v + w*l) / (2*radius);
			double Vl_obj = (2*v - w*l) / (2*radius);

			double Dr = 2*pi*radius*rightTicks/TPR;
			double Dl = 2*pi*radius*leftTicks/TPR;
			double Dc = (Dr + Dl) / (double)2;

			x = x + Dc * cos(theta);
			y = y + Dc * sin(theta);
			theta = theta + (Dr - Dl) / l;

			Vr_fbk = Dr / dt;
			Vl_fbk = Dl / dt;

			double velocityToPwm = 255 / MAX_VELOCITY;

			rightPwm = (int)rightPid->execute(Vr_obj*velocityToPwm, Vr_fbk*velocityToPwm);  // radius*Vr_obj*velocityToPwm
			leftPwm = (int)leftPid->execute(Vl_obj*velocityToPwm, Vl_fbk*velocityToPwm);

			
		};

		double getX(){
			return x;
		};

		double getY(){
			return y;
		};

		double getVr(){
			return Vr_fbk;
		};

		double getVl(){
			return Vl_fbk;
		};

		double getTheta(){
			return theta;
		};

		int getRightPWM(){
			return rightPwm;
		};

		int getLeftPWM(){
			return leftPwm;
		};
};

#endif
