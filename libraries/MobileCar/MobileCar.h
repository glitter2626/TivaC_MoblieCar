#ifndef _MOBILECAR_H_
#define _MOBILECAR_H_

#include<Encoder.h>
#include<PID.h>
#include<Motordriver.h>

class MobileCar{

	private:
		double x;
		double y;
		double theta;   // rad

		double Vr_fbk;  // use carefully
		double Vl_fbk;  // use carefully
		
		double Vx;
		double Vy;
		double W;

		int rightPwm;
		int leftPwm;
		
		const float pi = 3.1415;
		const float radius = 0.045;
		const float l = 0.36;

		const float MAX_VELOCITY = 0.35;

	public:
		
		MobileCar(double x, double y, double theta){

			this->x = x;
			this->y = y;
			this->theta = theta;

			Vr_fbk = 0;
			Vl_fbk = 0;
			
			Vx = 0;
			Vy = 0;

			rightPwm = 0;
			leftPwm = 0;
		};

		void execute(double v, double w, float dt, PID *leftPid, PID *rightPid){

			double Vr_obj = (2*v + w*l) / (2.0*radius);
			double Vl_obj = (2*v - w*l) / (2.0*radius);	

			
			double velocityToPwm = 255.0 / MAX_VELOCITY;

			rightPwm = (int)(Vr_obj*radius*velocityToPwm);  
			leftPwm = (int)(Vl_obj*radius*velocityToPwm);

			
		    //rightPwm = (int)rightPid->execute(Vr_obj*radius*velocityToPwm, Vr_fbk*radius*velocityToPwm);  
			//leftPwm = (int)leftPid->execute(Vl_obj*radius*velocityToPwm, Vl_fbk*radius*velocityToPwm);

			
		};

		void updateOdometry(long leftTicks, long rightTicks, float dt){

			double Dr = 2.0*pi*radius*rightTicks/(double)TPR;
			double Dl = 2.0*pi*radius*leftTicks/(double)TPR;
			double Dc = (Dr + Dl) / (double)2;
			
			
			Vr_fbk = Dr / (dt * radius);
			Vl_fbk = Dl / (dt * radius);
			
			Vx = Dc * cos(theta) / dt; //0.5 * radius * (Vr_fbk + Vl_fbk) * cos(theta);
			Vy = Dc * sin(theta) / dt; //0.5 * radius * (Vr_fbk - Vl_fbk) * sin(theta);
			W = (Dr - Dl) / (l * dt);
 
			x = x + Dc * cos(theta);
			y = y + Dc * sin(theta);
			theta = theta + (Dr - Dl) / l;
			theta = atan2(sin(theta), cos(theta));

			
		};

		float getRadius(){
			return radius;
		};


		float getL(){
			return l;
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
		
		double getVx(){
		    return Vx;
		};
		
		double getVy(){
		    return Vy;
	    };
	    
	    double getW(){
	        return W;
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
