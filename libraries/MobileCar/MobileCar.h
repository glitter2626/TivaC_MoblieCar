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
		
		double obj_rightTicks;
		double obj_leftTicks;
		
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
			
			obj_rightTicks = 0.0;
		    obj_leftTicks = 0.0;
		};

		void execute(double v, double w, float dt, PID *leftPid, PID *rightPid, long leftTicks, long rightTicks){
		
		    if(v == 0.0 && w == 0.0){
		    
		        rightPid->reset();
		        leftPid->reset();
		        
		        rightPwm = 0;
		        leftPwm = 0;
		        
		        return;
		    }
		

			double Vr_obj = (2*v + w*l) / (2.0*radius);
			double Vl_obj = (2*v - w*l) / (2.0*radius);	

			
			double velocityToPwm = 255.0 / MAX_VELOCITY;

			//rightPwm = (int)(Vr_obj*radius*velocityToPwm);  
			//leftPwm = (int)(Vl_obj*radius*velocityToPwm);
			
			obj_rightTicks = TPR / ((2.0 * radius * pi) / (Vr_obj * radius * dt));
			obj_leftTicks = TPR / ((2.0 * radius * pi) / (Vl_obj * radius * dt));
			
		    rightPwm = int(rightPwm + rightPid->execute(obj_rightTicks, rightTicks, dt));  
			leftPwm = int(leftPwm + leftPid->execute(obj_leftTicks, leftTicks, dt));

			rightPwm = rightPwm > 255? 255 : rightPwm;
			rightPwm = rightPwm < -255? -255 : rightPwm;
			
			leftPwm = leftPwm > 255? 255 : leftPwm;
			leftPwm = leftPwm < -255? -255 : leftPwm;
		};

		void updateOdometry(long leftTicks, long rightTicks, double dt){

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
		
		double getObjRightTicks(){
		    return obj_rightTicks;
		};
		
		double getObjLeftTicks(){
		    return obj_leftTicks;
		};
};

#endif
