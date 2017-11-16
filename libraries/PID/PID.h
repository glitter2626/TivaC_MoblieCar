#ifndef _PID_H_
#define _PID_H_

class PID{
	
	private:

		double e;	// input value
		double old_e;	// previous e

		double e_dot;	// e - old_e
		double E;	// E += e

		double Kp;
		double Kd;
		double Ki;


	public:
		
		PID(double Kp, double Kd, double Ki){
			
			e = 0;
			old_e = 0;

			e_dot = 0;
			E = 0;

			this->Kp = Kp;
			this->Kd = Kd;
			this->Ki = Ki;	
		};

		double execute(double value, double feedback){
		
			e = value - feedback;
			e_dot = e - old_e;
			E = E + e;
			u = Kp * e + Kd * e_dot + Ki * E;
			old_e = e;

			return u;
		};

};

#endif
