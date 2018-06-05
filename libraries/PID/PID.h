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

		double execute(double value, double feedback, double dt){
			
			e = value - feedback;
			e_dot = (e - old_e) / dt;
			E = E + e * dt;
			double u = Kp * e + Kd * e_dot + Ki * E;
			old_e = e;

			return u / 50.0;
		};
		
		void reset(){
		
		    e = 0.0;	// input value
		    old_e = 0.0;	// previous e

		    e_dot = 0.0;	// e - old_e
		    E = 0.0;	// E += e

		};

};

#endif
