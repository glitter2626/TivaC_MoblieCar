#ifndef _KALMAN_H_
#define _KALMAN_H_

class Kalman{

	private:

		double x;
		double p;
		double Kg;
        
	    double R;
		double Q;

		Kalman();

	public:
		
		Kalman(double init_x, double estimated_error, double measure_noise, double process_noise){
			
			x = init_x;
			p = estimated_error;
			R = measure_noise;
			Q = process_noise;

			Kg = 0;
		};

		double execute(double sensor_value){
			
			x = x;
			p = p + Q;

			Kg = p / (p + R);
			x = x + Kg * (sensor_value - x);
			p = (1 - Kg) * p;

			return x;
		};
};


#endif
