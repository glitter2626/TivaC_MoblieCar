#ifndef _ULTRASOUND_H_
#define _ULTRASOUND_H_

class Ultrasonic{

	private:

		int echo;
		int trig;


	public:

		Ultrasonic(int trig, int echo){
			
			this->echo = echo;
			this->trig = trig;
		};

		void SetupUltrasonic(){

			pinMode(echo, INPUT);
			pinMode(trig, OUTPUT);
		};

		long UpdateUltrasonic(){

			digitalWrite(trig, LOW);
			delayMicroseconds(2);
			digitalWrite(trig, HIGH);
			delayMicroseconds(10);
			digitalWrite(trig, LOW);

			long duration = pulseIn(echo, HIGH);

			long cm = duration / 29 / 2;

			return cm;
		};
};

#endif
