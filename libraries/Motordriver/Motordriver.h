#ifndef _MOTOERDRIVER_H_
#define _MOTOERDRIVER_H_

class Motordriver{

	/*****************************************
		INA	INB	MODE
		1	1	Vcc brake
		1	0	Clockwise
		0	1	Counterclockwise
		0	0	Gnd brake
	*****************************************/

	private:

		int leftA;	// pin control leftr motor
		int leftB;
		int leftPwm;

		int rightA;	// pin control right motor
		int rightB;
		int rightPwm;

	public:

		Motordriver(int leftA, int leftB, int leftPwm, int rightA, int rightB, int rightPwm){

			this->leftA = leftA;
			this->leftB = leftB;
			this->leftPwm = leftPwm;
			this->rightA = rightA;
			this->rightB = rightB;
			this->rightPwm = rightPwm;

		};


		void setup(){
			
			pinMode(leftA, OUTPUT);
			pinMode(leftB, OUTPUT);
			pinMode(leftPwm, OUTPUT);
			pinMode(rightA, OUTPUT);
			pinMode(rightB, OUTPUT);
			pinMode(rightPwm, OUTPUT);
		};

		void drive(int pwmR, int pwmL){
			
			if(pwmR > 0 && pwmL > 0)
				forward(pwmR, pwmL);
			else if(pwmR < 0 && pwmL < 0)
				backward(pwmR, pwmL);
			else if(pwmR > 0)
				moveLeft(pwmR, pwmL);
			else if(pwmL > 0)
				moveRight(pwmR, pwmL);			
			else 
				stop();
			
		};

		void forward(int pwmR, int pwmL){
			
			digitalWrite(rightA, LOW);
			digitalWrite(rightB, HIGH);
			analogWrite(rightPwm, pwmR);
			delay(2);

			digitalWrite(leftA, HIGH);
			digitalWrite(leftB, LOW);
			analogWrite(leftPwm, pwmL);
			delay(2);
		};

		void backward(int pwmR, int pwmL){

			digitalWrite(rightA, HIGH);
			digitalWrite(rightB, LOW);
			analogWrite(rightPwm, -pwmR);
			delay(2);

			digitalWrite(leftA, LOW);
			digitalWrite(leftB, HIGH);
			analogWrite(leftPwm, -pwmL);
			delay(2);
		};

		void stop(){

			digitalWrite(leftA, HIGH);
			digitalWrite(leftB, HIGH);
			analogWrite(leftPwm, 0);
			delay(2);

			digitalWrite(rightA, HIGH);
			digitalWrite(rightB, HIGH);
			analogWrite(rightPwm, 0);
			delay(2);
		};

		void moveRight(int pwmR, int pwmL){

			if(pwmR > 0){
				digitalWrite(rightA, LOW);
				digitalWrite(rightB, HIGH);
				analogWrite(rightPwm, pwmR);
				delay(2);
			}
			else if(pwmR < 0){
				digitalWrite(rightA, HIGH);
				digitalWrite(rightB, LOW);
				analogWrite(rightPwm, -pwmR);
				delay(2);

			}
			else{
				digitalWrite(rightA, HIGH);
				digitalWrite(rightB, HIGH);
				analogWrite(rightPwm, 0);
				delay(2);
			}

			digitalWrite(leftA, HIGH);
			digitalWrite(leftB, LOW);
			analogWrite(leftPwm, pwmL);
			delay(2);
		};

		void moveLeft(int pwmR, int pwmL){

			digitalWrite(rightA, LOW);
			digitalWrite(rightB, HIGH);
			analogWrite(rightPwm, pwmR);
			delay(2);

			if(pwmL > 0){
				digitalWrite(leftA, HIGH);
				digitalWrite(leftB, LOW);
				analogWrite(leftPwm, pwmL);
				delay(2);
			}
			else if(pwmL < 0){
				digitalWrite(leftA, LOW);
				digitalWrite(leftB, HIGH);
				analogWrite(leftPwm, -pwmL);
				delay(2);
			}
			else{
				digitalWrite(leftA, HIGH);
				digitalWrite(leftB, HIGH);
				analogWrite(leftPwm, 0);
				delay(2);
			}

		};

};

#endif
