#ifndef _MOTOEDRIVER_H_
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
			this->leftB = leftB
			this->leftPwm = leftPwm;
			this->rightA = rightA;
			this->leftB = leftB
			this->rightPwm = rightPwm;


		void Setup(){
			
			pinMode(leftA, OUTPUT);
			pinMode(leftB, OUTPUT);
			pinMode(leftPwm, OUTPUT);
			pinMode(rightA, OUTPUT);
			pinMode(rightB, OUTPUT);
			pinMode(rightPwm, OUTPUT);
		}

		void Forward(int pwm){

			digitalWrite(leftA, HIGH);
			digitalWrite(leftB, LOW);
			analogWrite(leftPwm, pwm);

			digitalWrite(rightA, LOW);
			digitalWrite(rightB, HIGH);
			analogWrite(rightPwm, pwm);
		}

		void Backward(int pwm){

			digitalWrite(leftA, LOW);
			digitalWrite(leftB, HIGH);
			analogWrite(leftPwm, pwm);

			digitalWrite(rightA, HIGH);
			digitalWrite(rightB, LOW);
			analogWrite(rightPwm, pwm);
		}

		void Stop(){

			digitalWrite(leftA, HIGH);
			digitalWrite(leftB, HIGH);
			analogWrite(leftPwm, 0);

			digitalWrite(rightA, HIGH);
			digitalWrite(rightB, HIGH);
			analogWrite(rightPwm, 0);
		}

		void MoveRight(int pwm){

			digitalWrite(leftA, HIGH);
			digitalWrite(leftB, LOW);
			analogWrite(leftPwm, pwm);

			digitalWrite(rightA, HIGH);
			digitalWrite(rightB, HIGH);
			analogWrite(rightPwm, 0);
		}

		void MoveLeft(int pwm){

			digitalWrite(leftA, HIGH);
			digitalWrite(leftB, HIGH);
			analogWrite(leftPwm, 0);

			digitalWrite(rightA, LOW);
			digitalWrite(rightB, HIGH);
			analogWrite(rightPwm, pwm);
		}

};

#endif
