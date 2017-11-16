#ifndef _ENCODER_H_
#define _ENCODER_H_

#define TPR 4200  // ticks / per rev

class Encoder{

	private:

		int leftPinA;
		int leftPinB;

		int rightPinA;
		int rightPinB;

		volatile long leftTicks;
		volatile long rightTicks;

		volatile long ticks;

	public:

		Encoder(int leftPinA, int leftPinB, int rightPinA, int rightPinB){

			this->leftPinA = leftPinA;
			this->leftPinB = leftPinB;
			this->rightPinA = rightPinA;
			this->rightPinB = rightPinB;

			leftTicks = 0;
			rightTicks = 0;
			ticks = 0

			attachInterrupt(leftPinA, this->UpdateLeftEncoder, RISING);

			attachInterrupt(rightPinA, this->UpdateRightEncoder, RISING);
		};

		void setup(){
		
			pinMode(leftPinA, INPUT_PULLUP);
			pinMode(leftPinB, INPUT_PULLUP);
			pinMode(rightPinA, INPUT_PULLUP);
			pinMode(rightPinB, INPUT_PULLUP);
		};

		long updateLeftEncoder(){

			int leftB = digitalRead(leftPinB);

			leftTicks -= leftB? -1 : 1;
			
			ticks++;

			return leftTicks;
		};

		long UpdateRightEncoder(){

			int rightB = digitalRead(rightPinB);

			rightTicks += rightB? -1 : 1;

			ticks++;

			return rightTicks;
		};

		long getRightTicks(){

			return rightTicks;
		};

		long getLeftTicks(){

			return leftTicks;
		};

		long getCPR(){

			return ticks * 2;
		}

		void clearTicks(){

			leftTicks = 0;
			rightTicks = 0;
			ticks = 0;
		};
};
#endif
