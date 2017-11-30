#ifndef _ENCODER_H_
#define _ENCODER_H_

#define TPR 4200  // ticks / per rev

class Encoder{

	public:

		static int leftPinA;
		static int leftPinB;

		static int rightPinA;
		static int rightPinB;

		static volatile long leftTicks;
		static volatile long rightTicks;

		static volatile long ticks;

		static void setup(){
		
			pinMode(leftPinA, INPUT_PULLUP);
			pinMode(leftPinB, INPUT_PULLUP);
			pinMode(rightPinA, INPUT_PULLUP);
			pinMode(rightPinB, INPUT_PULLUP);

			attachInterrupt(leftPinA, updateLeftEncoder, RISING);

			attachInterrupt(rightPinA, updateRightEncoder, RISING);
		};

		static void updateLeftEncoder(){

			int leftB = digitalRead(leftPinB);

			leftTicks -= leftB? -1 : 1;
			
			ticks++;

			//return leftTicks;
		};

		static void updateRightEncoder(){

			int rightB = digitalRead(rightPinB);

			rightTicks += rightB? -1 : 1;

			ticks++;

			//return rightTicks;
		};

		Encoder(int leftPinA, int leftPinB, int rightPinA, int rightPinB){

			this->leftPinA = leftPinA;
			this->leftPinB = leftPinB;
			this->rightPinA = rightPinA;
			this->rightPinB = rightPinB;

			leftTicks = 0;
			rightTicks = 0;
			ticks = 0;

		};


		static long getRightTicks(){

			return rightTicks;
		};

		static long getLeftTicks(){

			return leftTicks;
		};

		static long getCPR(){

			return ticks * 2;
		}

		static void clearTicks(){

			leftTicks = 0;
			rightTicks = 0;
			ticks = 0;
		};
};

int Encoder::leftPinA;
int Encoder::leftPinB;

int Encoder::rightPinA;
int Encoder::rightPinB;

volatile long Encoder::leftTicks;
volatile long Encoder::rightTicks;

volatile long Encoder::ticks;

#endif
