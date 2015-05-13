#ifndef Drive_H_
#define Drive_H_

#include <Arduino.h>

class Drive
{
public:
  	/*
  	@brief Class constructor.
   	@param: The sensor pins
   	*/
  	Drive(int Rfwd, int Rbkw, int Lfwd, int Lbkw) {
  		// initialize passed in params to the class's vars
  		R_fwd = Rfwd;
  		R_bkw = Rbkw;
  		L_fwd = Lfwd;
  		L_bkw = Lbkw;
  	}

	// DRIVING Functions for left and right motor ***** Probably needs testing to set pwmvalue
	//turn left motor on or off
	void leftForward(int pwmvalue){
	  //digital write for testing/debugging
	  analogWrite(L_fwd, pwmvalue);
	  analogWrite(L_bkw, LOW);
	}
	void rightForward(int pwmvalue){
	  //digital write for testing/debugging
	  analogWrite(R_fwd, pwmvalue);
	  analogWrite(R_bkw, LOW);
	}
	void leftBackward(int pwmvalue){
	  analogWrite(L_fwd, LOW);
	  analogWrite(L_bkw, pwmvalue);
	}
	void rightBackward(int pwmvalue){
	  analogWrite(R_fwd, LOW);
	  analogWrite(R_bkw, pwmvalue);  
	}
protected:
	int R_fwd;
	int R_bkw;
	int L_fwd;
	int L_bkw;
};

#endif
