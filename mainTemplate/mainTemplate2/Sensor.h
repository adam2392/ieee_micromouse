#ifndef Sensor_H_
#define Sensor_H_

#include <Arduino.h>

class Sensor
{
public:
  	/*
  	@brief Class constructor.
   	@param: The sensor pins
   	*/
  	Sensor(int LFEmitter, int LFReceiver, int RFEmitter, 
        int RFReceiver, int SIDEHIGH_POWER, int SIDELOW_POWER,
        int RReceiver, int LReceiver) {
  		
  		// initialize passed in params to the class's vars
  		LF_Emitter = LFEmitter;
  		LF_Receiver = LFReceiver;
  		RF_Emitter = RFEmitter;
  		RF_Receiver = RFReceiver;
  		SIDE_HIGH_POWER = SIDEHIGH_POWER;
  		SIDE_LOW_POWER = SIDELOW_POWER;
  		R_Receiver = RReceiver;
  		L_Receiver = LReceiver;
  	}


	// sensor functions
	// READ SENSORS
	int readLeft(){
	  return analogRead(L_Receiver);
	}
	int readRight(){
	  return analogRead(R_Receiver);
	}
	int readLeftFront(){
	  return analogRead(LF_Receiver);
	}
	int readRightFront(){
	  return analogRead(RF_Receiver);
	}

	// WRITE SENSORS
	void writeSides(){
	  digitalWrite(SIDE_HIGH_POWER, HIGH);
	}
	void writeSidesLow(){
	  digitalWrite(SIDE_LOW_POWER, HIGH);
	}
	void writeLeftFront(){
	  digitalWrite(LF_Emitter, HIGH);
	}
	void writeRightFront(){
	  digitalWrite(RF_Emitter, HIGH);
	}
	void stopLeftFront(){
	  digitalWrite(LF_Emitter, LOW);  
	}
	void stopRightFront(){ 
	  digitalWrite(RF_Emitter, LOW );  
	}
	void stopSides(){
	  digitalWrite(SIDE_HIGH_POWER, LOW );  
	}

protected:
	int LF_Emitter; 	//left front emitter
	int LF_Receiver; 	//left front receiver
	int RF_Emitter;     //right front emitter
	int RF_Receiver;     //right front receiver
	int SIDE_HIGH_POWER;  //turn on Right and Left Emitters
	int SIDE_LOW_POWER;  //turn off Right and Left Emitters
	int R_Receiver;    //Right sensor receiver
	int L_Receiver;
};

#endif
