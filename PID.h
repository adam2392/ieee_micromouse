/*
 * PID.h
 *
 *  Created on: Nov 23, 2014
 *      Author: Adam Li / adam2392

Q:
1. do we need accLimit error?
2. How to calculate error in our case?
 */



#ifndef PID_H_
#define PID_H_

struct PID_params {
	double set;		//setpoint
	double input;		//input into PID
	double output;		//output variable after PID calculation
	double accumulated;	//accumulated error
	double previous;	//previous error
	double error;		//current amount of error

	double proportional;	//Kp term
	double integral;	//Ki term
	double derivative;	//Kd term

	double accLimit;
};

typedef struct PID_params PID_params;

class PID {
public:
	//Constructor passed in pointer p to the PID_params struct
	PID(PID_params *p);	
	
	void setInput(double in);	//set the input into the PID algorithm
	void setSetPoint(double set);	//set the set point you want to reach
	double getOutput();		//access the output of PID
	void clear();			//clear up the PID params
	void process(double dt);	//carry out the PID process
private:
	PID_params *m_param;
};

#endif /* PID_H_ */
