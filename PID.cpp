/*
 * PID.cpp
 *
 *  Created on: Nov 23, 2014
 *      Author: Adam Li / adam2392
 */

#include “PID.h”

/* Function: PID(PID_params *p)
Description: Only constructor for the PID class
Inputs: PID_param *p - a pointer to the struct of PID_parameters
*/
PID::PID(PID_param *p) {
	//m_param stores all the variables in struct and their values
	m_param = p;	//the private m_param variable gets pointer to struct
}

/* Function: setInput(double in)
Description: Sets the input variable to the PID algorithm
Inputs: double in - a numerical value for the input
*/
void PID::setInput(double in) {
	m_param->input = in;
}

/* Function: setSetPoint(double set)
Description: Sets the set point (goal to reach) variable to the PID algorithm
Inputs: double set - a numerical value for the stepping we want to reach
*/
void PID::setSetPoint(double set) {
	m_param->set = set;
}

double PID::getOutput() {
	return m_param->output;
}

//reset all struct variables
void PID::clear() {
	m_param->error = 0;
	m_param->previous = 0;
	m_param->accumulated = 0;
	m_param->input = 0;
	m_param->output = 0;
}

//dt is the amount of time the program has been running
void PID::process(double dt) {
	//calculate the error
	m_param->error = m_param->set - m_param->input;

	//integrate error
	m_param->accumulated += m_param->error * dt;

	//limit integrated error to +/- accLimit
	if (m_param->accumulated > m_param->accLimit {
		m_param->accumulated = m_param->accLimit;
	}
	else if (m_param->accumulated < -m_param->accLimit) {
		m_param->accumulated = -m_param->accLimit;
	}

	//perform PID calculation
	m_param->output = m_param->proportional * m_param->error 
			+ m_param->integral * m_param->accumulated + 
			m_param->derivative * (m_param->error - m_param->previous) / dt;
	
	//save current error for next time to calculate derivative
	m_param->previous = m_param->error;
}


