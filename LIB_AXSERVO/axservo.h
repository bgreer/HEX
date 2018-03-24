// target: arbotix-m
// depends on ax12.h for things like ax12SetRegister()
#include <ax12.h>

class axservo
{
public:
    static constexpr uint64_t SERVO_DELAY = 10; // us

    axservo (uint8_t i, bool reversed = false);
	
    void setPosition (const float& angle);
    void setRotationLimits (float angle1, float angle2);
	void setTorqueEnable (bool enable);
	void setComplianceSlopes (int cw, int ccw);
    void setComplianceMargins (int cw, int ccw);
	uint8_t getError ();

  // important info
  float getPosition ()
  {
    int val;
    val = ax12GetRegister(_id,AX_PRESENT_POSITION_L,2);
		_errid = ax12GetLastError();
		if (_reverse) val = 1023-val;
    return val*300./1023.; // assumes joint mode
  }
  float getSpeed () // between -1 and 1 (relative to max?)
  {
    int val;
    val = ax12GetRegister(_id,AX_PRESENT_SPEED_L,2);
		_errid = ax12GetLastError();
    val = (val&0x3FF) * ((val&0x400)?-1:1);
		if (_reverse) val = -val;
    return val*0.111; // assumes joint mode
  }
  float getLoad () // between -1 and 1 (relative to max?)
  {
    int val;
    val = ax12GetRegister(_id,AX_PRESENT_LOAD_L,2);
		_errid = ax12GetLastError();
    val = (val&0x3FF) * ((val&0x400)?-1:1);
		if (_reverse) val = -val;
    return val/1024.;
  }
	int getReturnDelay () // in usec
	{
		int val;
		delayMicroseconds(SERVO_DELAY);
		val = ax12GetRegister(_id,AX_RETURN_DELAY_TIME,1);
		_errid = ax12GetLastError();
		return val*2;
	}
	void setReturnDelay (int usec)
	{
		int val;
		val = usec/2;
		delayMicroseconds(SERVO_DELAY);
		ax12SetRegister(_id,AX_RETURN_DELAY_TIME,val);
	}
	bool getTorqueEnable ()
	{
		return (ax12GetRegister(_id,AX_TORQUE_ENABLE,1)==1);
	}
  
  // do less important things
  
  int getBaudRate ()
  {
    int val;
    delayMicroseconds(SERVO_DELAY);
    val = ax12GetRegister(_id,AX_BAUD_RATE,1);
		_errid = ax12GetLastError();
		return val;
  }
  float getCWAngleLimit ()
	{
    delayMicroseconds(SERVO_DELAY);
		if (_reverse) return ax12GetRegister(_id,AX_CCW_ANGLE_LIMIT_L,2)*300./1023.;
		return ax12GetRegister(_id,AX_CW_ANGLE_LIMIT_L,2)*300./1023.;
	}
	float getCCWAngleLimit ()
	{
    delayMicroseconds(SERVO_DELAY);
		if (_reverse) return ax12GetRegister(_id,AX_CW_ANGLE_LIMIT_L,2)*300./1023.;
		return ax12GetRegister(_id,AX_CCW_ANGLE_LIMIT_L,2)*300./1023.;
	}
  uint8_t getTemperature ()
  {
    delayMicroseconds(SERVO_DELAY);
    return ax12GetRegister(_id,AX_PRESENT_TEMPERATURE,1);
  }
	int getTemperatureLimit ()
	{
    delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_LIMIT_TEMPERATURE,1);
	}
	float getMaxTorque ()
	{
		delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_MAX_TORQUE_L,2)*100./1023.;
	}
	int getReturnLevel ()
	{
		delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_RETURN_LEVEL,1);
	}
	void setReturnLevel (int level)
	{
		delayMicroseconds(SERVO_DELAY);
		ax12SetRegister(_id,AX_RETURN_LEVEL,level);
	}
	int getAlarmLED ()
	{
		delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_ALARM_LED,1);
	}
	int getAlarmShutdown ()
	{
		delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_ALARM_SHUTDOWN,1);
	}
	float getLowerVoltageLimit ()
	{
    delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_DOWN_LIMIT_VOLTAGE,1)*0.1;
	}
	float getUpperVoltageLimit()
	{
    delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_UP_LIMIT_VOLTAGE,1)*0.1;
	}
	int getLEDStatus ()
	{
		delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_LED,1);
	}
	float getCWComplianceMargin ()
	{
		delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_CW_COMPLIANCE_MARGIN,1)*300./1023.;
	}
	float getCCWComplianceMargin ()
	{
		delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_CCW_COMPLIANCE_MARGIN,1)*300./1023.;
	}
	int getCWComplianceSlope ()
	{
		delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_CW_COMPLIANCE_SLOPE,1);
	}
	int getCCWComplianceSlope ()
	{
		delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_CCW_COMPLIANCE_SLOPE,1);
	}
	float getTorqueLimit ()
	{
		delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_TORQUE_LIMIT_L,2)*100./1023.;
	}
  float getVoltage ()
  {
    delayMicroseconds(SERVO_DELAY);
    return ax12GetRegister(_id,AX_PRESENT_VOLTAGE,1)*0.1;
  }
	float getPunch ()
	{
		delayMicroseconds(SERVO_DELAY);
		return ax12GetRegister(_id,AX_PUNCH_L,2)*100./1023.;
	}

	// for debugging
	void fullDebug ()
	{
		Serial.print("-- FULL DEBUG ON SERVO ");
		Serial.print(_id);
		Serial.println(" --");

		Serial.print("04 Baud Rate Value: ");
		Serial.println(getBaudRate());

		Serial.print("05 Return Delay Time: ");
		Serial.print(getReturnDelay());
		Serial.println(" us");

		Serial.print("06 CW Angle Limit: ");
		Serial.println(getCWAngleLimit());

		Serial.print("08 CCW Angle Limit: ");
		Serial.println(getCCWAngleLimit());

		Serial.print("11 Temperature Limit: ");
		Serial.print(getTemperatureLimit());
		Serial.println("C");

		Serial.print("12 Lower Voltage Limit: ");
		Serial.print(getLowerVoltageLimit());
		Serial.println("V");

		Serial.print("13 Upper Voltage Limit: ");
		Serial.print(getUpperVoltageLimit());
		Serial.println("V");

		Serial.print("14 Max Torque: ");
		Serial.print(getMaxTorque());
		Serial.println("%");

		Serial.print("16 Return Level: ");
		Serial.println(getReturnLevel());

		Serial.print("17 Alarm LED: ");
		Serial.println(getAlarmLED());

		Serial.print("18 Alarm Shutdown: ");
		Serial.println(getAlarmShutdown());

		Serial.print("24 Torque Enable: ");
		if (getTorqueEnable()) Serial.println("true");
		else Serial.println("false");

		Serial.print("25 LED Status: ");
		Serial.println(getLEDStatus());

		Serial.print("26 CW Compliance Margin: ");
		Serial.println(getCWComplianceMargin());

		Serial.print("27 CCW Compliance Margin: ");
		Serial.println(getCCWComplianceMargin());

		Serial.print("28 CW Compliance Slope: ");
		Serial.println(getCWComplianceSlope());
		
		Serial.print("29 CCW Compliance Slope: ");
		Serial.println(getCCWComplianceSlope());

		Serial.print("34 Torque Limit: ");
		Serial.print(getTorqueLimit());
		Serial.println("%");

		Serial.print("36 Current Position: ");
		Serial.println(getPosition());

		Serial.print("42 Current Voltage: ");
		Serial.print(getVoltage());
		Serial.println("V");

		Serial.print("43 Current Temperature: ");
		Serial.print(getTemperature());
		Serial.println("C");


		Serial.print("48 Punch: ");
		Serial.print(getPunch());
		Serial.println("%");
	}
private:
	uint8_t _id, _errid;
	bool _reverse;
	// the 'reverse' boolean allows for a reversal of angle (target, limits, etc)
	// this helps when some servos are mounted backwards, don't need to worry about
	// orientation every time you set something.
	
	// an error code is included in data packets received from the servos whenever asked for data
	// try to grab this error code as often as possible

};
