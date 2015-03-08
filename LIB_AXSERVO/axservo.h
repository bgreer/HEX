// target: arbotix-m

#define SERVO_DELAY 5

class axservo
{
public:
  uint8_t id;

  axservo (uint8_t i)
  {
    id = i;
  }
  
  // do important things
  void setID (uint8_t val) // UNTESTED
  {
    ax12SetRegister(id,AX_ID,val); // eeprom
    id = val;
  }
  void setPosition (float angle)
  {
    int val;
    val = (int)(angle*1023./300.);
    ax12SetRegister2(id,AX_GOAL_POSITION_L,val); // ram
		delay(SERVO_DELAY);
  }
  void setRotationLimits (float angle1, float angle2)
  {
    int val1, val2;
    
    val1 = (int)(angle1*1023./300.);
    val2 = (int)(angle2*1023./300.);
    
    delay(SERVO_DELAY);
    if (val1 >= 0 && val1 <= 1023 && val1 <= val2)
      ax12SetRegister2(id,AX_CW_ANGLE_LIMIT_L,val1); // eeprom
    else
      Serial.println("ERROR: Invalid rotation limit");
      
    delay(SERVO_DELAY);
    
    if (val2 >= 0 && val2 <= 1023 && val1 <= val2)
      ax12SetRegister2(id,AX_CCW_ANGLE_LIMIT_L,val2); // eeprom
    else
      Serial.println("ERROR: Invalid rotation limit");
    delay(SERVO_DELAY);
  }
	void setTorqueEnable (bool enable)
	{
		if (enable)
			ax12SetRegister(id,AX_TORQUE_ENABLE,1);
		else
			ax12SetRegister(id,AX_TORQUE_ENABLE,0);
	}
  
  // important info
  float getPosition ()
  {
    int val;
    val = ax12GetRegister(id,AX_PRESENT_POSITION_L,2);
    return val*300./1023.; // assumes joint mode
  }
  float getSpeed () // between -1 and 1 (relative to max?)
  {
    int val;
    val = ax12GetRegister(id,AX_PRESENT_SPEED_L,2);
    val = (val&0x3FF) * ((val&0x400)?-1:1);
    return val*0.111; // assumes joint mode
  }
  float getLoad () // between -1 and 1 (relative to max?)
  {
    int val;
    val = ax12GetRegister(id,AX_PRESENT_LOAD_L,2);
    val = (val&0x3FF) * ((val&0x400)?-1:1);
    return val/1024.;
  }
	int getReturnDelay () // in usec
	{
		int val;
		val = ax12GetRegister(id,AX_RETURN_DELAY_TIME,1);
		return val*2;
	}
	bool getTorqueEnable ()
	{
		return (ax12GetRegister(id,AX_TORQUE_ENABLE,1)==1);
	}
  
  // do less important things
  
  int getBaudRate ()
  {
    int val;
    delay(SERVO_DELAY);
    val = ax12GetRegister(id,AX_BAUD_RATE,1);
		return val;
  }
  float getCWAngleLimit ()
	{
    delay(SERVO_DELAY);
		return ax12GetRegister(id,AX_CW_ANGLE_LIMIT_L,2)*300./1023.;
	}
	float getCCWAngleLimit ()
	{
    delay(SERVO_DELAY);
		return ax12GetRegister(id,AX_CCW_ANGLE_LIMIT_L,2)*300./1023.;
	}
  uint8_t getTemperature ()
  {
    delay(SERVO_DELAY);
    return ax12GetRegister(id,AX_PRESENT_TEMPERATURE,1);
  }
	int getTemperatureLimit ()
	{
    delay(SERVO_DELAY);
		return ax12GetRegister(id,AX_LIMIT_TEMPERATURE,1);
	}
	float getLowerVoltageLimit ()
	{
    delay(SERVO_DELAY);
		return ax12GetRegister(id,AX_DOWN_LIMIT_VOLTAGE,1)*0.1;
	}
	float getUpperVoltageLimit()
	{
    delay(SERVO_DELAY);
		return ax12GetRegister(id,AX_UP_LIMIT_VOLTAGE,1)*0.1;
	}
  float getVoltage ()
  {
    delay(SERVO_DELAY);
    return ax12GetRegister(id,AX_PRESENT_VOLTAGE,1)*0.1;
  }

	// for debugging
	void fullDebug ()
	{
		Serial.print("-- FULL DEBUG ON SERVO ");
		Serial.print(id);
		Serial.println(" --");

		Serial.print("0x04 Baud Rate Value: ");
		Serial.println(getBaudRate());

		Serial.print("0x05 Return Delay Time: ");
		Serial.print(getReturnDelay());
		Serial.println(" us");

		Serial.print("0x06 CW Angle Limit: ");
		Serial.println(getCWAngleLimit());

		Serial.print("0x08 CCW Angle Limit: ");
		Serial.println(getCCWAngleLimit());

		Serial.print("0x11 Temperature Limit: ");
		Serial.print(getTemperatureLimit());
		Serial.println("C");

		Serial.print("0x12 Lower Voltage Limit: ");
		Serial.print(getLowerVoltageLimit());
		Serial.println("V");

		Serial.print("0x13 Upper Voltage Limit: ");
		Serial.print(getUpperVoltageLimit());
		Serial.println("V");


		Serial.print("0x42 Current Voltage: ");
		Serial.print(getVoltage());
		Serial.println("V");

		Serial.print("0x43 Current Temperature: ");
		Serial.print(getTemperature());
		Serial.println("C");
	}
};
