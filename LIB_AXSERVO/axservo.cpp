#include "axservo.h"

axservo::axservo (uint8_t i, bool reversed) {
    _errid = 0;
    _id = i;
    _reverse = reversed;
}

// set the target angle
void axservo::setPosition (const float& angle)
{
    int val = (int)(angle*1023./300.);
    if (_reverse) val = 1023-val;
    ax12SetRegister2(_id,AX_GOAL_POSITION_L,val); // ram
    delayMicroseconds(SERVO_DELAY);
}

void axservo::setRotationLimits (float angle1, float angle2)
{
    int val1, val2;

    val1 = (int)(angle1*1023./300.);
    val2 = (int)(angle2*1023./300.);

    if (_reverse)
    {
        val1 = 1023-val1;
        val2 = 1023-val2;
    }

    delayMicroseconds(SERVO_DELAY);
    if (val1 >= 0 && val1 <= 1023 && val1 <= val2)
        ax12SetRegister2(_id,AX_CW_ANGLE_LIMIT_L,val1); // eeprom
    else
        Serial.println("ERROR: Invalid rotation limit");

    delayMicroseconds(SERVO_DELAY);

    if (val2 >= 0 && val2 <= 1023 && val1 <= val2)
        ax12SetRegister2(_id,AX_CCW_ANGLE_LIMIT_L,val2); // eeprom
    else
        Serial.println("ERROR: Invalid rotation limit");
    delayMicroseconds(SERVO_DELAY);
}

void axservo::setTorqueEnable (bool enable)
{
    if (enable) {
        ax12SetRegister(_id,AX_TORQUE_ENABLE,1);
    } else {
        ax12SetRegister(_id,AX_TORQUE_ENABLE,0);
    }
}

void axservo::setComplianceSlopes (int cw, int ccw)
{
    int temp;
    if (_reverse)
    {
        temp = cw;
        cw = ccw;
        ccw = temp;
    }
    delayMicroseconds(SERVO_DELAY);
    ax12SetRegister(_id,AX_CW_COMPLIANCE_SLOPE,cw);
    delayMicroseconds(SERVO_DELAY);
    ax12SetRegister(_id,AX_CCW_COMPLIANCE_SLOPE,ccw);
}

void axservo::setComplianceMargins (int cw, int ccw)
{
    int temp;
    if (_reverse)
    {
        temp = cw;
        cw = ccw;
        ccw = temp;
    }
    delayMicroseconds(SERVO_DELAY);
    ax12SetRegister(_id,AX_CW_COMPLIANCE_MARGIN,cw);
    delayMicroseconds(SERVO_DELAY);
    ax12SetRegister(_id,AX_CCW_COMPLIANCE_MARGIN,ccw);
}

uint8_t axservo::getError ()
{
    _errid = ax12GetLastError();
    return _errid;
}

