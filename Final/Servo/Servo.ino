#include <Servo.h> 

#define MIN_ANGLE_XY 0
#define MAX_ANGLE_XY 150

#define MIN_ANGLE_RZ 0
#define MAX_ANGLE_RZ 150

#define RELAY_PIN 13

Servo XY;
Servo RZ;
int incomingByte;
int temp, angle_xy, angle_rz;

void setup() 
{
    XY.attach(9, 544, 2400);
    RZ.attach(10, 544, 2400);  
    pinMode(RELAY_PIN, OUTPUT);
    temp = 0; 
    Serial.begin(115200);
} 

/* XY, RZ. ON : H OFF : L */
void loop() 
{
    if(Serial.available() > 0)
    {
        incomingByte = Serial.read();
        
        if(incomingByte == ',')
        {
            angle_xy = temp;
            if(angle_xy > MAX_ANGLE_XY)
              angle_xy = MAX_ANGLE_XY;
            if(angle_xy < MIN_ANGLE_XY)
              angle_xy = MIN_ANGLE_XY;
            XY.write(angle_xy);
            temp = 0;
        }
        
        else if(incomingByte == '.')
        {
            angle_rz = temp;
            if(angle_rz > MAX_ANGLE_RZ)
                angle_rz = MAX_ANGLE_RZ;
            if(angle_rz < MIN_ANGLE_RZ)
                angle_rz = MIN_ANGLE_RZ;  
            RZ.write(angle_rz);
            temp = 0;
        }
        
        else if(incomingByte == 'H')
            digitalWrite(RELAY_PIN, HIGH);
        
        else if(incomingByte == 'L')
            digitalWrite(RELAY_PIN, LOW);
            
        else if('0' <= incomingByte && incomingByte <= '9')
        {
          temp *= 10;
          temp += incomingByte - '0'; 
        }
    }
}



