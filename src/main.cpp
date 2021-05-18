#include <Arduino.h>
#include <Wire.h>
#include <main.h>



  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
  uint8_t readByte(uint8_t address, uint8_t subAddress);
  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
 

enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2 
};


struct  imu_t {
  int16_t  accSmooth[3];
  int16_t  gyroData[3];
  int16_t  magADC[3];
  int16_t  gyroADC[3];
  int16_t  accADC[3];
};  
struct imu_t imu;
 

#define MPU6050_ADDRESS     0x68  
#define GRYO_DLPF_CFG       0x00

#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = X; imu.accADC[PITCH]  = Y; imu.accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = X; imu.gyroADC[PITCH] = Y; imu.gyroADC[YAW] = Z;}
static uint8_t rawADC[6];

 
int16_t AcX, AcY, AcZ, acc_total_vector, Tmp, GyX, GyY, GyZ;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
int16_t Gyro_X, Gyro_Y, Gyro_Z;
double gyro_axis_cal[4]; 
 float roll_level_adjust, pitch_level_adjust;

  void computeIMU();
void getGyro();
void getACC();

 void ACC_init();
 void Gyro_init();

void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);
  Wire.setClock(400000);
  pinMode(2, OUTPUT);
  Gyro_init();
  ACC_init();
 
}
 
 unsigned long loop_timer;
unsigned long firstInterval  = 1000;  //1kHz
unsigned long secondInterval = 2000; //500Hz
unsigned long thirdInterval  = 4000; //250Hz
unsigned long firstTimer  = 0;
unsigned long secondTimer = 0;
unsigned long thirdTimer  = 0;


 void loop()
{
  unsigned long now = micros();
  if(now - firstTimer >= firstInterval)  //get gyro and filter it out
  {
    firstTimer = micros();
     getGyro();   
             
    imu.gyroData[ROLL]  = imu.gyroData[ROLL]  * 0.9  + imu.gyroADC[ROLL]  * 0.1;
    imu.gyroData[PITCH] = imu.gyroData[PITCH] * 0.9  + imu.gyroADC[PITCH] * 0.1;
    imu.gyroData[YAW]   = imu.gyroData[YAW]   * 0.9  + imu.gyroADC[YAW]   * 0.1; 
    computeIMU();
  }

   if(now - secondTimer >= secondInterval)
  {
    secondTimer = micros();
   getACC();
  }

  if(now - thirdTimer >= thirdInterval)
  {
    thirdInterval = micros();
    Serial.println(angle_roll);
    

  }
   
  
}

  
 void Gyro_init() {
 
 
  Wire.beginTransmission( MPU6050_ADDRESS );
  Wire.write(0x6B);                    // PWR_MGMT_1 register
  Wire.write(0x80);                    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(50);

  Wire.beginTransmission( MPU6050_ADDRESS );
  Wire.write(0x6B);                   //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)                 
  Wire.write(0x03);                               
  Wire.endTransmission(true);

  Wire.beginTransmission( MPU6050_ADDRESS );
  Wire.write(0x1A);                         
  Wire.write(GRYO_DLPF_CFG);          //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)                     
  Wire.endTransmission(true);

  Wire.beginTransmission( MPU6050_ADDRESS );
  Wire.write(0x1B );                         
  Wire.write( 0x18 );          //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec                        
  Wire.endTransmission(true);
 
}

 void ACC_init() {

  Wire.beginTransmission( MPU6050_ADDRESS );
  Wire.write( 0x1C );            //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.                  
  Wire.write( 0x10 );                          
  Wire.endTransmission(true);
       
}

 void getGyro() {
 
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, 1);
  imu.gyroADC[ROLL]  = Wire.read() << 8 | Wire.read();//ROLL
  imu.gyroADC[PITCH] = Wire.read() << 8 | Wire.read();//PITCH
  imu.gyroADC[YAW]   = Wire.read() << 8 | Wire.read();
}
 
 void getACC() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, 1);
 
  imu.accADC[ROLL]  = Wire.read() << 8 | Wire.read();//ROLL
  imu.accADC[PITCH] = Wire.read() << 8 | Wire.read();//PITCH
  imu.accADC[YAW]   = Wire.read() << 8 | Wire.read();
 
}

 
void computeIMU()
{
 
  angle_pitch += imu.gyroADC[PITCH] *  0.00001526;  // 500hz 0.00003053 , 0.0000005328   //1000Hz 0.00001526 , 0.0000002663
  angle_roll  += imu.gyroADC[ROLL] *  0.00001526;

  angle_pitch -= angle_roll * sin(imu.gyroADC[YAW]  * 0.0000002663);   //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(imu.gyroADC[YAW]  * 0.0000002663);   //If the IMU has yawed transfer the pitch angle to the roll angel.

  acc_total_vector = sqrt((imu.accADC[ROLL] * imu.accADC[ROLL]) + (imu.accADC[PITCH] * imu.accADC[PITCH]) + (imu.accADC[YAW] * imu.accADC[YAW]));    //Calculate the total accelerometer vector.

  if (abs(imu.accADC[YAW]) < acc_total_vector) {
    angle_roll_acc = asin((float)imu.accADC[YAW] / acc_total_vector) * 57.296;
  }

  if (abs(imu.accADC[ROLL]) < acc_total_vector) {
    angle_pitch_acc = asin((float)imu.accADC[ROLL] / acc_total_vector) * -57.296;
  }
  angle_pitch_acc -= -0.25;     //calibration values                                               
  angle_roll_acc -= 88.52;     //calibration values
  
  angle_pitch = angle_pitch * 0.99 + angle_pitch_acc * 0.01;
  angle_roll  = angle_roll  * 0.99 + angle_roll_acc  * 0.01;
 
} 




 