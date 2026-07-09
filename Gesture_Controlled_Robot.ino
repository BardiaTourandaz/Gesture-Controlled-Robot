// #include <Arduino.h>
#include <Wire.h>
#include <TinyMPU6050.h>
#include <SoftwareSerial.h>
SoftwareSerial bluetooth(7,6); // RX, TX
char t;
float yaw;
float setPoint = 0;
float Kp = 3;
float Kd = 1;
float Ki = 0;
double err;
double toerr = 0;
double preerr = 0;
int strt = 80;
MPU6050 mpu (Wire);
#define MLa 5     //left motor 1st pin
#define MLb 9     //left motor 2nd pin
#define MRa 10    //right motor 1st pin
#define MRb 11    //right motor 2nd pin
int control_sig;
int i = 5;
const long interval = 3000;
unsigned long premilis = 0;
#include<Wire.h>
unsigned long strt;
unsigned long current;
#define MPU6050_AXOFFSET 158
#define MPU6050_AYOFFSET 9
#define MPU6050_AZOFFSET -91
#define MPU6050_GXOFFSET 19
#define MPU6050_GYOFFSET -42
#define MPU6050_GZOFFSET -26
bool is_L = false;
bool is_R = false;
bool is_f = false;
long sampling_timer;
const int MPU_addr=0x68;  // I2C address of the MPU-6050
const long time = 7000;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // Raw data of MPU6050
float GAcX, GAcY, GAcZ; // Convert accelerometer to gravity value
float Cal_GyX,Cal_GyY,Cal_GyZ; // Pitch, Roll & Yaw of Gyroscope applied time factor
float acc_pitch, acc_roll, acc_yaw; // Pitch, Roll & Yaw from Accelerometer
float angle_pitch, angle_roll, yaw; // Angle of Pitch, Roll, & Yaw
float alpha = 0.95; // Complementary constant
const long time = 12000;
void setup()
{
  Wire.begin();
 
  init_MPU6050();
  strt = millis();
  Serial.begin(115200);
  bluetooth.begin(9600);
pinMode(MLa,OUTPUT);   //left motors forward
pinMode(MLb,OUTPUT);   //left motors reverse
pinMode(MRa,OUTPUT);   //right motors forward
pinMode(MRb,OUTPUT);   //right motors reverse
}
void loop() 
{
  current = millis();
  if(current - strt < 6000)
  {
    forward();
  }
  if(current - strt >= 6000 && current - strt<= 8000)
  {
    is_L = true;
  }

  if(bluetooth.available()>0)
  {
    t = bluetooth.read();
    Serial.println(t);
    Read raw data of MPU6050
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    GAcX = (float) AcX / 4096.0;
    GAcY = (float) AcY / 4096.0;
    GAcZ = (float) AcZ / 4096.0;
    acc_pitch = atan ((GAcY - (float)MPU6050_AYOFFSET/4096.0) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951; // 180 / PI = 57.29577951
    acc_roll = - atan ((GAcX - (float)MPU6050_AXOFFSET/4096.0) / sqrt(GAcY * GAcY + GAcZ * GAcZ)) * 57.29577951; 
    acc_yaw = atan (sqrt(GAcX * GAcX + GAcZ * GAcZ) / (GAcZ - (float)MPU6050_AZOFFSET/4096.0)) * 57.29577951; 
    Cal_GyX += (float)(GyX - MPU6050_GXOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
    Cal_GyY += (float)(GyY - MPU6050_GYOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
    Cal_GyZ += (float)(GyZ - MPU6050_GZOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
    angle_pitch = alpha * (((float)(GyX - MPU6050_GXOFFSET) * 0.000244140625) + angle_pitch) + (1 - alpha) * acc_pitch;
    angle_roll = alpha * (((float)(GyY - MPU6050_GYOFFSET) * 0.000244140625) + angle_roll) + (1 - alpha) * acc_roll;
    yaw += (float)(GyZ - MPU6050_GZOFFSET) * 0.000244140625; // Accelerometer doesn't have yaw value
    Serial.print(" | angle_yaw = "); Serial.println(yaw);
    while(micros() - sampling_timer < 4000); //
    sampling_timer = micros(); //Reset the sampling timer  
    Serial.print("\n");
    setpoint();
    err = setPoint - yaw;
    double P = Kp*err;
    double D = Kd *(err - preerr);
    double I = Ki*(toerr);
    double PD = P+D+I;
    preerr= err;
    toerr += err;
    control_sig = (int)PD;
    if(control_sig < -10)
    {
      control_sig = -10;
      }  
    if(control_sig < 0)
    {
      control_sig = -control_sig;
      }
    if(control_sig > 5)
    {
      control_sig = 5;
      }
    Serial.print("control signal: ");
    Serial.print(control_sig);
    Serial.print("\n");
    if(i == 5 )
    {
      forward();
      }
    if(i != 5)
    {
      Break();
      }
    if(i != 5)
    {
      left();
      is_L = true;
      }
    if(i != 5)
    {
      is_R = true;
      }
    if(i != 5)
    {
      stop();
      }
    if(is_R == true)
    {
      right();
    }
    if(is_f == true)
    {
      forward();
    }
// Serial.print("i: ");
// Serial.print(i);
// Serial.print("\n");
// delay(100);
  
}
void setpoint()
{
  if(setPoint == 0)
  {
    setPoint = yaw;
  }
  else
  {
    
  }
//   Serial.print("setpoint: ");
//   Serial.print(setPoint);
//   Serial.print("\n");
}
void forward()
{
  if(err < 0)
  {

    analogWrite(MLa,80 + control_sig);
    digitalWrite(MLb,LOW);
    digitalWrite(MRa,LOW);
    analogWrite(MRb,60 - 2*(control_sig));
    Serial.print("Left_s : ");
    Serial.print(79 + control_sig);
    Serial.print("\n");
    Serial.print("Right_s : ");
    Serial.print(2 - (control_sig));
    Serial.print("\n");
  }
  if(err > 0)
  {
    analogWrite(MLa,80 - (control_sig));
    digitalWrite(MLb,LOW);
    digitalWrite(MRa,LOW);
    analogWrite(MRb,60 +(control_sig));
    Serial.print("Left_s : ");
    Serial.print(79 - control_sig);
    Serial.print("\n");
    Serial.print("Right_s : ");
    Serial.print(62 + (control_sig));
    Serial.print("\n");
  }
}
void Break()
{
  if(err < 0)
  {
  digitalWrite(MLa,LOW);
  analogWrite(MLb,70 - (control_sig));
  analogWrite(MRa,58 + (control_sig));
  digitalWrite(MRb,LOW);     
  }
  if(err > 0)
  {
  digitalWrite(MLa,LOW);
  analogWrite(MLb,70 + (control_sig));
  analogWrite(MRa,58 - (control_sig));
  digitalWrite(MRb,LOW); 
  }
}
void right()
{
  if(yaw > setPoint - 87)
  {
  analogWrite(MLa,80);
  digitalWrite(MLb,LOW);
  digitalWrite(MRa,LOW);
  digitalWrite(MRb,LOW);
  }
  else
  {
    stop();
    is_R = false;
    // setPoint -= 87;
  }
}
void left()
{
  if(yaw < setPoint + 87)
  {
  digitalWrite(MLa,LOW);
  digitalWrite(MLb,LOW);
  digitalWrite(MRa,LOW);
  analogWrite(MRb,50);
  }
  if(yaw > setPoint + 87)
  {
    stop();
    is_L = false;
    setPoint = yaw;
    Break();
    // setPoint += 87;
  }
}
void stop()
{
  digitalWrite(MLa,LOW);
  digitalWrite(MLb,LOW);
  digitalWrite(MRa,LOW);
  digitalWrite(MRb,LOW); 
}
void init_MPU6050(){
  //MPU6050 Initializing & Reset
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //MPU6050 Clock Type
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x03);     // Selection Clock 'PLL with Z axis gyroscope reference'
  Wire.endTransmission(true);

  //MPU6050 Gyroscope Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // Gyroscope Configuration register
  //Wire.write(0x00);     // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
  //Wire.write(0x08);     // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
  //Wire.write(0x10);     // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
  Wire.write(0x18);     // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]
  Wire.endTransmission(true);

  //MPU6050 Accelerometer Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // Accelerometer Configuration register
  //Wire.write(0x00);     // AFS_SEL=0, Full Scale Range = +/- 2 [g]
  //Wire.write(0x08);     // AFS_SEL=1, Full Scale Range = +/- 4 [g]
  Wire.write(0x10);     // AFS_SEL=2, Full Scale Range = +/- 8 [g]
  //Wire.write(0x18);     // AFS_SEL=3, Full Scale Range = +/- 10 [g]
  Wire.endTransmission(true);

  //MPU6050 DLPF(Digital Low Pass Filter)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);  // DLPF_CFG register
  Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
  //Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz 
  //Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz 
  //Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz 
  //Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz 
  //Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz 
  //Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz 
  Wire.endTransmission(true);
}