// includes section ------------------------------------------------------------------------------


	#include <Adafruit_Sensor.h> // need for Adafruit_BMP3XX
	#include <Adafruit_BMP3XX.h> // pressure sensor
	#include <kalmanvert.h> // kalman fusion filter
	#include <Simple_MPU6050.h> // accel lib
	#include <toneAC.h> // makes loud beeps
	#include <LCD5110_Graph.h> // display

// includes section ------------------------------------------------------------------------------



// lib objects section ----------------------------------------------------------------------------

	kalmanvert kalmanvert;
	Adafruit_BMP3XX bmp; // I2C
	Simple_MPU6050 mpu;
	ENABLE_MPU_OVERFLOW_PROTECTION();
	// SCK  - Pin 8, MOSI - Pin 6, DC   - Pin 7, RST  - Pin 11, CS   - Pin 12 -> display pins
	LCD5110 myGLCD(8, 6, 7, 11, 12);


// lib objects section ----------------------------------------------------------------------------


// defs section ------------------------------------------------------------------------------


#define MPU6050_ADDRESS_AD0_LOW     0x68
#define SEALEVELPRESSURE_HPA (1013.25)
/*             _________________________________________________________*/
//              X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
#define OFFSETS  -2476,   -3272,     336,     162,     -53,      41
#define VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER 6
#define VERTACCEL_CAL_SCALE_MULTIPLIER 16
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 14
#define LIGHT_INVENSENSE_QUAT_SCALE_SHIFT 30
#define LIGHT_INVENSENSE_QUAT_SCALE ((double)(1LL << LIGHT_INVENSENSE_QUAT_SCALE_SHIFT))
#define VERTACCEL_G_TO_MS 9.80665
#define INITIAL_FREQ_UP (2500) // starting frequency of beep on up
#define FREQ_DOWN (400) // frequency of beep on down
#define FREQ_COEF (100) //  coeficent for bumping up  frequency on higher vario
#define DOWN_VARIO_SOUND_THRESHHOLD (-2) // m/s
#define UP_MIN_VARIO_SOUND_THRESHOLD (0.5) // m/s
#define UP_MAX_VARIO_SOUND_THRESHOLD (10) // m/s
#define BEEP_DURATION (300) // ms
#define BEEP_DURATION_X2 (BEEP_DURATION * 2) // for pause after beep, by default shoold be twice BEEP_DURATION
#define BEEP_DURATION_COEF (50) // ms, for shorter beeps
#define BEEP_DURATION_COEF_X2 (BEEP_DURATION_COEF * 2) // by default shoold be twice BEEP_DURATION_COEF

// defs section ------------------------------------------------------------------------------



// useful vars ------------------------------------------------------------------------------

    boolean  fl = true; // flag for determine beep or not to beep
    extern uint8_t SmallFont[]; // fonts for dsiplay
    extern uint8_t MediumNumbers[]; // fonts for dsiplay
    int val = 0; // for display
    int flyAlt = 0; 
    int startAlt = 0;
    int curAlt = 0;
    unsigned long time = 0; // for beeps duration

// useful vars ------------------------------------------------------------------------------

void printValues(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
// it's just need to be here, it required by this mpu.on_FIFO(printValues), later i will fix this
}

// compute vertical acceleration ------------------------------------------------------------

void compute(int16_t *imuAccel, int32_t *imuQuat, double* vertVector, double& vertAccel) {
    double accel[3]; 
    double quat[4];

int64_t calibratedAccel;
  calibratedAccel = (int64_t)imuAccel[0] << VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER;
  calibratedAccel -= 0;
  calibratedAccel *= (0 + ((int64_t)1 << VERTACCEL_CAL_SCALE_MULTIPLIER));
  accel[0] = ((double)calibratedAccel)/((double)((int64_t)1 << (VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER + VERTACCEL_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT)));

  calibratedAccel = (int64_t)imuAccel[1] << VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER;
  calibratedAccel -= 0;
  calibratedAccel *= (0 + ((int64_t)1 << VERTACCEL_CAL_SCALE_MULTIPLIER));
  accel[1] = ((double)calibratedAccel)/((double)((int64_t)1 << (VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER + VERTACCEL_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT)));

  calibratedAccel = (int64_t)imuAccel[2] << VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER;
  calibratedAccel -= 0;
  calibratedAccel *= (0 + ((int64_t)1 << VERTACCEL_CAL_SCALE_MULTIPLIER));
  accel[2] = ((double)calibratedAccel)/((double)((int64_t)1 << (VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER + VERTACCEL_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT)));

  for(int i = 0; i<4; i++)
    quat[i] = ((double)imuQuat[i])/LIGHT_INVENSENSE_QUAT_SCALE;



  vertVector[0] = 2*(quat[1]*quat[3]-quat[0]*quat[2]);
  vertVector[1] = 2*(quat[2]*quat[3]+quat[0]*quat[1]);
  vertVector[2] = 2*(quat[0]*quat[0]+quat[3]*quat[3])-1;


  double ra[3];
  for(int i = 0; i<3; i++) 
    ra[i] = accel[i] - vertVector[i];
  
  /* compute vertical acceleration */
  vertAccel = (vertVector[0]*ra[0] + vertVector[1]*ra[1] + vertVector[2]*ra[2]) * VERTACCEL_G_TO_MS;



}

// compute vertical acceleration ------------------------------------------------------------




void setup()
{  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

   



	bmp.begin();
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_200_HZ);

     kalmanvert.init(bmp.readAltitude(SEALEVELPRESSURE_HPA),
                0.0,
                0.1,
                0.3,
                millis());

    mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS);
    mpu.on_FIFO(printValues);

    myGLCD.InitLCD();
    myGLCD.setFont(MediumNumbers);
    delay(1000); // wait for display
    startAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);   // saving start altitude 

}

void loop()
{       
// reading and filtering data --------------------------------------------------------------
    mpu.dmp_read_fifo();// Must be in loop
    double vertAccel;
    double tmpVertVector[3];

    compute(mpu.accel, mpu.quat, tmpVertVector, vertAccel);

    kalmanvert.update( bmp.readAltitude(SEALEVELPRESSURE_HPA),
                       vertAccel,
                       millis() );
	float vario = kalmanvert.getVelocity();
    vario = constrain(vario, -10, 10);

// reading and filtering data ---------------------------------------------------------------



      

// audio section ------------------------------------------------------------------------------

    if ((millis() - time) > BEEP_DURATION - vario * BEEP_DURATION_COEF && !fl) {
        fl = !fl;
        noToneAC();
    }
    if ((vario > UP_MIN_VARIO_SOUND_THRESHOLD) && (vario < UP_MAX_VARIO_SOUND_THRESHOLD)) {  
        if (fl && ((millis() - time) > BEEP_DURATION_X2 - vario * BEEP_DURATION_COEF_X2)) {
            time = millis();
            toneAC((INITIAL_FREQ_UP + vario * FREQ_COEF));
            fl = !fl;
        }
    }  else if (vario < DOWN_VARIO_SOUND_THRESHHOLD) {
        toneAC(FREQ_DOWN);
    } else {
        noToneAC();
    }

// audio section ------------------------------------------------------------------------------


}

