// on 2020.01.13 it's best version


// includes section ------------------------------------------------------------------------------
    #include <Arduino.h>
    #include <IntTW.h> // initialize i2c sensors
    #include <vertaccel.h> // compute vertical acceliration
    #include <LightInvensense.h> // for mpu
    #include <TwoWireScheduler.h> // fetching data from acelerometer and pressure sensors
    #include <kalmanvert.h> // kalman fusion filter
    #include <toneAC.h> // makes loud beeps
    #include <LCD5110_Graph.h> // display
// includes section ------------------------------------------------------------------------------

// defs section ------------------------------------------------------------------------------

    #define SEALEVELPRESSURE_HPA (1013.25) 
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

// lib objects section ------------------------------------------------------------------------------
    Vertaccel TWScheduler::vertaccel;
    Ms5611 TWScheduler::ms5611;
    kalmanvert kalmanvert;
    // SCK  - Pin 8, MOSI - Pin 6, DC   - Pin 7, RST  - Pin 11, CS   - Pin 12 -> display pins
    LCD5110 myGLCD(8, 6, 7, 11, 12);
// lib objects section ------------------------------------------------------------------------------



//useful vars ------------------------------------------------------------------------------


    boolean  fl = true; // flag for determine beep or not to beep
    extern uint8_t SmallFont[]; // fonts for diplay
    extern uint8_t MediumNumbers[]; // fonts for diplay
    int val = 0; // for display
    int flyAlt = 0;
    int startAlt = 0;
    int curAlt = 0;
    unsigned long time = 0; // for beeps duration


//useful vars ------------------------------------------------------------------------------

void setup()
{



    // init i2c
    intTW.begin();
    twScheduler.init();
    // init filtering
    kalmanvert.init(twScheduler.getAlti(),
                0.0,
                0.1,
                0.3,
                millis());

    // init display 
    myGLCD.InitLCD();
    myGLCD.setFont(MediumNumbers);
    delay(1000); // wait for display
    startAlt = twScheduler.getAlti(); // saving start altitude         
}

void loop()
{

    



// filtering section ------------------------------------------------------------------------------

    kalmanvert.update(twScheduler.getAlti(), twScheduler.getAccel(NULL), millis()); // reading new data

    float vario = kalmanvert.getVelocity(); // fetching filtered velocity
    vario = constrain(vario, -10, 10); // ready to use vertical velocity


// filtering section ------------------------------------------------------------------------------



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



// display section ------------------------------------------------------------------------------

    val = vario * 10;
    val = constrain(val, -25, 25);
    val = map(val, -25, 25, 40, 0); // scaling horizontal pointer
    curAlt = twScheduler.getAlti(); // current altitude above sea level
    flyAlt = curAlt - startAlt; // alltitude above start
    myGLCD.clrScr(); 
    myGLCD.setFont(SmallFont); 
    myGLCD.print("var: ", 0, 3);
    myGLCD.print("alt: ", 0, 40);
    myGLCD.drawLine(0, 20, 10, 20);
    myGLCD.fillTriangle(42, 20, 15, 20, 15, val);
    myGLCD.setFont(MediumNumbers);
    myGLCD.printNumF(vario, 1, 35, 2, '.', 4, '/');
    myGLCD.printNumI(flyAlt, 40, 30);
    myGLCD.update();


// display section ------------------------------------------------------------------------------




}
