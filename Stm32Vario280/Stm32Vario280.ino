// includes section ------------------------------------------------------------------------------
	#include <Tone.h> // tone lib for stm32
	#include <LCD5110_Graph.h> // display Nokia 5110
	#include <Adafruit_Sensor.h> // need for Adafruit_BMP280
	#include <Adafruit_BMP280.h> // sensor pressure lib
// includes section ------------------------------------------------------------------------------

// lib objects section ----------------------------------------------------------------------------
	// SCK  - Pin 8, MOSI - Pin 6, DC   - Pin 7, RST  - Pin 11, CS   - Pin 12 -> display pins
	LCD5110 myGLCD(PB8,PB9,PB10,PB11,PB12); // dispaly
	Adafruit_BMP280 bme; // pressure sensor

// lib objects section ----------------------------------------------------------------------------

// defs section ------------------------------------------------------------------------------

    #define SEALEVELPRESSURE_HPA (1022.65) 
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

// filtering settings section ------------------------------------------------------------------------------


	float alt[101];
	float tim[101];
	float Altitude;
	int samples = 80;
	int maxsamples = 100;

// filtering settings section ------------------------------------------------------------------------------


//useful vars ------------------------------------------------------------------------------


	boolean  fl = true; // flag for determine beep or not to beep
	extern uint8_t SmallFont[]; // fonts for dsiplay
	extern uint8_t MediumNumbers[]; // fonts for dsiplay
	int val = 0; // for display
	int p = PB14; // pin for + or S on beeper
	int flyAlt = 0;
	int startAlt = 0;
	int curAlt = 0;

//useful vars ------------------------------------------------------------------------------


void setup() {
	// init 
	bme.begin()
	myGLCD.InitLCD();
	myGLCD.setFont(MediumNumbers);
	delay(1000); // wait for display
	startAlt = bme.readAltitude(SEALEVELPRESSURE_HPA); // saving start altitude
}

void loop() {



// filtering section ------------------------------------------------------------------------------

	float tempo = millis();
	float vario = 0;
	float N1 = 0;
	float N2 = 0;
	float N3 = 0;
	float D1 = 0;
	float D2 = 0;
	Altitude = bme.readAltitude(1022.65);
	for (int cc = 1; cc <= maxsamples; cc++) {                           // усреднитель
	alt[(cc - 1)] = alt[cc];
	tim[(cc - 1)] = tim[cc];
	}
	alt[maxsamples] = Altitude;
	tim[maxsamples] = tempo;
	float stime = tim[maxsamples - samples];
	for (int cc = (maxsamples - samples); cc < maxsamples; cc++) {
	N1 += (tim[cc] - stime) * alt[cc];
	N2 += (tim[cc] - stime);
	N3 += (alt[cc]);
	D1 += (tim[cc] - stime) * (tim[cc] - stime);
	D2 += (tim[cc] - stime);
	}
	vario = 1000 * ((samples * N1) - N2 * N3) / (samples * D1 - D2 * D2);
	vario = constrain(vario, -10, 10);
// filtering section ------------------------------------------------------------------------------


// audio section ------------------------------------------------------------------------------

    if ((millis() - time) > BEEP_DURATION - vario * BEEP_DURATION_COEF && !fl) {
        fl = !fl;
        noTone(p);
    }
    if ((vario > UP_MIN_VARIO_SOUND_THRESHOLD) && (vario < UP_MAX_VARIO_SOUND_THRESHOLD)) {  
        if (fl && ((millis() - time) > BEEP_DURATION_X2 - vario * BEEP_DURATION_COEF_X2)) {
            time = millis();
            tone(p, (INITIAL_FREQ_UP + vario * FREQ_COEF));
            fl = !fl;
        }
    }  else if (vario < DOWN_VARIO_SOUND_THRESHHOLD) {
        tone(p, FREQ_DOWN);
    } else {
        noTone(p);
    }

// audio section ------------------------------------------------------------------------------

// display section ------------------------------------------------------------------------------
	val = vario * 10;
	val = constrain(val, -25, 25);
	val = map(val, -25, 25, 40, 0);
	curAlt = bme.readAltitude(1022.65);
	flyAlt = curAlt - startAlt;
	myGLCD.clrScr();
	myGLCD.setFont(SmallFont);
	myGLCD.print("var: ", 0, 3);
	myGLCD.print("alt: ", 0, 40);
	myGLCD.drawLine(0, 20, 10, 20);
	//myGLCD.drawLine(42, 20, 15, val);
	myGLCD.fillTriangle(42, 20, 15, 20, 15, val);
	myGLCD.setFont(MediumNumbers);
	myGLCD.printNumF(vario, 1, 35, 2, '.', 4, '/');
	myGLCD.printNumI(flyAlt, 40, 30);
	myGLCD.update();
// display section ------------------------------------------------------------------------------
}
