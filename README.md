# variometer
various version of variometers


ms5611_accel_vario
  Variometer on arduino using ms5611 pressure sensor, MPU6050 accelerometer and Nokia 5110 display. Vertical velocity computing by kalman sensor fusion. Very accurate and fast vario, current best version. Based on GNUVario: https://prunkdump.github.io/GNUVario/
I did some changes, for personal use. If you want to use it just upload sketch and connect all things by default. Beeper connects to 9 and 10 pins.

388_vario
  Variometer on arduino using BMP388 and Nokia 5110 display. Simple filter. Not so fast like previous but still working.
connections are the same.


Stm32Vario280
  Variometer on stm32 board "blue pill" using BMP280 and Nokia 5110 display. Using stm32duino bootloader. Same code, different beeper lib, pin for beeper 2.


vario_388_accel
  Same vario with kalman sensor fusion, different pressure sensor bmp 388 and different mpu6050 lib. But can't work with display, too small memory size in arduino, so just beeper. Maby later will be version with lighter libs or for stm32. 
  
  vario388stm32 
    Same as  388_vario, but faster and different beeper lib, pin for beeper 2. Board "blue pill". Using stm32duino bootloader.
