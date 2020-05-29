# OVERDRIVE

Teensy 4.0 RC/LED Controller

![](./teensy40_overdrive_beta1/wire1.jpg)

## Requirement for rc car autonomous driving
* Teensy 4.0
* 4ch or 3ch rc receiver/transmitter (test with Futaba R334SBS-E/Futaba 7PX)
* Jetson Nano (I don't test on raspberry pi3/4.)

## Requirement for rc car led decolation
* Teensy 4.0
* 4ch or 3ch rc receiver/transmitter
* USB mobile battery
* 5V 20mA single connected LEDs. (for beta1)

## Youtube
[![run](https://img.youtube.com/vi/BgRjPW4X-rY/default.jpg)](https://www.youtube.com/watch?v=BgRjPW4X-rY)

## Known issue
* sometime i2c error occures with PCA9685 emulator.<br>
* Some motors seem to be affected by noise. (maybe tired motor)<br>
  * If you feels bad signal with throttle on, try polishing the commutator and brush of the motor.<br>

## Teensy Setup
[Arduino IDE](https://www.arduino.cc/en/main/software)<br>
[Teensyduino](https://www.pjrc.com/teensy/td_download.html)<br>
[teensy4_i2c](https://github.com/Richard-Gemmell/teensy4_i2c)<br>
* Install Arduino IDE.
* Install Teensyduino.
* Install teensy4_i2c.
* Setup Arduino IDE

### Install Arduino IDE.
Download ARDUINO 1.8.12 Windows ZIP file for non admin install.<br>
unzip it.<br>

### Install Teensyduino.
Download Teensyduino 1.51 Windows XP/7/8/10 Installer.<br>
execute and install.<br>

### Install teensy4_i2c.
git clone and copy the directory into arduino libraries.<br>
```
git clone https://github.com/Richard-Gemmell/teensy4_i2c
cp -r teensy4_i2c arduino/hardware/teensy/avr/libraries/
```

### Setup Arduino IDE
* Tools
Board: "Teensy 4.0"<br>
USB Type: "Serial + Keyboard + Mouse + Joystick"<br>
CPU Speed: "600 MHz"<br>
Optimize: "Faster"<br>
![](./teensyduino.png)

## DonkeyCar Setup
* donkeycar 3.1.2 overdrive for rc
* donkeycar 3.1.2 overdrive simulator
* donkeycar 3.1.1

### donkeycar 3.1.2 overdrive for rc
on training PC:
```
cd ~/project
git clone https://github.com/naisy/donkeycar
cd donkeycar
git checkout overdrive
pip install -e .[pc]
donkey createcar ~/mycar
```
on Jetson Nano:
```
cd ~/project
git clone https://github.com/naisy/donkeycar
cd donkeycar
git checkout overdrive
pip install -e .[nano]
donkey createcar ~/mycar
```
and use myconfig.py
```
cd ~/project
git clone https://github.com/naisy/overdrive
cp overdrive/donkeycar_overdrive/cfg* ~/mycar
```
Normal recording.
```
python manage.py drive --js --myconfig=cfg_manual.py
```
Normal autonomous driving.
```
python manage.py dirve --model=mylinear.h5 --myconfig=cfg_auto.py
```
Assist recording.<br>
Assist is an experimental feature.<br>
Create and use speed30 model.<br>
Steering: user steering.<br>
Throttle: user throttle + ai throttle.<br>
```
python manage.py drive --js --model=mylinear.h5 --myconfig=cfg_assist.py
```
Assist data generate.
```
wget https://raw.githubusercontent.com/naisy/donkeycar_tools/master/make_ai_to_train_data.py
python make_ai_to_train_data.py
mv data data_assist_org
mv data_ai data
python train.py --model=mylinear2.h5 --type=linear
```

### donkeycar 3.1.2 overdrive simulator
Requirements: Experience installing donkeycar simulator.<br>
donkeycar_sim-racer directory in this repo will be delete. Because I created overdrive branch in my forked donkeycar repo.

```
cd ~/project
git clone https://github.com/naisy/donkeycar
cd donkeycar
git checkout overdrive
pip install -e .[pc]
donkey createcar ~/sim_racer
```
and use myconfig.py
```
cd ~/project
git clone https://github.com/naisy/overdrive
cp overdrive/donkeycar_overdrive/cfg* ~/sim_racer
```
Normal recording.
```
python manage.py drive --js --myconfig=cfg_manual_server.py
```
Normal autonomous driving.
```
python manage.py dirve --model=myrnn.h5 --myconfig=cfg_auto_server.py
```
Assist recording.
```
python manage.py drive --js --model=myrnn.h5 --myconfig=cfg_assist_server.py
```
Assist data generate.
```
wget https://raw.githubusercontent.com/naisy/donkeycar_tools/master/make_ai_to_train_data.py
python make_ai_to_train_data.py
mv data data_assist_org
mv data_ai data
python train.py --model=myrnn2.h5 --type=rnn
```

### donkeycar 3.1.1
Use donkecar 3.1.2 overdrive for rc.

This is for donkeycar 3.1.1
```
cp donkeycar311/*.py ~/project/donkeycar/donkeycar/parts/
cp donkeycar311/myconfig.py.nano_120fps ~/mycar/myconfig.py
```

## 4ch Transmitter Settings
Steering and throttle: These are normal rc car setting.<br>
* ch1
  * steering
* ch2
  * throttle
* ch3
  * manual - auto mode change.
* ch4
  * delete record.

Futaba 7PX<br>
![](./transmitter.jpg)<br>
![](./transmitter_manual.jpg)<br>
![](./transmitter_auto.jpg)<br>

## 3ch Transmitter Settings
Steering and throttle: These are normal rc car setting.<br>
* ch1
  * steering
* ch2
  * throttle
* ch3
  * manual - auto mode change.

Tamiya TTU-08 (FINESPEC 2.4G)<br>
![](./transmitter_3ch.jpg)<br>
## 3ch Wireing
![](./transmitter_3ch_wire1.jpg)<br>

## Record training data
* Transmitter ch3 auto mode.
* Transmitter ch3 manual mode. (change from auto to manual mode is one of flags)
* Transmitter ch2 throttle on. (start recording)
* Transmitter ch3 auto mode. (stop recording)
* Transmitter ch4 delete records. (delete 120 records.)

## Autonomous driving
* Transmitter ch3 auto mode.<br>
If you think your rc car will crash, apply the brakes immediately. Overdrive gives priority to manual operation.<br>
One second after you release your hand, it switches to automatic mode.<br>

## Teensy 4.0 OVERDRIVE beta1
See [Teensy 4.0 OVERDRIVE beta1](./README_teensy40_overdrive_beta1.md)<br>

## ROADMAP
### Teensy 4.0 OVERDRIVE beta2 (in 2020)
* Breadboard
  * LED update. Add more LEDs.
* I2C
  * Add new virtual I2C device for RC control instead of PCA9685 emulator. It will be optimized for the RC setting more than PCA9685.
* Support high speed digital servo. (Manual mode in beta1 is already optimized. But PCA9685 mode is not optimized yet.)
