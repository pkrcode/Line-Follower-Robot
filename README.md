# Line Follower Robot using ATmega2560

This is a simple embedded systems project that implements a Line Follower Robot using the ATmega2560 microcontroller. The robot uses IR sensors to detect and follow a black line on a white surface. It is programmed in Embedded C and built with Atmel Studio.

---

## 🧾 Description

This project uses 3 IR sensors placed at the front of the robot to detect the position of the line. The logic in `main.c` determines the required motor actions based on the sensor inputs. A motor driver (like L298N) is used to control the motors.

An LCD display is optionally used to show system status.

---

## 📁 Project Structure

- `src/`: Contains main program (`main.c`) and LCD driver code (`lcd.c`, `lcd.h`)
- `programmer/AVRDUDE_ATMEGA2560/`: Contains compiled hex file and tools/scripts for flashing it onto ATmega2560
- `project/`: Contains the Atmel Studio `.cproj` project file

---

## ⚙️ How to Build

### Option 1: Atmel Studio
Open the `.cproj` file in `project/LineFollower.cproj` and click **Build**.

### Option 2: AVR-GCC (Linux)
If you're using `avr-gcc`, compile like this:
```bash
avr-gcc -mmcu=atmega2560 -Os -o main.elf src/main.c src/lcd.c
avr-objcopy -O ihex -R .eeprom main.elf main.hex
```

---

## 🔌 How to Upload

You can use the precompiled hex file `LineFollower.hex` found in `programmer/AVRDUDE_ATMEGA2560/`.

### AVRDUDE Command (with USBasp):
```bash
avrdude -C avrdude.conf -c usbasp -p m2560 -U flash:w:LineFollower.hex
```

Windows users can double-click `stkrun.bat` to upload directly.

---

## 🛠️ Hardware Required

- ATmega2560 microcontroller
- IR sensors (3)
- Motor driver module (L298N or similar)
- 2 DC motors
- Power supply (battery)
- Optional: LCD 16x2 display

---

## 📷 Optional Add-ons

You can extend this by adding:
- Speed control with PWM
- Curve handling
- Obstacle detection
- Proteus simulation files

---

## 👤 Author

**Praveen Kumar**  
B.Tech ECE, IIIT Senapati, Manipur  
GitHub: [pkrcode](https://github.com/pkrcode)  
LinkedIn: [praveenk-dev](https://linkedin.com/in/praveenk-dev)

---

## 🪪 License

This project is open-source. You can add a license like MIT, GPLv3, or Apache 2.0 if you wish.
