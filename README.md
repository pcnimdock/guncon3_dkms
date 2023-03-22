# GunCon 3 USB Lightgun Driver
A DKMS driver for GunCon3 compatible with Ubuntu

It's a modification beardypig and redemp Guncon2 Driver

Can calibrate 2 GunCon3 with evdev, xinput method for joystick mode and xinput for mouse mode.

In mouse mode, detects the 2 ligthguns but I think it's a caos. I prefer joystick mode

## rules.d
Add 99-guncon3.rules to /dev/udev/rules.d

For mouse mode modify ENV{ID_INPUT_MOUSE}="1"

For joystick mode ENV{ID_INPUT_MOUSE}="0"

## Calibration

Execute ./calibrate.sh


### Build and install

```shell
sudo apt install dkms
sudo ./install.sh
sudo modprobe guncon3
sudo cp ./calibration/99-guncon3.rules /dev/udev/rules.d
```
Python calibration script depends: pygame evdev wheel
