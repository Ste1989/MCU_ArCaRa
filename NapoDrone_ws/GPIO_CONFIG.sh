#!/bin/bash
echo "configurazione GPIO pin gpio 37 (26)"
sudo chmod 777 /sys/class/gpio/export
sudo chmod 777 /sys/class/gpio/unexport

sleep 3
echo "configuro gpio26"
sudo echo 26 > /sys/class/gpio/export

sleep 3
echo "abilito accessi gpio26"
sudo chmod 777 /sys/class/gpio/gpio26/value 
sudo chmod 777 /sys/class/gpio/gpio26/direction


echo "configuro PWM (pin 32)"
sudo chmod 777 /sys/class/pwm/pwmchip0/export
sudo chmod 777 /sys/class/pwm/pwmchip0/unexport

sleep 3
echo "configuro pwm0"
sudo echo 0 > /sys/class/pwm/pwmchip0/export

sleep 3
echo "abilito accessi pwm0"
sudo chmod 777 /sys/class/pwm/pwmchip0/pwm0/enable
sudo chmod 777 /sys/class/pwm/pwmchip0/pwm0/duty_cycle
sudo chmod 777 /sys/class/pwm/pwmchip0/pwm0/period









