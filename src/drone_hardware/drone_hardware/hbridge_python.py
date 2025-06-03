# H-Bridge Motor Control with Raspberry Pi


# This script controls a DC motor using an H-Bridge driver (Pololu MD08A) with a Raspberry Pi.





import RPi.GPIO as GPIO


import time





# UNumeration GPIO (BCM)


GPIO.setmode(GPIO.BCM)





# Define pin numbers


IN1 = 17


IN2 = 27


PWM_PIN = 22





# Pin config


GPIO.setup(IN1, GPIO.OUT)


GPIO.setup(IN2, GPIO.OUT)


GPIO.setup(PWM_PIN, GPIO.OUT)





# Inicialization PWM (20 kHz)


pwm = GPIO.PWM(PWM_PIN, 20000)


pwm.start(0)





try:


    while True:


        print("\n--- Motor Control ---")


        print("f - fornt, b - back, l - front (50% PWM), r - back (50% PWM), s - stop, q - quick")


        kierunek = input("Direction: ").lower()





        if kierunek == 'q':


            print("Ending..")


            break





        elif kierunek == 's':


            GPIO.output(IN1, GPIO.LOW)


            GPIO.output(IN2, GPIO.LOW)


            pwm.ChangeDutyCycle(0)


            print("SMotor stoping.")


            continue





        elif kierunek in ['f', 'b']:


            wypelnienie = input("Choice PWM (0-100): ")


            try:


                duty = float(wypelnienie)


                if not 0 <= duty <= 100:


                    print(" 0% - 100%")


                    continue


            except ValueError:


                print("Error")


                continue





        elif kierunek in ['l', 'r']:


            duty = 50


            print("PWM set on 50% for direct")


        else:


            print("Error")


            continue





        # Ustawienie kierunku


        if kierunek in ['f', 'r']:


            GPIO.output(IN1, GPIO.HIGH)


            GPIO.output(IN2, GPIO.LOW)


            print("Motor spining on: RIGHT" if kierunek == 'r' else "Motor spining on: RIGHT")


        elif kierunek in ['b', 'l']:


            GPIO.output(IN1, GPIO.LOW)


            GPIO.output(IN2, GPIO.HIGH)


            print("Motor spining on: LEFT" if kierunek == 'l' else "Motor spining on: LEFT")





        # Ustaw prędkość


        pwm.ChangeDutyCycle(duty)





except KeyboardInterrupt:


    print("\nStopped by user")





finally:


    pwm.stop()


    GPIO.cleanup()


    print("GPIO cleaned up, ending program.")





#######################################################################


# Instructions:


# Start program and choose direction:


# f - forward           [Enter the desired PWM value (0-100) when prompted]


# b - backward          [Enter the desired PWM value (0-100) when prompted]


# l - left (50% PWM)


# r - right (50% PWM)


# s - stop


# q - quick (exit)  


# 


# MADE ON RASPBERRY PI 4B !!!!


# Make sure to connect the H-Bridge to the Raspberry Pi GPIO pins as follows:   





# | Raspberry Pi GPIO | Pin fizyczny | Pololu MD08A pin | Opis                  |


# | ----------------- | ------------ | ---------------- | --------------------- |


# | GPIO17            | 11           | IN1              | Kierunek 1            |


# | GPIO27            | 13           | IN2              | Kierunek 2            |


# | GPIO22            | 15           | PWM              | Sterowanie prędkością |


# | GND               | 6            | GND              | Masa wspólna          |





# | Źródło zasilania | Pololu MD08A | Opis                           |


# | ---------------- | ------------ | ------------------------------ |


# | +12V z zasilacza | VM           | Zasilanie silnika (np. 12V DC) |


# | Masa zasilacza   | GND          | Wspólna masa z RPi             |





# | Pololu MD08A pin | Do czego                    |


# | ---------------- | --------------------------- |


# | OUT1             | Do jednego pinu silnika DC  |


# | OUT2             | Do drugiego pinu silnika DC |