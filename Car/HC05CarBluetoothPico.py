from machine import Pin, PWM, UART
import time

# Configuration des broches
motor_speed_pin = PWM(Pin(20))
servo_pin = PWM(Pin(21))

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1), timeout= 100)

# Initialisation des variables
speed = 0
angle = 100
last_angle = 0

# Configurer le PWM pour le moteur
motor_speed_pin.freq(1000)

# Configurer le PWM pour le servo
servo_pin.freq(50)

# Fonction pour convertir un angle en duty cycle pour le servo
def angle_to_duty(angle):
    return int((angle/180 * 0.05 + 0.05)* 65535)


# Boucle principale
while True:
    if uart.any():
        received_message = uart.readline().decode('utf-8').rstrip("\n")
        speed_str, angle_str = received_message.split(',')
    
    
        try:
            new_speed = int(speed_str)
            new_angle = int(angle_str)

            if 0 <= new_speed <= 150:
                speed = new_speed
                        
            if 40 <= new_angle <= 160:
                angle = new_angle
                print(angle)
                
                        
                        
        except ValueError:
            pass

    if last_angle != angle:
        servo_duty = angle_to_duty(angle)
        servo_pin.duty_u16(servo_duty)
        last_angle = angle

    motor_speed_pin.duty_u16(int((speed / 255) * 65535))

    time.sleep(0.01)
    speed = 0
    angle = 100
