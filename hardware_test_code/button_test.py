from gpiozero import Button
import time

button = Button(2)
# button.wait_for_press()
# print('You pushed me')

# print on the first button press, loop until it is pressed again
button.wait_for_press()
print('You pushed me')
time.sleep(1.0)  # debounce
while True:
    if button.is_pressed:
        print('You pushed me again')
        exit()
    else:
        print('You did not')
    time.sleep(0.1)

