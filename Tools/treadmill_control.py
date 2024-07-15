from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.by import By
from tkinter import Tk, Frame, Button, Label, Scale, Spinbox, Entry, IntVar
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np


def send_to_treadmill(parameter, value):
    """
    Send command to the treadmill with the web page completed by Selenium.
    """
    # Set Chrome options for headless mode
    chrome_options = Options()
    chrome_options.add_argument('--headless')  # Run Chrome in headless mode

    # Set up the WebDriver with the specified Chrome options
    driver = webdriver.Chrome(options=chrome_options)

    try:
        # Open the target webpage
        driver.get('http://192.168.124.16/GSP.htm')  # Replace with the actual URL of the page

        # Locate the D900 input element and set its value
        d900_input = driver.find_element(By.NAME, 'D900')
        d900_input.clear()  # Clear existing text
        d900_input.send_keys(str(parameter))  # Enter the new value

        # Locate the D901 input element and set its value
        d901_input = driver.find_element(By.NAME, 'D901')
        d901_input.clear()  # Clear existing text
        d901_input.send_keys(str(value))  # Enter the new value

        # Locate the "Write" button and click it
        write_button = driver.find_element(By.XPATH, '//input[@type="submit" and @value="Write"]')
        write_button.click()
        print('Send {} {}'.format(parameter,value))

    finally:
        # Close the WebDriver
        driver.quit()

def update_speed(val):
    send_to_treadmill(61, val)
    label_speed.config(text="Speed : {:.2f} m/s".format(int(val)*0.00541))
    label_real_speed.config(text="Equivalent real speed : {:.2f} km/h".format(int(val)*0.00541*18*3.6))
    if int(val)==0:
        send_to_treadmill(65,0)

def start():
    send_to_treadmill(65, 97)
    send_to_treadmill(61, int(scale.get()))

def create_window():
    """
    Tkinter window with buttons to start/stop the treadmill, a scale to set the speed, and entries to set other parameters.
    """
    global label_speed, scale, speed_entry,label_real_speed  # Declare as global to update in other functions
    root = Tk()
    root.title("ADAS on Treadmill : Control the treadmill")
    root.option_add("*Font", "Arial 16")

    # Frame for Start/Stop buttons
    frame_buttons = Frame(root, padx=20, pady=20)
    frame_buttons.pack()

    start_button = Button(frame_buttons, text="Start", command=start)
    start_button.grid(column=0, row=0, padx=10)

    stop_button = Button(frame_buttons, text="Stop", command=lambda: send_to_treadmill(65, 0))
    stop_button.grid(column=1, row=0, padx=10)

    # Frame for Scale
    frame_scale = Frame(root, padx=20, pady=20)
    frame_scale.pack()

    label_speed = Label(frame_scale, text="Speed : {:.2f} m/s".format(0))
    label_speed.grid(column=0, row=0, columnspan=2, pady=10)

    label_real_speed = Label(frame_scale, text="Equivalent real speed : {:.2f} km/h".format(0))
    label_real_speed.grid(column=0, row=1, columnspan=2, pady=10)

    scale = Scale(frame_scale, from_=0, to=600, orient='horizontal', command=update_speed, length=400)
    scale.grid(column=0, row=2, columnspan=2, pady=10)

    # Entry for Speed
    label_speed_entry = Label(frame_scale, text="Enter speed (m/s):", justify='center')
    label_speed_entry.grid(column=0, row=3, pady=5)

    speed_entry = Entry(frame_scale, width=10, justify='center')
    speed_entry.grid(column=1, row=3, pady=5)

    button_set_speed = Button(frame_scale, text="Set speed", command=lambda: set_speed(speed_entry.get()))
    button_set_speed.grid(column=0, row=4, columnspan=2, pady=10)

    # Frame for Other Parameters
    frame_parameters = Frame(root, padx=20, pady=20)
    frame_parameters.pack()

    label_parameters = Label(frame_parameters, text="Other parameters:")
    label_parameters.grid(column=0, row=0, columnspan=2, pady=10)

    label_number = Label(frame_parameters, text="Number:", justify='center')
    label_number.grid(column=0, row=1, pady=5)

    Xinput = Spinbox(frame_parameters, from_=0, to=1000, width=5, justify='center', textvariable=IntVar(value=0))
    Xinput.grid(column=0, row=2, pady=5)

    label_value = Label(frame_parameters, text="Value:", width=5, justify='center')
    label_value.grid(column=1, row=1, pady=5)

    Yinput = Spinbox(frame_parameters, from_=0, to=1000, width=5, justify='center', textvariable=IntVar(value=0))
    Yinput.grid(column=1, row=2, pady=5)

    button_set_input = Button(frame_parameters, text="Set input", command=lambda: send_to_treadmill(Xinput.get(), Yinput.get()))
    button_set_input.grid(column=0, row=3, columnspan=2, pady=10)

    set_speed(1)

    root.mainloop()

def set_speed(speed_str):
    try:
        speed = float(speed_str)
        # Update label or send speed to treadmill
        val=int(speed/0.00541)
        if val > 600:
            val = 600
            speed_entry.delete(0, 'end')  # Efface le contenu actuel de l'Entry
            speed_entry.insert(0, "{:.2f}".format(val * 0.00541))  # Insère la nouvelle valeur dans l'Entry

        scale.set(val)
    except ValueError:
        print("Error", "Please enter a valid speed (a number).")

create_window()


