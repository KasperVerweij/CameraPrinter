import serial
from PIL import Image,ImageOps
import RPi.GPIO as GPIO
import picamera
import numpy as np
import os

BUTTON1_PIN = 23
BUTTON2_PIN = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON1_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(BUTTON2_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(25,GPIO.OUT)

camera = picamera.PiCamera()
camera.exposure_mode = 'auto'
camera.awb_mode = 'auto'


def button1_Click():
    try:
        GPIO.output(25,GPIO.HIGH)
        print("Opening serial port...")
        serialPort = serial.Serial('/dev/serial0', 9600, timeout=1)  # Internal serial port of the Raspberry Pi with timeout
        print("Serial port opened successfully.")

        if serialPort.is_open:
            print("Serial port is open.")
            print("Closing serial port...")
            serialPort.close()
            print("Serial port closed.")

        print("Opening serial port...")
        serialPort.open()
        print("Serial port opened successfully.")

        textToPrint = "your_text_here"  # Replace with the text you want to print

        newLineCommand = bytes([0x0A])  # New line

        serialPort.write(newLineCommand)
        print("New line command sent.")


        speedCommand = bytes([0x1B,0x23,0x35,0xFF])
        
        serialPort.write(speedCommand)
        image_path = "/home/pi/Desktop/picta.jpg"  # Path to the image on the desktop
        if os.path.exists(image_path):
            print("Image file found.")
            image = Image.open(image_path)
            width, height = image.size
            width = 48
            height = 8 * 48
            image = image.resize((width * 8, height))
            image = applyFloydSteinbergDithering(image)
            image = ImageOps.invert(image)

            startPageModeCommand = bytes([0x1B, 0x4C])
            serialPort.write(startPageModeCommand)
            print("Page mode command sent.")

            command = bytes([
                0x1D, 0x76, 0x30, 0,  # GS ( v 0
                width & 0xFF, (width >> 8) & 0xFF,  # Image width
                height & 0xFF, (height >> 8) & 0xFF  # Image height
            ])
            serialPort.write(command)
            print("Image size command sent.")

            printAndFeedCommand = bytes([0x0A])  # LF: Print and Feed Paper
            serialPort.write(printAndFeedCommand)
            print("Print and feed command sent.")

            image_data = np.array(image.convert("L")).flatten()
            image_data = np.packbits(image_data)

            serialPort.write(image_data)
            print("Image data sent.")

            serialPort.write(newLineCommand)
            serialPort.write(printAndFeedCommand)
            serialPort.write(printAndFeedCommand)
            serialPort.write(printAndFeedCommand)
            print("New line command sent.")

            print("Closing serial port...")
            serialPort.close()
            print("Serial port closed.")
            GPIO,output(25,GPIO.LOW)
        else:
            print("Error: Image file not found.")
    except Exception as ex:
        print("Error:", ex)
# @jit(nopython=True)
def applyFloydSteinbergDithering(input_image):
    output_image = input_image.convert("L")
    pixels = output_image.load()
    width, height = output_image.size

    for y in range(height):
        for x in range(width):
            old_pixel = pixels[x, y]
            new_pixel = 255 if old_pixel > 128 else 0
            
            pixels[x, y] = new_pixel

            error = old_pixel - new_pixel

            if x + 1 < width:
                pixels[x + 1, y] += int(error * 7 / 16)
            if x - 1 >= 0 and y + 1 < height:
                pixels[x - 1, y + 1] += int(error * 3 / 16)
            if y + 1 < height:
                pixels[x, y + 1] += int(error * 5 / 16)
            if x + 1 < width and y + 1 < height:
                pixels[x + 1, y + 1] += int(error * 1 / 16)

    return output_image

# Usage
try:
    while True:
        if not GPIO.input(BUTTON1_PIN):
            button1_Click()
        if not GPIO.input(BUTTON2_PIN):
            print("picture picture")
            camera.capture('/home/pi/Desktop/picta.jpg')
            
except KeyboardInterrupt:
    pass
GPIO.cleanup()
camera.close()
#button1_Click()
