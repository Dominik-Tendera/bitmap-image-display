import serial
from PIL import Image
import time

def send_rgb_over_uart(port, filename):
    ser = serial.Serial(port, baudrate=115200, timeout=1000)

    # return
    try:
        with Image.open(filename) as img:
            rgb_data = list(img.getdata())
            for rgb in rgb_data: 
                r, g, b = rgb
                ser.write(r.to_bytes(1))
                ser.write(g.to_bytes(1))
                ser.write(b.to_bytes(1))
            
        print("RGB values sent over UART successfully.")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        ser.close()

if __name__ == "__main__":

    uart_port = 'COM5'                  #CHANGE FOR YOU SERIAL PORT NUMBER 

def przedstawienie():
    print("Przedstawienie:\n")
    counter = 0                         #przemiatanie wieloma obrazkami
    while(counter < 20):
        bmp_filename = 'spaceinvader.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(1.6)
        bmp_filename = 'ghost.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(1.6)
        bmp_filename = 'kirby.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(1.6)
        bmp_filename = 'ghost.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(1.6)
        bmp_filename = 'muchomorek.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(1.6)
        bmp_filename = 'essa.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(1.6)
        bmp_filename = 'nohej.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(1.6)
        bmp_filename = 'smiech.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(1.6)
        bmp_filename = 'sinus.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(1.6)
        bmp_filename = 'testimage.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(1.6)
        bmp_filename = 'pacman1.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(1.6)
        counter += 1
    
def pacman():
    print("PacMan:\n")
    counter = 0                         #PACMAN GIF
    while(counter < 100):
        bmp_filename = 'pacman1.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(0.1)
        bmp_filename = 'pacman2.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(0.1)
        bmp_filename = 'pacman3.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(0.1)
        bmp_filename = 'pacman4.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(0.2)
        bmp_filename = 'pacman3.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(0.1)
        bmp_filename = 'pacman2.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(0.1)
        bmp_filename = 'pacman1.bmp'
        send_rgb_over_uart(uart_port, bmp_filename)
        time.sleep(0.1)
        counter += 1
    
def sinus():
    print("Sinusoida:\n")
    bmp_filename = 'sinus.bmp'
    send_rgb_over_uart(uart_port, bmp_filename)

def test():
    print("Test:\n")
    bmp_filename = 'test.bmp'
    send_rgb_over_uart(uart_port, bmp_filename)


while True:
    print("Menu:\n1. Sinusoida.\n2. PacMan\n3. Przedstawienie\n4. Test\n Cokolwiek innego = koniec\n")
    wybor = input("Co chcesz zrobić? numerek: ")
    if (wybor == '1'):
        sinus()
    elif (wybor == '2'):
        pacman()
    elif (wybor == '3'):
        przedstawienie()
    elif (wybor == '4'):
        test()
    else:
        break
    