import serial
import threading
import time


PORT = "COM7"
BAUD = 115200


ser = serial.Serial(
    PORT,
    BAUD,
    timeout=0.1
)


latest_distance = -1



def read_serial():

    global latest_distance


    while True:

        try:

            if ser.in_waiting:


                line = ser.readline().decode(errors="ignore").strip()


                print(line)


                if line.startswith("DIST:"):

                    value = line.split(":")[1]

                    latest_distance = int(value)



        except Exception:
            pass


        time.sleep(0.01)



thread = threading.Thread(
    target=read_serial,
    daemon=True
)

thread.start()



def get_distance():

    return latest_distance



def send_command(command):

    try:

        ser.write(
            (command+"\n").encode()
        )


        print("Sending command:",command)


    except Exception as e:

        print(e)