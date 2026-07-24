import cv2

from serial_comm import (
    get_distance,
    send_command
)



# Laptop camera

cap = cv2.VideoCapture(0)



if not cap.isOpened():

    print("Camera not found")
    exit()



last_command = ""



while True:


    ret, frame = cap.read()


    if not ret:
        break



    distance = get_distance()



    print("Distance:", distance)



    # Decision logic

    if distance != -1 and distance < 50:


        command = "CENTER"


    else:

        command = "NONE"



    if command != last_command:

        send_command(command)

        last_command = command



    cv2.putText(
        frame,
        f"Distance: {distance} cm",
        (20,40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0,255,0),
        2
    )


    cv2.imshow(
        "EchoPath",
        frame
    )


    key=cv2.waitKey(1)&0xff


    if key==27:
        break



    if cv2.getWindowProperty(
        "EchoPath",
        cv2.WND_PROP_VISIBLE
    ) < 1:
        break



cap.release()

cv2.destroyAllWindows()