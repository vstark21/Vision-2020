import cv2
import numpy as np
import serial as ser


def contourCenter(mask):
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    maxRadius = 0
    maxContourCenter = (0, 0)
    for contour in contours:
        contourCenter, radius = cv2.minEnclosingCircle(contour)
        if radius > maxRadius:
            maxRadius = radius
            maxContourCenter = contourCenter

    return maxContourCenter, maxRadius, len(contours)


cap = cv2.VideoCapture("http://192.168.137.42:4747/mjpegfeed")
print("Camera Connected")
serial = ser.Serial("COM3", baudrate=9600, timeout=0)
print("Serial Connected")
box = 0

while cap.isOpened():

    _, frame = cap.read()
    cv2.imshow("image", frame)
    frame = frame[25:, :]
    frame = np.rot90(frame)
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    sensor = serial.read()

    lowerBoundRed1 = np.array([0, 135, 70])
    upperBoundRed1 = np.array([20, 255, 255])
    maskOne = cv2.inRange(hsv, lowerBoundRed1, upperBoundRed1)

    lowerBoundRed2 = np.array([150, 120, 70])
    upperBoundRed2 = np.array([180, 255, 255])
    maskTwo = cv2.inRange(hsv, lowerBoundRed2, upperBoundRed2)

    maskRed = maskOne + maskTwo

    lowerBoundBlue = np.array([90, 130, 80])
    upperBoundBlue = np.array([130, 255, 255])
    maskBlue = cv2.inRange(hsv, lowerBoundBlue, upperBoundBlue)

    lowerBoundGreen = np.array([30, 200, 185])
    upperBoundGreen = np.array([[70, 255, 255]])
    maskGreen = cv2.inRange(hsv, lowerBoundGreen, upperBoundGreen)

    kernel = np.array((7, 7), dtype=int)

    resultRed = cv2.erode(maskRed, kernel, iterations=2)
    resultBlue = cv2.erode(maskBlue, kernel, iterations=2)
    resultGreen = cv2.erode(maskGreen, kernel, iterations=2)

    resultGreen = cv2.morphologyEx(resultGreen, cv2.MORPH_OPEN, kernel, iterations=2)
    resultRed = cv2.morphologyEx(resultRed, cv2.MORPH_OPEN, kernel, iterations=2)
    resultBlue = cv2.morphologyEx(resultBlue, cv2.MORPH_OPEN, kernel, iterations=2)

    resultRed = cv2.dilate(maskRed, kernel, iterations=2)
    resultBlue = cv2.dilate(maskBlue, kernel, iterations=2)
    resultGreen = cv2.dilate(maskGreen, kernel, iterations=2)

    resultRed = cv2.dilate(maskRed, kernel, iterations=3)
    resultBlue = cv2.dilate(maskBlue, kernel, iterations=3)
    resultGreen = cv2.dilate(maskGreen, kernel, iterations=3)


    redCenter, redRadius, redBlocks = contourCenter(resultRed)
    blueCenter, blueRadius, blueBlocks = contourCenter(resultBlue)
    greenCenter, greenRadius, greenBlocks = contourCenter(resultGreen)

    if sensor == b'd':
        if box < 3:
            if redRadius > blueRadius:
                serial.write(b'R')
                box += 1
        else:
            if greenRadius > blueRadius:
                serial.write(b'G')
                break

    if box < 3:
        if blueBlocks == 0:
            if redBlocks == 0:
                serial.write(b'L')
            else:
                if redCenter[0] < w // 3:
                    serial.write(b'l')
                elif redCenter[0] < 2 * w // 3:
                    serial.write(b'f')
                else:
                    serial.write(b'r')
        else:
            if redBlocks == 0:
                if blueCenter[0] < w // 2:
                    serial.write(b'r')
                else:
                    serial.write(b'l')
            else:
                if blueRadius >= redRadius:
                    if blueCenter[0] < w // 2:
                        serial.write(b'r')

                    else:
                        serial.write(b'l')

                else:
                    if redCenter[0] < w // 3:
                        serial.write(b'l')
                    elif redCenter[0] < 2 * w // 3:
                        serial.write(b'f')
                    else:
                        serial.write(b'r')

    if box == 3:
        if blueBlocks == 0:
            if greenBlocks == 0:
                serial.write(b'L')
            else:
                if greenCenter[0] < w // 3:
                    serial.write(b'l')
                elif greenCenter[0] < 2 * w // 3:
                    serial.write(b'f')
                else:
                    serial.write(b'r')
        else:
            if blueCenter[0] <= w // 2:
                serial.write(b'r')
            else:
                serial.write(b'l')



    x = cv2.waitKey(1)
    if x == ord('q'):
        break
cap.release()
serial.close()
cv2.destroyAllWindows()
