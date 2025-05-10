import os
import numpy as np
import cv2 as cv

templates = []
templateSizes = []

def detect(frame):
    for i in range(12):
        w, h = templateSizes[i]
        template = templates[i]
        res = cv.matchTemplate(frame, template, cv.TM_CCOEFF_NORMED)
        minVal, maxVal, minLoc, maxLoc = cv.minMaxLoc(res)
        if maxVal > 0.7:
            border = [maxLoc, (maxLoc[0] + w, maxLoc[1] + h)]
            print("detected: " + str(i+1))
            return border;

    return False

def main():
    for i in range(12):
        templates.append(cv.imread('templates/' + str(i+1) + '.png', cv.IMREAD_GRAYSCALE))
        templateSizes.append(templates[i].shape[::-1])

    vid = cv.VideoCapture(0)
    if not vid.isOpened():
        print("Failed to open camera")
        exit();
    
    while True:
        ret, frame = vid.read()
        
        if not ret:
            print("Failed to recieve frame")

        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        border = detect(frame)

        if border != False:
            cv.rectangle(frame, border[0], border[1], 255, 2)

        cv.imshow('stream', frame)
        if cv.waitKey(1) == ord('q'):
            break;

    vid.release()
    cv.destroyAllWindows()

main()
