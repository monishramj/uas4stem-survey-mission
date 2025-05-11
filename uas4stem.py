import os
import numpy as np
import cv2 as cv

sift = cv.xfeatures2d.SIFT_create()
bf = cv.BFMatcher()
templates = []
templateSizes = []
siftKP = []
siftDesc = []

def detectTemplate(frame):
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

def detectSIFT(frame):
    kp, desc = sift.detectAndCompute(frame, None)
    maxMatchesCnt = 0
    maxMatchesID = -1
    maxMatches = False
    for i in range(12):
        matches = bf.knnMatch(siftDesc[i], desc, k=2)
        validMatches = []
        if len(matches) != 0 and len(matches[0]) == 2:
            for m, n in matches:
                if m.distance < 0.5 * n.distance:
                    validMatches.append(m)
        if len(validMatches) > maxMatchesCnt:
            maxMatchesCnt = len(validMatches)
            maxMatchesID = i
            maxMatches = validMatches
    if maxMatches != False:
        return [maxMatchesID, maxMatches]
    else:
        return maxMatches


def main():
    for i in range(12):
        templates.append(cv.imread('templates/' + str(i+1) + '.png', cv.IMREAD_GRAYSCALE))
        templates[i] = cv.resize(templates[i], None, fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)
        templateSizes.append(templates[i].shape[::-1])
        kp, desc = sift.detectAndCompute(templates[i], None)
        siftKP.append(kp)
        siftDesc.append(desc)


    vid = cv.VideoCapture(0)
    if not vid.isOpened():
        print("Failed to open camera")
        exit();
    
    while True:
        ret, frame = vid.read()
        
        if not ret:
            print("Failed to recieve frame")

        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        kp, desc = sift.detectAndCompute(frame, None)
        detect = detectSIFT(frame)
        if detect != False:
            frame = cv.drawMatches(templates[detect[0]], siftKP[detect[0]], frame, kp, detect[1], None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

        cv.imshow('stream', frame)
        if cv.waitKey(1) == ord('q'):
            break;

    vid.release()
    cv.destroyAllWindows()

main()
