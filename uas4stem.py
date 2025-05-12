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
    best = 0
    bestID = -1
    bestMatches = False
    for i in range(12):
        matches = bf.knnMatch(siftDesc[i], desc, k=2)
        validMatches = []
        if len(matches) != 0 and len(matches[0]) == 2:
            avr = 0
            for m, n in matches:
                if m.distance / n.distance < 0.5:
                    validMatches.append(m)
                    avr += m.distance / n.distance
            if len(validMatches) != 0:
                avr /= len(validMatches)
            else:
                continue
            avr = len(validMatches) / len(matches)
        if avr > best:
            best = avr
            bestID = i
            bestMatches = validMatches
    if best != 0:
        return [bestID, bestMatches]
    else:
        return bestMatches

def findPOI(frame):
    ret, thresh = cv.threshold(frame, 200, 255, cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    validContours = []
    for i in range(len(contours)):
        contour = contours[i]

        #check solidity
        hull = cv.convexHull(contour)
        if cv.contourArea(hull) == 0:
            continue
        solidity = abs(cv.contourArea(contour) / cv.contourArea(hull))
        if abs(solidity - 1) > 0.1:
            continue

        #check approx
        epsilon = 0.02 * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)
        if len(approx) != 4:
            continue

        if hierarchy[0][i][3] != -1:
            continue

        validContours.append(contour)
    validContours.sort(key=lambda contour: -cv.contourArea(contour))
    return validContours


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

    centerX = 0
    centerY = 0

    dx = 0
    dy = 0
    while True:
        ret, frame = vid.read()
        
        if not ret:
            print("Failed to recieve frame")

        if centerX == 0 and centerY == 0:
            w, h = frame.shape[:2]
            centerX = w/2
            centerY = h/2

        #filter out green
        filtered = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        lowerGreen = np.array([40, 255, 255])
        upperGreen = np.array([80, 0, 0])
        maskA = cv.inRange(filtered, np.array([0, 0, 0]), lowerGreen)
        maskB = cv.inRange(filtered, upperGreen, np.array([255, 255, 255]))
        mask = cv.bitwise_or(maskA, maskB)
        filtered = cv.bitwise_and(filtered, filtered, mask=mask)
        filtered = cv.cvtColor(filtered, cv.COLOR_HSV2BGR)
        filtered = cv.cvtColor(filtered, cv.COLOR_BGR2GRAY)

        contours = findPOI(filtered)
        frame = cv.drawContours(frame, contours, -1, (0, 255, 0), 2)

        if len(contours) != 0:
            M = cv.moments(contours[0])
            targetX = int(M["m10"] / M["m00"])
            targetY = int(M["m01"] / M["m00"])
            dx = centerX - targetX
            dy = centerY - targetY
            print("(" + str(dx) + ", " + str(dy) + ")")

        kp, desc = sift.detectAndCompute(filtered, None)
        detect = detectSIFT(filtered)
        if detect != False:
            frame = cv.drawMatches(templates[detect[0]], siftKP[detect[0]], frame, kp, detect[1], None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)


        cv.imshow('stream', frame)
        if cv.waitKey(1) == ord('q'):
            break;

    vid.release()
    cv.destroyAllWindows()

main()
