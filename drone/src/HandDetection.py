import cv2
import time
import mediapipe as mp
import math
import numpy as np


class handDetector():
    def __init__(self, mode=False, maxHands = 2, detectionCon = 0.5, trackCon=0.5, modComp = 1):
        self.mode = mode
        self.maxHands = maxHands
        self.modComp = modComp
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,self.modComp,self.detectionCon,
                                        self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
    
    def findHands(self, img, suc=True, draw=True):
        if suc == True:
            imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.results = self.hands.process(imgRGB)
            # print(results.multi_hand_landmarks)

        
        
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img
    
    def findPosition(self, img, handNo=0, draw=True):
        
        self.lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                # print(id,lm)
                h, w, c = img.shape
                cx, cy = int(lm.x*w), int(lm.y*h)
                # print(id, cx, cy)
                self.lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255,0, 255), cv2.FILLED)
        
        
        return self.lmList
    
    
    def findDistanceAlt(self, img, p1=0, p2=20, draw=True):
        x1, y1 = self.lmList[p1][1], self.lmList[p1][2]
        x2, y2 = self.lmList[p2][1], self.lmList[p2][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if draw:
            cv2.circle(img, (x1, y1), 10, (0, 255, 255), cv2.FILLED)
            cv2.circle(img, (x2, y2), 10, (0, 255, 255), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.circle(img, (cx, cy), 10, (0, 255, 255), cv2.FILLED)

        lengthZ = math.hypot(x2 - x1, y2 - y1)
        lengthZ = np.interp(lengthZ, [100, 280], [0, 3])
        return lengthZ
    
    def findDistanceX(self, img, p1=0, p2=12, draw=True):
        x1, y1 = self.lmList[p1][1], self.lmList[p1][2]
        x2, y2 = self.lmList[p2][1], self.lmList[p2][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if draw:
            cv2.circle(img, (x1, y1), 10, (220, 220, 0), cv2.FILLED)
            cv2.circle(img, (x2, y2), 10, (220, 220, 0), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.circle(img, (cx, cy), 10, (220, 220, 0), cv2.FILLED)

        lengthX = math.hypot(x2 - x1, y2 - y1)
        lengthX = np.interp(lengthX, [150, 250], [0, 1])
        return lengthX
    
    
    
    def findDistanceWZ(self, img, p1=4, p2=6, draw=True):
        x1, y1 = self.lmList[p1][1], self.lmList[p1][2]
        x2, y2 = self.lmList[p2][1], self.lmList[p2][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if draw:
            cv2.circle(img, (x1, y1), 10, (155, 155, 155), cv2.FILLED)
            cv2.circle(img, (x2, y2), 10, (155, 155, 155), cv2.FILLED)
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.circle(img, (cx, cy), 10, (155, 155, 155), cv2.FILLED)

        lengthwz = math.hypot(x2 - x1, y2 - y1)
        lengthwz = np.interp(lengthwz, [45, 110], [0, 0.5])
        return lengthwz
        

 
def main():
    pTime = 0
    cTime = 0
    cap = cv2.VideoCapture(0)
    detector = handDetector()

    while True:
        success, img = cap.read()
        img = detector.findHands(img)
        lmList = detector.findPosition(img)
        lengthZ = detector.findDistanceAlt(img)
        lengthX = detector.findDistanceX(img)
        lengthwz = detector.findDistanceWZ(img)
        if len(lmList) !=0:
            print(f'Высота: {lengthZ}, Скорость по Х: {lengthX}, Скорость поворота: {lengthwz}')
        
        
        cTime = time.time()
        fps = 1/(cTime - pTime)
        pTime = cTime
    
    
        cv2.putText(img, str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN,3, (255,0,255),3)       
        
            
        cv2.imshow('Image',img)
        cv2.waitKey(1)
         
 
 
    
if __name__ == "__main__":
    main()