import cv2

cap = cv2.VideoCapture(0)

while True:
    _, source = cap.read()
    cv2.imshow('source', source)
    gray = cv2.cvtColor(source, cv2.COLOR_BGR2GRAY)
    _, result = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
    cv2.imshow('result', result)
    cv2.waitKey(1)
