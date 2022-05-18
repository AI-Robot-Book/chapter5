import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    _, source = cap.read()
    cv2.imshow('source', source)
    gray = (
        0.299 * source[:, :, 2] +
        0.587 * source[:, :, 1] +
        0.114 * source[:, :, 0]).round()
    result = (gray > 127) * np.uint8(255)
    cv2.imshow('result', result)
    cv2.waitKey(1)
