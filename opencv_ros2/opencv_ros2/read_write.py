import cv2

img = cv2.imread('fruits.jpg')
img_grayscale = cv2.imread('fruits.jpg',cv2.IMREAD_GRAYSCALE)

cv2.imshow('image',img)
cv2.imshow('grayscale image',img_grayscale)

cv2.imwrite('fruits_grayscale.jpg',img_grayscale)

cv2.waitKey(0)

cv2.destroyAllWindows()
