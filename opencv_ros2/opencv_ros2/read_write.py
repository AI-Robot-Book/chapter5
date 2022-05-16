# cv2 ライブラリをインポートする
import cv2

# 画像を読み込むには，関数 cv2.imread() を用います．
img = cv2.imread('fruits.jpg')
img_grayscale = cv2.imread('fruits.jpg',cv2.IMREAD_GRAYSCALE)

# 画像をウィンドウに表示するには，関数 cv2.imshow() を用います．
cv2.imshow('image',img)
cv2.imshow('grayscale image',img_grayscale)

# 画像の書き込みには，関数 cv2.imwrite() を用います．
cv2.imwrite('fruits_grayscale.jpg',img_grayscale)

# waitKey()はキーが押されるのを待ってウィンドウを閉じる、0は不定ループを指定する．
cv2.waitKey(0)

# cv2.destroyAllWindows() は、作成したすべてのウィンドウを単純に破棄します．
cv2.destroyAllWindows()
