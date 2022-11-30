import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
image = cv2.imread('/home/muammer/Desktop/qr/111.png')
obj1 = pyzbar.decode(image)
font = cv2.FONT_HERSHEY_PLAIN
(x, y, w, h) = obj1[0].rect
print("a")
for obj in obj1:
    cv2.putText(image, str(obj.data), (50, 50), font, 2,(255, 0, 0), 3)
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
    cv2.imshow("qr kod",image)
    cv2.waitKey(0)
    print("Type:", obj.type)
    print("Data: ", obj.data, "\n")
