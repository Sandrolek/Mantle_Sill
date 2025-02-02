import sys

import numpy as np
import cv2

im = cv2.imread('arrow_together.png')
im3 = im.copy()

gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

blur = cv2.GaussianBlur(gray, (5, 5), 0)
# thresh = cv2.adaptiveThreshold(blur, 255, 1, 1, 11, 2)  #?
ret, thresh = cv2.threshold(blur, 127, 255, cv2.THRESH_BINARY_INV)
cv2.imshow('gray', gray)
cv2.imshow('thresh', thresh)

#################      Now finding Contours         ###################

contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)  # ?

samples = np.empty((0, 100))
responses = []
keys = [i for i in range(48, 58)]

for cnt in contours:
    if cv2.contourArea(cnt) > 50:
        [x, y, w, h] = cv2.boundingRect(cnt)

        if h > 28:
            cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)
            roi = thresh[y:y + h, x:x + w]
            roismall = cv2.resize(roi, (10, 10))
            cv2.imshow('roi', roismall)
            cv2.imshow('norm', im)
            key = cv2.waitKey(0)

            if key == 27:  # (escape to quit)
                sys.exit()
            elif key in keys:
                responses.append(int(chr(key)))
                sample = roismall.reshape((1, 100))
                samples = np.append(samples, sample, 0)
                cv2.rectangle(im, (x, y), (x + w, y + h), (255, 255, 255), 2)


#samples = np.array(samples, np.float32)
responses = np.array(responses, np.float32)
responses = responses.reshape((responses.size, 1))  # ?

print(responses)

#model = cv2.ml.KNearest_create()
#model.train(samples, cv2.ml.ROW_SAMPLE, responses)

#model.save('model.yaml')

np.savetxt('generalsamples.data', samples)
np.savetxt('generalresponses.data', responses)
