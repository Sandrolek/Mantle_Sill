import cv2
import numpy as np

#######   training part    ###############

samples = np.loadtxt('generalsamples.data', np.float32)
responses = np.loadtxt('generalresponses.data', np.float32)
responses = responses.reshape((responses.size, 1))

model = cv2.ml.KNearest_create()
model.train(samples, cv2.ml.ROW_SAMPLE, responses)

############################# testing part  #########################

relatives = {
    0: 'Down',
    1: 'Left',
    2: 'Up',
    3: 'Right'
}

im = cv2.imread('arrow_together.png')
out = np.zeros(im.shape, np.uint8)
gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
# thresh = cv2.adaptiveThreshold(gray, 255, 1, 1, 11, 2)

ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

for cnt in contours:
    if cv2.contourArea(cnt) > 50:
        [x, y, w, h] = cv2.boundingRect(cnt)
        if h > 28:
            try:
                cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)
                roi = thresh[y:y + h, x:x + w]
                l = float(w) / h
                roismall = cv2.resize(roi, (10, 10))
                roismall = roismall.reshape((1, 100))
                roismall = np.float32(roismall)
                retval, results, neigh_resp, dists = model.findNearest(roismall, k=1)
                print(type(results[0][0]))
                num = int(results[0][0])  # ?
                result = relatives[num]
                cv2.putText(out, result, (x + w // 2, y + h // 2), 0, 1, (0, 255, 0))
            except cv2.Error as e:
                print('Invalid')

cv2.imshow('im', im)
cv2.imshow('out', out)
cv2.waitKey(0)
