import numpy as np
import cv2


def img_endecode(img):
    # type img: cv::mat
    # encode image from cv::mat
    img_encode = cv2.imencode('.bmp', img)
    data_encode = np.array(img_encode[1])
    str_encode = data_encode.tostring()

    # decode and display
    with open('str.bmp', 'w') as f:
        f.write(str_encode)
    nparr = np.fromstring(str_encode, np.uint8)
    img_decode = cv2.imdecode(nparr, 1)
    cv2.imwrite('decode.bmp', img_decode)


img = cv2.imread('../Camera1_img0.jpg', 1)

img_endecode(img)
k = cv2.waitKey(0)
if k == 27:         # wait for ESC key to exit
    cv2.destroyAllWindows()
