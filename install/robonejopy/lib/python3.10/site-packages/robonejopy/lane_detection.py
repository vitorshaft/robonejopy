import cv2
import numpy as np

class LaneDetection:
    def __init__(self):
        self.draw_img = False
        self.scale_percent = 40

    def processImage(self, input_image):
        # Verificação de imagem vazia
        if input_image is None or input_image.size == 0:
            print("imagem de entrada None ou tamanho 0")
            return None, None, None

        # Redimensiona a imagem
        resized_img = input_image #self.resize_img(input_image, self.scale_percent)

        # Aplica a limiarização
        thresh_img = self.findThreshold(resized_img)

        # Converte a imagem para o formato suportado por findContours
        thresh_img = self.convert_image_for_contours(thresh_img)

        # Calcula o erro de cruzamento e o ângulo
        cte, angle, output_image = self.calculateContours(thresh_img, resized_img)

        return cte, angle, output_image

    def resize_img(self, img, scale_percent):
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized_img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
        resized_img = resized_img[75:, :]  # Cortar a parte superior se necessário
        return resized_img

    def findThreshold(self, img):
        try:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            s_channel = hsv[:, :, 1]
            blurred = cv2.GaussianBlur(s_channel, (3, 3), 1)
            binarized_image = np.zeros_like(blurred)
            binarized_image[blurred > 120] = 1
            kernel = np.ones((3, 3), dtype=np.uint8)
            dilated_img = cv2.dilate(binarized_image, kernel)
        except Exception as e:
            print(f"Error in findThreshold: {e}")
            dilated_img = np.zeros_like(img)
        return dilated_img

    def convert_image_for_contours(self, image):
        if len(image.shape) == 3:
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray_image = image
        if gray_image.dtype != np.uint8:
            gray_image = gray_image.astype(np.uint8)
        return gray_image

    def calculateContours(self, thresh_img, original_img):
        contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cte, angle = None, None

        if contours:
            blackbox = cv2.minAreaRect(contours[0])
            (x_min, y_min), (w_min, h_min), ang = blackbox
            if ang < -45:
                ang += 90
            if w_min < h_min and ang > 0:
                ang = (90 - ang) * -1
            if w_min > h_min and ang < 0:
                ang = 90 + ang
            setpoint = thresh_img.shape[1] / 2
            cte = -int(x_min - setpoint)
            angle = -int(ang)

            if self.draw_img:
                box = cv2.boxPoints(blackbox)
                box = np.int0(box)
                cv2.drawContours(original_img, [box], 0, (255, 0, 0), 1)
                ang_msg = f"Angle Error = {angle}"
                err_msg = f"Error = {cte}"
                cv2.putText(original_img, ang_msg, (130, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                cv2.putText(original_img, err_msg, (130, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                cv2.line(original_img, (int(x_min), 0), (int(x_min), thresh_img.shape[0]), (0, 0, 255), 1)

        return cte, angle, original_img
