import cv2
import numpy as np

class LaneDetection:
    def __init__(self):
        # Ative para ver os textos e caixas de detecção no Foxglove
        self.draw_img = True 
        self.scale_percent = 40
        # Inicializa o equalizador de contraste local
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

    def processImage(self, input_image):
        if input_image is None or input_image.size == 0:
            return None, None, None

        # Redimensionamento já é feito pelo nó seguidor, 
        # mas mantemos a flexibilidade aqui se necessário.
        resized_img = input_image 

        # 1. Pré-processamento e Threshold Robusto
        thresh_img = self.findThreshold(resized_img)

        # 2. Conversão para o formato de contornos
        thresh_img = self.convert_image_for_contours(thresh_img)

        # 3. Cálculo de erro (CTE) e ângulo
        cte, angle, output_image = self.calculateContours(thresh_img, resized_img)

        return cte, angle, output_image

    def findThreshold(self, img):
        try:
            # --- BLINDAGEM CONTRA LUZ ---
            # Converte para escala de cinza
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Aplica CLAHE para normalizar iluminação desigual no galpão
            equalized = self.clahe.apply(gray)
            
            # Blur suave para remover ruído de sensores baratos (ESP32-CAM)
            blurred = cv2.GaussianBlur(equalized, (5, 5), 0)
            
            # Threshold de Otsu: Calcula automaticamente o melhor limiar
            # É muito superior ao "valor fixo 120" para ambientes dinâmicos
            _, binarized_image = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            
            # Morfologia: Fecha buracos na linha detectada e remove pequenos pontos brancos
            kernel = np.ones((5, 5), dtype=np.uint8)
            dilated_img = cv2.morphologyEx(binarized_image, cv2.MORPH_OPEN, kernel)
            
        except Exception as e:
            print(f"Error in findThreshold: {e}")
            dilated_img = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
            
        return dilated_img

    def convert_image_for_contours(self, image):
        if len(image.shape) == 3:
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray_image = image
        return gray_image.astype(np.uint8)

    def calculateContours(self, thresh_img, original_img):
        # Encontra apenas o maior contorno (assume-se que seja a linha/corredor)
        contours, _ = cv2.findContours(thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cte, angle = None, None

        if contours:
            # Pega o maior contorno por área para evitar detecção de aves soltas
            c = max(contours, key=cv2.contourArea)
            
            # Filtro de área mínima: Ignora se o "corredor" for muito pequeno (falso positivo)
            if cv2.contourArea(c) > 500:
                blackbox = cv2.minAreaRect(c)
                (x_min, y_min), (w_min, h_min), ang = blackbox
                
                # Lógica de correção de ângulo OpenCV
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
                    cv2.drawContours(original_img, [box], 0, (0, 255, 0), 2)
                    
                    # Feedback visual de erro
                    cv2.putText(original_img, f"CTE: {cte}", (10, 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.line(original_img, (int(setpoint), 0), (int(setpoint), thresh_img.shape[0]), (255, 0, 0), 1)

        return cte, angle, original_img