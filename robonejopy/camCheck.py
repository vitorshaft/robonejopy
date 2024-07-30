import cv2

# Endereço IP da ESP32-CAM e a porta para a stream de vídeo
#url = 'http://192.168.76.224:81/stream'
url = '/dev/video0'
# Cria um objeto VideoCapture para capturar a stream de vídeo
cap = cv2.VideoCapture(url)

while True:
    # Captura cada frame da stream de vídeo
    ret, frame = cap.read()

    # Verifica se o frame foi capturado corretamente
    if not ret:
        break

    # Exibe o frame em uma janela de visualização
    cv2.imshow('ESP32-CAM Stream', frame)

    # Para sair do loop e fechar a janela pressione 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera os recursos e fecha a janela ao finalizar
cap.release()
cv2.destroyAllWindows()
