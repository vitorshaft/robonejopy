import serial
import time

# Defina a porta serial do Arduino
porta_serial = '/dev/ttyUSB0'  # Modifique conforme necessário

# Configurar a comunicação serial
serial_arduino = serial.Serial(porta_serial, 9600, timeout=1)

# Aguardar um momento para garantir que a comunicação seja estabelecida
time.sleep(2)
mensagem = 1
try:
    while True:
        if(mensagem > 7):
            mensagem = 0
        # Enviar dados para o Arduino
        #mensagem = 1#"1\n2\n3\n"
        #serial_arduino.write(mensagem.encode())
        serial_arduino.write(str(mensagem).encode())
        # Aguardar um breve intervalo
        time.sleep(1)

        # Ler resposta do Arduino
        resposta = serial_arduino.readline().decode().strip()
        print("Resposta do Arduino:", resposta)
        mensagem = mensagem+1

except KeyboardInterrupt:
    print("Programa interrompido pelo usuário.")

finally:
    # Fechar a porta serial ao finalizar
    serial_arduino.close()
