import cv2
import numpy as np
import time
from collections import deque
import serial  # Arduino ile iletişim için

# Arduino seri portunu açma (Port ismini kendi bilgisayarınıza göre ayarlayın)
arduino = serial.Serial('COM3', 9600)  # 'COM3' yerine Arduino'nuzun bağlı olduğu portu yazın
time.sleep(2)  # Arduino'nun bağlantı kurması için bekleme süresi

# Kamera başlatma
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Kamera açılamadı!")
    exit()

# Stabilizasyon için değişkenler
finger_buffer = deque(maxlen=5)  # Son 5 parmak sayısını tutar
last_command_time = time.time()
command_cooldown = 5  # Komutlar arası 1.3 saniye gecikme
previous_finger_count = 0

def detect_fingers(contour):
    fingers = 0
    hull = cv2.convexHull(contour, returnPoints=False)
    if len(hull) > 3:
        defects = cv2.convexityDefects(contour, hull)
        if defects is not None:
            for i in range(defects.shape[0]):
                s, e, f, d = defects[i, 0]
                start = tuple(contour[s][0])
                end = tuple(contour[e][0])
                far = tuple(contour[f][0])

                # Üçgen uzunluklarını hesapla
                a = np.linalg.norm(np.array(end) - np.array(start))
                b = np.linalg.norm(np.array(far) - np.array(start))
                c = np.linalg.norm(np.array(far) - np.array(end))

                angle = np.degrees(np.arccos((b**2 + c**2 - a**2) / (2 * b * c)))
                if angle <= 90 and d > 1000:
                    fingers += 1

    return min(fingers + 1, 5)

def is_fist(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)

    if perimeter == 0 or hull_area == 0:
        return False

    circularity = 4 * np.pi * (area / (perimeter ** 2))
    solidity = float(area) / hull_area

    return circularity > 0.4 and solidity > 0.9 and area < 15000

def stabilize_finger_count(finger_count):
    finger_buffer.append(finger_count)
    return int(np.mean(finger_buffer))

while True:
    ret, frame = cap.read()
    if not ret:
        print("Görüntü alınamadı!")
        break

    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_skin = np.array([0, 30, 60], dtype=np.uint8)
    upper_skin = np.array([20, 150, 255], dtype=np.uint8)
    skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)

    kernel = np.ones((5, 5), np.uint8)
    skin_mask = cv2.erode(skin_mask, kernel, iterations=1)
    skin_mask = cv2.dilate(skin_mask, kernel, iterations=2)
    skin_mask = cv2.GaussianBlur(skin_mask, (5, 5), 0)

    height, width = frame.shape[:2]
    skin_mask[:int(height / 3), :] = 0

    contours, _ = cv2.findContours(skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea, default=None)

    finger_count = 0
    if largest_contour is not None:
        area = cv2.contourArea(largest_contour)
        if area > 2000:
            if is_fist(largest_contour):
                finger_count = 0
            else:
                finger_count = detect_fingers(largest_contour)

            hull = cv2.convexHull(largest_contour)
            cv2.drawContours(frame, [hull], -1, (0, 255, 0), 2)

    stabilized_finger_count = stabilize_finger_count(finger_count)

    cv2.putText(frame, f'Parmak Sayisi: {stabilized_finger_count}', (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    if time.time() - last_command_time > command_cooldown:
        if stabilized_finger_count != previous_finger_count:
            if stabilized_finger_count == 0:
                print("DUR")
                arduino.write(b'0\n')
            elif stabilized_finger_count == 1:
                print("GERİ GİT")
                arduino.write(b'1\n')
            elif stabilized_finger_count == 2:
                print("SAĞA GİT")
                arduino.write(b'2\n')
            elif stabilized_finger_count == 3:
                print("SOLA GİT")
                arduino.write(b'3\n')
            elif stabilized_finger_count in [4, 5]:
                print("İLERİ GİT")
                arduino.write(b'5\n')

            last_command_time = time.time()
            previous_finger_count = stabilized_finger_count

    cv2.imshow("El Algilama", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
