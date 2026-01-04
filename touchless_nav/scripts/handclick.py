import cv2
import numpy as np
from collections import deque
import mediapipe as mp
import time

class HandClickDetector:
    """Detects 'tap' gesture (forward motion of index finger)."""
    def __init__(self, max_num_hands=1, detection_conf=0.7, tracking_conf=0.7):
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=max_num_hands,
            min_detection_confidence=detection_conf,
            min_tracking_confidence=tracking_conf
        )
        self.z_history = deque(maxlen=5)
        self.click_state = False
        self.last_click_time = 0
        self.cooldown = 0.4  # seconds between taps

    def process_frame(self, frame):
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)
        click_coords = None

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                x, y, z = int(index_tip.x * w), int(index_tip.y * h), index_tip.z
                self.z_history.append(z)

                if len(self.z_history) >= 2:
                    dz = self.z_history[-1] - self.z_history[-2]

                    if (dz < -0.02 and not self.click_state
                            and time.time() - self.last_click_time > self.cooldown):
                        self.click_state = True
                        self.last_click_time = time.time()
                        click_coords = (x, y)

                    elif dz > 0.015 and self.click_state:
                        self.click_state = False

                color = (0, 255, 0) if self.click_state else (0, 0, 255)
                cv2.circle(frame, (x, y), 8, color, cv2.FILLED)

        return frame, click_coords