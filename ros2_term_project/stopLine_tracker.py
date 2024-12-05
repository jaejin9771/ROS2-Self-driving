import cv2
import numpy
import numpy as np


class StopLineTracker:
    def __init__(self):
        self.stop_line_detected = False

    def process(self, img: numpy.ndarray) -> None:
        """
        정지선 감지를 위한 프로세스 함수.
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 흰색 범위 정의
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])

        # 흰색 영역을 이진 마스크로 변환
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 이미지의 높이와 너비 가져오기
        h, w, _ = img.shape
        search_top = int(h / 2 + 40)
        search_bot = int(h / 2 + 60)

        # 관심 영역만 마스킹
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # 윤곽선 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1400:  # 특정 면적 이상인 경우 정지선으로 간주
                self.stop_line_detected = True
                break
        else:
            self.stop_line_detected = False

        # 정지선 감지 결과를 이미지에 그리기
        if self.stop_line_detected:
            cv2.putText(img, "Stop Line Detected", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # 이미지 출력
        cv2.imshow("Stop Line Mask", mask)
        cv2.waitKey(3)