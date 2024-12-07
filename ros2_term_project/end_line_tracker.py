import cv2
import numpy
import numpy as np


class EndLineTracker:
    def __init__(self):
        self.end_line_detected = False

    def process(self, img: numpy.ndarray) -> None:
        """
        주황색 선 감지를 위한 프로세스 함수.
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 주황색 범위 정의 (필요에 따라 조정)
        lower_orange = np.array([10, 100, 100])  # HSV에서 주황색의 하한값
        upper_orange = np.array([25, 255, 255])  # HSV에서 주황색의 상한값

        # 주황색 영역을 이진 마스크로 변환
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # 이미지의 높이와 너비 가져오기
        h, w, _ = img.shape
        search_top = int(h / 2 + 30)
        search_bot = int(h / 2 + 70)

        # 관심 영역만 마스킹
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # 윤곽선 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 150:  # 특정 면적 이상인 경우 주황색 선으로 간주
                self.end_line_detected = True
                break

        # 이미지 출력
        # cv2.imshow("End Line Mask", mask)
        cv2.waitKey(3)

