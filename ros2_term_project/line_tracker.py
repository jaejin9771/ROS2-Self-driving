import cv2
import numpy
import numpy as np


class LineTracker:
    def __init__(self):
        self._delta = 0.0

    @property
    def delta(self):
        return self._delta

    @delta.setter
    def delta(self, value):
        self._delta = value

    def process(self, img: numpy.ndarray) -> None:

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 흰색 범위 정의
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([180, 30, 255])

        # 흰색 영역을 이진 마스크로 변환
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 이미지의 높이와 너비 가져오기
        h, w, _ = img.shape
        search_top = int(3 * h / 4 - 25)
        search_bot = int(3 * h / 4 - 5)

        # 관심 영역만 마스킹
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # 정지선 검출을 위한 가로 방향의 경계값 설정
        stop_line_width_thresh = w // 2  # 정지선의 최소 너비 임계값

        # 마스크에서 윤곽선 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 왼쪽과 오른쪽 끝점을 저장할 변수
        leftmost_x = w
        rightmost_x = 0

        for contour in contours:
            # 각 윤곽선의 외접 사각형 계산
            x, y, w_rect, h_rect = cv2.boundingRect(contour)

            # 정지선 무시 조건: 윤곽선의 너비가 충분히 길고 높이가 작음
            if w_rect > stop_line_width_thresh and h_rect < 20:  # 높이 임계값은 상황에 따라 조정
                # 정지선일 경우 무시
                continue

            # 왼쪽과 오른쪽 x 좌표 업데이트
            leftmost_x = min(leftmost_x, x)
            rightmost_x = max(rightmost_x, x)

        # 왼쪽 끝점과 오른쪽 끝점의 중간 계산
        if leftmost_x < rightmost_x:  # 올바른 감지일 경우만 실행
            midpoint_x = (leftmost_x + rightmost_x) // 2 + 40
            midpoint_y = (search_top + search_bot) // 2

            # 원본 이미지에 중간점 그리기
            cv2.circle(img, (midpoint_x, midpoint_y), 10, (0, 0, 255), -1)

            # 이미지 중심과의 오프셋 계산
            err = midpoint_x - w / 2
            self.delta = err

        # 이미지 출력
        cv2.imshow("Window", img)
        cv2.imshow("Mask", mask)
        cv2.waitKey(3)



