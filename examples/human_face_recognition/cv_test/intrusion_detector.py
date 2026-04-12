import argparse
import csv
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np


ALGORITHM_LABELS = {
    "knn": "KNN Standard",
    "mog2": "MOG2 Stable",
}


@dataclass
class DetectorConfig:
    intrusion_threshold: float = 0.03
    border_width_ratio: float = 0.08
    border_width_min_px: int = 3
    gaussian_blur_ksize: int = 5
    learning_rate: float = 0.005  # 背景模型在线更新速率
    bootstrap_frames: int = 20
    bootstrap_learning_rate: float = 0.1  # 初始化建模阶段使用更高学习率
    min_blob_area: int = 15  # 连通域最小面积，小于该值的噪声会被忽略
    temporal_window: int = 2  # 时间平均窗口大小（帧数）
    temporal_threshold: float = 0.02  # 多帧平滑后的入侵阈值
    gradient_levels: int = 64  # 手工背景差分时使用的梯度量化级数
    gradient_threshold: int = 3  # 手工背景差分时的量化梯度阈值
    consecutive_trigger_frames: int = 3  # 连续多少帧触发后，才判定为入侵
    alarm_hold_frames: int = 10  # 报警锁存帧数，触发后至少保持这么多帧
    bg_method: str = "knn"  # knn（标准版）/ mog2（光强变化稳定版）
    knn_history: int = 200
    knn_dist2_threshold: float = 400.0
    knn_detect_shadows: bool = False
    mog2_history: int = 200
    mog2_var_threshold: float = 16.0
    mog2_detect_shadows: bool = False


class IntrusionDetector:
    def __init__(self, config: DetectorConfig):
        self.config = config
        self.bg_model = None
        self.ratio_buffer = deque(maxlen=config.temporal_window)
        self.trigger_count = 0
        self.alarm_hold_count = 0
        self.bg_subtractor = None
        self._reset_models()

    def _reset_models(self) -> None:
        self.bg_model = None
        self.bg_subtractor = None

        if self.config.bg_method == "mog2":
            self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
                history=self.config.mog2_history,
                varThreshold=self.config.mog2_var_threshold,
                detectShadows=self.config.mog2_detect_shadows,
            )
        elif self.config.bg_method == "knn":
            self.bg_subtractor = cv2.createBackgroundSubtractorKNN(
                history=self.config.knn_history,
                dist2Threshold=self.config.knn_dist2_threshold,
                detectShadows=self.config.knn_detect_shadows,
            )
        else:
            raise ValueError(f"Unsupported background method: {self.config.bg_method}")

    @staticmethod
    def _to_gray_blur(frame_roi: np.ndarray, ksize: int) -> np.ndarray:
        gray = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2GRAY)
        if ksize % 2 == 0:
            ksize += 1
        return cv2.GaussianBlur(gray, (ksize, ksize), 0)

    def _build_border_mask(self, h: int, w: int) -> np.ndarray:
        bw = int(min(h, w) * self.config.border_width_ratio)
        bw = max(bw, self.config.border_width_min_px)
        bw = min(bw, max(1, min(h, w) // 2))

        mask = np.zeros((h, w), dtype=np.uint8)
        mask[:bw, :] = 255
        mask[-bw:, :] = 255
        mask[:, :bw] = 255
        mask[:, -bw:] = 255
        return mask

    def _prepare_border_frame(self, frame_roi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        gray = self._to_gray_blur(frame_roi, self.config.gaussian_blur_ksize)
        border_mask = self._build_border_mask(*gray.shape)
        border_frame = cv2.bitwise_and(gray, border_mask)
        return border_frame, border_mask

    def bootstrap(self, cap: cv2.VideoCapture, roi: tuple[int, int, int, int]) -> None:
        x, y, w, h = roi
        self._reset_models()

        current_pos = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        first_manual_frame = None

        for _ in range(self.config.bootstrap_frames):
            ok, frame = cap.read()
            if not ok:
                break

            crop = frame[y : y + h, x : x + w]
            if crop.size == 0:
                continue

            border_frame, _ = self._prepare_border_frame(crop)
            if self.bg_subtractor is not None:
                self.bg_subtractor.apply(border_frame, learningRate=self.config.bootstrap_learning_rate)
            elif first_manual_frame is None:
                first_manual_frame = border_frame.astype(np.float32)

        cap.set(cv2.CAP_PROP_POS_FRAMES, current_pos)

        if self.bg_subtractor is None:
            self.bg_model = first_manual_frame

        self.ratio_buffer.clear()
        self.trigger_count = 0
        self.alarm_hold_count = 0

    def process(self, frame_roi: np.ndarray) -> tuple[bool, float, float, int, int, np.ndarray, np.ndarray]:
        """
        处理单帧 ROI 图像。

        返回：
            intrusion_decision, current_ratio, smoothed_ratio, trigger_count, alarm_hold_count, fg_mask, border_mask
        """
        border_frame, border_mask = self._prepare_border_frame(frame_roi)

        if self.bg_subtractor is not None:
            fg = self.bg_subtractor.apply(border_frame, learningRate=self.config.learning_rate)
            fg = np.where(fg > 0, 255, 0).astype(np.uint8)
        else:
            if self.bg_model is None:
                self.bg_model = border_frame.astype(np.float32)

            bg_u8 = cv2.convertScaleAbs(self.bg_model)
            diff = cv2.absdiff(border_frame, bg_u8)
            step_size = 256.0 / self.config.gradient_levels
            quantized = (diff / step_size).astype(np.uint8)
            fg = ((quantized > self.config.gradient_threshold).astype(np.uint8)) * 255

        fg = cv2.bitwise_and(fg, border_mask)

        kernel_small = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        kernel_large = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kernel_small, iterations=2)
        fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, kernel_large, iterations=2)
        fg = cv2.bitwise_and(fg, border_mask)

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(fg, connectivity=8)
        cleaned = np.zeros_like(fg)
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area >= self.config.min_blob_area:
                cleaned[labels == i] = 255
        cleaned = cv2.bitwise_and(cleaned, border_mask)

        cut_pixels = cv2.countNonZero(cleaned)
        border_pixels = cv2.countNonZero(border_mask)
        current_ratio = float(cut_pixels) / float(border_pixels) if border_pixels > 0 else 0.0

        self.ratio_buffer.append(current_ratio)
        smoothed_ratio = float(np.mean(list(self.ratio_buffer)))

        triggered = (
            smoothed_ratio > self.config.temporal_threshold
            or current_ratio > self.config.intrusion_threshold
        )
        if triggered:
            self.trigger_count += 1
        else:
            self.trigger_count = 0

        state_triggered = self.trigger_count >= self.config.consecutive_trigger_frames
        if state_triggered:
            self.alarm_hold_count = self.config.alarm_hold_frames
        elif self.alarm_hold_count > 0:
            self.alarm_hold_count -= 1
        intrusion = self.alarm_hold_count > 0

        if self.bg_subtractor is None:
            bg_update_mask = cv2.bitwise_and(cv2.bitwise_not(cleaned), border_mask)
            cv2.accumulateWeighted(
                border_frame.astype(np.float32), self.bg_model, self.config.learning_rate, mask=bg_update_mask
            )

        return intrusion, current_ratio, smoothed_ratio, self.trigger_count, self.alarm_hold_count, cleaned, border_mask


class ROIDrawer:
    """交互式 ROI 绘制工具，只保留矩形和圆形。"""

    def __init__(self, frame: np.ndarray):
        self.original = frame.copy()
        self.display = frame.copy()
        self.output_mask = None
        self.shape_type = "rectangle"
        self.drawing = False
        self.start_pos = None
        self.window_name = "Custom ROI - Select Shape Type and Draw"

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_pos = (x, y)

        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.display = self.original.copy()

            if self.shape_type == "rectangle":
                cv2.rectangle(self.display, self.start_pos, (x, y), (0, 255, 0), 2)
            elif self.shape_type == "circle":
                radius = int(np.sqrt((x - self.start_pos[0]) ** 2 + (y - self.start_pos[1]) ** 2))
                cv2.circle(self.display, self.start_pos, radius, (0, 255, 0), 2)

            self.display_help()
            cv2.imshow(self.window_name, self.display)

        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            if self.shape_type == "rectangle" and self.start_pos != (x, y):
                self.finalize_rectangle((x, y))
            elif self.shape_type == "circle":
                self.finalize_circle((x, y))

    def finalize_rectangle(self, end_pos):
        """保存矩形区域。"""
        x1, y1 = self.start_pos
        x2, y2 = end_pos
        x_min, x_max = min(x1, x2), max(x1, x2)
        y_min, y_max = min(y1, y2), max(y1, y2)

        self.output_mask = np.zeros(self.original.shape[:2], dtype=np.uint8)
        cv2.rectangle(self.output_mask, (x_min, y_min), (x_max, y_max), 255, -1)

    def finalize_circle(self, end_pos):
        """保存圆形区域。"""
        center = self.start_pos
        radius = int(np.sqrt((end_pos[0] - center[0]) ** 2 + (end_pos[1] - center[1]) ** 2))
        if radius > 0:
            self.output_mask = np.zeros(self.original.shape[:2], dtype=np.uint8)
            cv2.circle(self.output_mask, center, radius, 255, -1)

    def display_help(self):
        """显示当前可用的绘制操作。"""
        help_text = f"Shape: {self.shape_type.upper()} | [1]=Rect [2]=Circle [ENTER]=Done [ESC]=Cancel"
        h = self.display.shape[0]
        cv2.putText(self.display, help_text, (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

    def select(self):
        """进入交互式绘制循环。"""
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.display = self.original.copy()
        self.display_help()
        cv2.imshow(self.window_name, self.display)

        while True:
            key = cv2.waitKey(1) & 0xFF

            if key == ord("1"):
                self.shape_type = "rectangle"
                self.display = self.original.copy()
                self.display_help()
                cv2.imshow(self.window_name, self.display)

            elif key == ord("2"):
                self.shape_type = "circle"
                self.display = self.original.copy()
                self.display_help()
                cv2.imshow(self.window_name, self.display)

            elif key == 13:
                if self.output_mask is not None:
                    break
                self.display = self.original.copy()
                cv2.putText(
                    self.display,
                    "Please draw a region first!",
                    (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
                self.display_help()
                cv2.imshow(self.window_name, self.display)

            elif key == 27:
                cv2.destroyWindow(self.window_name)
                raise ValueError("ROI selection cancelled.")

        cv2.destroyWindow(self.window_name)
        return self.output_mask, self.shape_type

    def mask_to_rect(self):
        """将 mask 转换为外接矩形坐标 (x, y, w, h)。"""
        contours, _ = cv2.findContours(self.output_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(cnt)
            return x, y, w, h
        return None


def select_roi(frame: np.ndarray, window_name: str = "Select ROI") -> tuple[int, int, int, int]:
    """选择 ROI，支持自定义矩形/圆形，或默认矩形框选。"""
    print("\n" + "=" * 60)
    print("ROI Selection Mode:")
    print("  [1] 自定义形状 - 矩形/圆形")
    print("  [2] 快速矩形选择 (默认)")
    print("=" * 60)

    choice = input("请选择模式 (1 或 2，默认=2): ").strip()

    if choice == "1":
        try:
            drawer = ROIDrawer(frame)
            _, shape_type = drawer.select()
            rect = drawer.mask_to_rect()
            if rect:
                print(f"✓ {shape_type.capitalize()} ROI selected: x={rect[0]}, y={rect[1]}, w={rect[2]}, h={rect[3]}")
                return rect
            raise ValueError("Failed to extract ROI from mask.")
        except ValueError as e:
            print(f"✗ {e}")
            return select_roi(frame, window_name)

    roi = cv2.selectROI(window_name, frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow(window_name)
    if roi[2] <= 0 or roi[3] <= 0:
        raise ValueError("Invalid ROI selected.")
    return int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect foreign-object intrusion into a user-selected ROI.")
    parser.add_argument("--video", type=str, default="video/test_3.mp4", help="Input video path")
    parser.add_argument("--threshold", type=float, default=0.05, help="Temporal intrusion threshold (smoothed border-cut ratio)")
    parser.add_argument("--csv", type=str, default="intrusion_metrics.csv", help="Output CSV path")
    parser.add_argument(
        "--bg-method",
        type=str,
        default="knn",
        choices=["knn", "mog2"],
        help="Background model on ROI border band: knn=standard, mog2=stable under strong illumination changes",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {args.video}")

    ok, first = cap.read()
    if not ok:
        raise RuntimeError("Failed to read first frame.")

    roi = select_roi(first, "Select ROI and press ENTER")

    config = DetectorConfig(temporal_threshold=args.threshold, bg_method=args.bg_method)
    detector = IntrusionDetector(config)

    detector.bootstrap(cap, roi)
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    x, y, w, h = roi
    fps = cap.get(cv2.CAP_PROP_FPS)
    fps = fps if fps and fps > 0 else 30.0

    with open(args.csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["frame", "time_sec", "intrusion", "current_ratio", "smoothed_ratio", "trigger_count", "hold_count"])

        frame_idx = -1
        step_mode = False
        while True:
            if step_mode:
                key = cv2.waitKey(0) & 0xFF
                if key == ord("q"):
                    break
                elif key != 82 and key != 83:
                    if key == ord(" "):
                        step_mode = False
                    elif key == ord("r"):
                        roi = select_roi(frame, "Reselect ROI and press ENTER")
                        x, y, w, h = roi
                        detector.bootstrap(cap, roi)
                    continue

            ok, frame = cap.read()
            if not ok:
                break

            frame_idx += 1
            crop = frame[y : y + h, x : x + w]
            if crop.size == 0:
                continue

            intrusion, current_ratio, smoothed_ratio, trigger_count, hold_count, fg, border_mask = detector.process(crop)

            writer.writerow([
                frame_idx,
                frame_idx / fps,
                int(intrusion),
                f"{current_ratio:.6f}",
                f"{smoothed_ratio:.6f}",
                trigger_count,
                hold_count,
            ])

            display = frame.copy()
            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 255), 2)

            label_1 = f"intrusion: {'YES' if intrusion else 'NO'}"
            label_2 = f"current ratio: {current_ratio * 100:.2f}%"
            label_3 = f"smoothed ratio: {smoothed_ratio * 100:.2f}%"
            label_4 = f"trigger count: {trigger_count}/{config.consecutive_trigger_frames}"
            label_5 = f"hold count: {hold_count}/{config.alarm_hold_frames}"
            label_6 = f"algorithm: {ALGORITHM_LABELS.get(config.bg_method, config.bg_method.upper())}"
            mode_label = "[STEP MODE]" if step_mode else "[AUTO MODE]"

            color = (0, 0, 255) if intrusion else (0, 200, 0)
            cv2.putText(display, mode_label, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 100, 200), 2)
            cv2.putText(display, label_1, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            cv2.putText(display, label_2, (20, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display, label_3, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display, label_4, (20, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display, label_5, (20, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display, label_6, (20, 225), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            hint_y = display.shape[0] - 100
            cv2.putText(
                display,
                "CONTROLS: [SPACE]=toggle mode | [UP/DN/LR arrows]=step | [R]=reselect ROI | [Q]=quit",
                (20, hint_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (100, 200, 255),
                1,
            )

            fg_vis = cv2.cvtColor(fg, cv2.COLOR_GRAY2BGR)
            fg_vis[border_mask > 0] = (0, 255, 255)

            stacked = cv2.hconcat([
                cv2.resize(display, (display.shape[1], display.shape[0])),
                cv2.resize(fg_vis, (display.shape[1], display.shape[0])),
            ])

            cv2.imshow("Intrusion Detection (left: frame, right: fg+border)", stacked)

            if not step_mode:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" "):
                    step_mode = True
                if key == ord("r"):
                    roi = select_roi(frame, "Reselect ROI and press ENTER")
                    x, y, w, h = roi
                    detector.bootstrap(cap, roi)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
