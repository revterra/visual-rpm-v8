#!/usr/bin/env python3
"""
RPM Meter v8 with GUI - Measures rotational speed by detecting black tape on a rotor
Uses Aravis for FLIR Blackfly S GigE Vision camera control and streaming
"""

import numpy as np
import cv2
import time
import os
from collections import deque
from datetime import datetime

import gi
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis


class RPMMeterGUI:
    def __init__(self, camera_address="169.254.172.226"):
        self.camera_address = camera_address
        self.width = 720
        self.height = 540

        # RPM calculation state
        self.last_trigger_time = None
        self.time_deltas = deque(maxlen=6)
        self.current_rpm = 0.0
        self.min_readings = 4

        # Detection state
        self.in_dark_region = False
        self.threshold = 100
        self.brightness = 0
        self.exposure_us = 2000  # microseconds (2ms — short enough for ~290fps)

        # ROI (will be updated after capture starts with actual dimensions)
        self.roi_x = 620
        self.roi_y = 490
        self.roi_w = 200
        self.roi_h = 100

        self.frame_count = 0
        self.camera_fps = 0.0
        self.arv_device = None
        self.fps_time = time.perf_counter()
        self.fps_frame_count = 0
        self.fps = 0.0
        self.target_display_fps = 30
        self.last_display_time = 0

        # Recording state
        self.recording = False
        self.video_writer = None
        self.record_path = None
        self.record_frame_count = 0

    def set_exposure(self, exposure_us):
        """Change camera exposure time in microseconds via Aravis GI bindings."""
        self.exposure_us = exposure_us
        if self.arv_device is not None:
            try:
                self.arv_device.set_float_feature_value("ExposureTime", float(exposure_us))
                try:
                    fps = self.arv_device.get_float_feature_value("AcquisitionFrameRate")
                    self.camera_fps = fps
                except Exception:
                    pass
            except Exception as e:
                print(f"Aravis set_exposure error: {e}")

    def start_capture(self):
        # Open Aravis camera via IP address (GigE Vision)
        self.arv_camera = Aravis.Camera.new(self.camera_address)
        self.arv_device = self.arv_camera.get_device()
        print(f"GigE camera connected: {self.camera_address}")

        # GigE tuning — jumbo frames and zero inter-packet delay
        self.arv_device.set_integer_feature_value("GevSCPSPacketSize", 9000)
        self.arv_device.set_integer_feature_value("GevSCPD", 0)
        pkt = self.arv_device.get_integer_feature_value("GevSCPSPacketSize")
        print(f"GigE packet size: {pkt}")

        # Disable auto features BEFORE switching to Mono8 (GainAuto caps fps at 200)
        self.arv_device.set_string_feature_value("GainAuto", "Off")

        # Configure camera — use Mono8 for RPM detection (color not needed)
        self.arv_device.set_string_feature_value("PixelFormat", "Mono8")
        # Full sensor resolution — 291fps achievable at 720x540 with jumbo frames
        self.arv_device.set_integer_feature_value("OffsetX", 0)
        self.arv_device.set_integer_feature_value("OffsetY", 0)
        self.arv_device.set_integer_feature_value("Width", 720)
        self.arv_device.set_integer_feature_value("Height", 540)
        self.arv_device.set_string_feature_value("ExposureAuto", "Off")
        self.arv_device.set_float_feature_value("ExposureTime", float(self.exposure_us))

        # Disable frame rate limiter — free-run at max sensor speed (~291fps)
        self.arv_device.set_boolean_feature_value("AcquisitionFrameRateEnable", False)
        bounds = self.arv_device.get_float_feature_bounds("AcquisitionFrameRate")
        print(f"Camera configured: 720x540 Mono8, ExposureTime={self.exposure_us}us, max FPS={bounds[1]:.1f}")

        # Create stream with large socket buffer for high-speed GigE
        self.stream = self.arv_camera.create_stream(None, None)
        self.stream.set_property("socket-buffer", Aravis.GvStreamSocketBuffer.FIXED)
        self.stream.set_property("socket-buffer-size", 33554432)  # 32MB
        payload = self.arv_camera.get_payload()
        # Enough buffers for throughput, but not so many that they add lag
        for _ in range(30):
            self.stream.push_buffer(Aravis.Buffer.new_allocate(payload))

        self.arv_camera.start_acquisition()
        print("Acquisition started")

        # Get actual frame dimensions from camera
        self.width = self.arv_camera.get_region()[2]
        self.height = self.arv_camera.get_region()[3]
        print(f"Frame size: {self.width}x{self.height}")
        self._held_buffer = None
        # Update ROI for actual dimensions
        self.roi_x = self.width // 2 - 50
        self.roi_y = self.height // 2 - 25

    def _pop_buffer(self, timeout_us=0):
        """Pop a buffer from the stream. timeout_us=0 for non-blocking."""
        if timeout_us > 0:
            buffer = self.stream.timeout_pop_buffer(timeout_us)
        else:
            buffer = self.stream.try_pop_buffer()
        if buffer is None:
            return None
        if buffer.get_status() != Aravis.BufferStatus.SUCCESS:
            self.stream.push_buffer(buffer)
            return None
        # Return previous buffer to pool before using new one
        if self._held_buffer is not None:
            self.stream.push_buffer(self._held_buffer)
        self._held_buffer = buffer
        # Zero-copy: numpy view directly into buffer memory
        data = buffer.get_data()
        return np.frombuffer(data, dtype=np.uint8).reshape(self.height, self.width)

    def drain_and_process(self):
        """Drain all available frames, process each for RPM, return the latest."""
        # Block briefly for the first frame so we don't spin the CPU
        latest_frame = self._pop_buffer(timeout_us=5000)  # 5ms
        if latest_frame is not None:
            self.process_frame(latest_frame)
            self.frame_count += 1
            if self.recording and self.video_writer is not None:
                self.video_writer.write(latest_frame)
                self.record_frame_count += 1
        # Non-blocking drain of remaining queued frames
        while True:
            frame = self._pop_buffer(timeout_us=0)
            if frame is None:
                break
            self.process_frame(frame)
            self.frame_count += 1
            if self.recording and self.video_writer is not None:
                self.video_writer.write(frame)
                self.record_frame_count += 1
            latest_frame = frame
        return latest_frame

    def process_frame(self, frame):
        # Get ROI
        x = max(0, min(self.roi_x, self.width - self.roi_w))
        y = max(0, min(self.roi_y, self.height - self.roi_h))
        w = min(self.roi_w, self.width - x)
        h = min(self.roi_h, self.height - y)

        roi = frame[y:y+h, x:x+w]
        gray_roi = roi if len(roi.shape) == 2 else cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        self.brightness = np.mean(gray_roi)
        is_dark = self.brightness < self.threshold
        current_time = time.perf_counter()

        # Rising edge detection
        if is_dark and not self.in_dark_region:
            self.in_dark_region = True

            if self.last_trigger_time is not None:
                delta_time = current_time - self.last_trigger_time

                if delta_time > 0.001:  # Min 1ms between triggers (max 60000 RPM)
                    self.time_deltas.append(delta_time)

                    # Only update RPM if we have enough readings
                    if len(self.time_deltas) >= self.min_readings:
                        sorted_deltas = sorted(self.time_deltas)
                        n = len(sorted_deltas)

                        if n >= 4:
                            trimmed = sorted_deltas[1:-1]
                        else:
                            trimmed = sorted_deltas

                        mid = len(trimmed) // 2
                        if len(trimmed) % 2 == 0:
                            median_delta = (trimmed[mid-1] + trimmed[mid]) / 2
                        else:
                            median_delta = trimmed[mid]

                        rpm = 60.0 / median_delta

                        if 1 < rpm < 200000:
                            self.current_rpm = rpm

            self.last_trigger_time = current_time

        elif not is_dark:
            self.in_dark_region = False

    def toggle_recording(self):
        if self.recording:
            # Stop recording
            self.recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            print(f"Recording stopped: {self.record_path} ({self.record_frame_count} frames)")
        else:
            # Start recording
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.record_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                f"rpm_recording_{timestamp}.avi"
            )
            # Use raw/uncompressed for speed — grayscale frames at full camera fps
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            target_fps = self.fps if self.fps > 0 else 280.0
            self.video_writer = cv2.VideoWriter(
                self.record_path, fourcc, target_fps,
                (self.width, self.height), isColor=False
            )
            self.record_frame_count = 0
            self.recording = True
            print(f"Recording started: {self.record_path} @ {target_fps:.0f} fps")

    def draw_overlay_scaled(self, frame, scale_x, scale_y):
        """Draw overlay on a scaled-down display frame"""
        rx = int(self.roi_x * scale_x)
        ry = int(self.roi_y * scale_y)
        rw = int(self.roi_w * scale_x)
        rh = int(self.roi_h * scale_y)

        color = (0, 0, 255) if self.in_dark_region else (0, 255, 0)
        cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), color, 2)

        # Draw info
        cv2.putText(frame, f"RPM: {self.current_rpm:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Brightness: {self.brightness:.1f}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Threshold: {self.threshold}", (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Exposure: {self.exposure_us/1000:.2f} ms", (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        status = "TAPE DETECTED" if self.in_dark_region else "Waiting..."
        cv2.putText(frame, status, (10, 155),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        readings = len(self.time_deltas)
        cv2.putText(frame, f"Readings: {readings}/{self.min_readings}", (10, 185),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 215),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # Recording indicator
        if self.recording:
            cv2.circle(frame, (frame.shape[1] - 30, 30), 12, (0, 0, 255), -1)
            cv2.putText(frame, f"REC {self.record_frame_count}", (frame.shape[1] - 160, 38),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(frame, "Q:Quit  WASD:Move  +/-:Resize  R:Record",
                    (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        return frame

    def _on_threshold_trackbar(self, val):
        self.threshold = val

    def _on_exposure_trackbar(self, val):
        exposure_us = max(100, val * 100)
        self.set_exposure(exposure_us)

    def run(self):
        print("Starting RPM meter v8 (GigE)...")
        print("Controls: Q=Quit, WASD=Move ROI, +/-=Resize, R=Record")

        self.start_capture()

        cv2.namedWindow('RPM Meter', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RPM Meter', 960, 720)

        cv2.createTrackbar('Threshold', 'RPM Meter', self.threshold, 255,
                           self._on_threshold_trackbar)
        cv2.createTrackbar('Exposure (0.1ms)', 'RPM Meter',
                           self.exposure_us // 100, 500,
                           self._on_exposure_trackbar)

        try:
            while True:
                # Drain all available frames — process every one for RPM, keep latest
                frame = self.drain_and_process()
                if frame is None:
                    continue

                # Calculate processing FPS every ~1 second
                now = time.perf_counter()
                elapsed = now - self.fps_time
                if elapsed >= 1.0:
                    self.fps = (self.frame_count - self.fps_frame_count) / elapsed
                    self.fps_frame_count = self.frame_count
                    self.fps_time = now

                # Display at ~30fps
                now = time.perf_counter()
                if now - self.last_display_time >= 1.0 / self.target_display_fps:
                    self.last_display_time = now
                    if len(frame.shape) == 2:
                        display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    else:
                        display = frame.copy()
                    scale_x = display.shape[1] / self.width
                    scale_y = display.shape[0] / self.height
                    display = self.draw_overlay_scaled(display, scale_x, scale_y)
                    cv2.imshow('RPM Meter', display)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('r'):
                        self.toggle_recording()
                    elif key == ord('w'):
                        self.roi_y = max(0, self.roi_y - 20)
                    elif key == ord('s'):
                        self.roi_y = min(self.height - self.roi_h, self.roi_y + 20)
                    elif key == ord('a'):
                        self.roi_x = max(0, self.roi_x - 20)
                    elif key == ord('d'):
                        self.roi_x = min(self.width - self.roi_w, self.roi_x + 20)
                    elif key == ord('+') or key == ord('='):
                        self.roi_w = min(self.width, self.roi_w + 40)
                        self.roi_h = min(self.height, self.roi_h + 20)
                    elif key == ord('-'):
                        self.roi_w = max(40, self.roi_w - 40)
                        self.roi_h = max(20, self.roi_h - 20)

        finally:
            self.stop()

    def stop(self):
        # Stop recording if active
        if self.recording:
            self.toggle_recording()
        if hasattr(self, 'arv_camera') and self.arv_camera:
            try:
                self.arv_camera.stop_acquisition()
            except Exception:
                pass
        self.arv_device = None
        self.arv_camera = None
        cv2.destroyAllWindows()
        print(f"\nFinal RPM: {self.current_rpm:.1f}")
        print(f"Total frames: {self.frame_count}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='RPM Meter v8 - GigE Camera')
    parser.add_argument('--camera', type=str, default="169.254.172.226",
                        help="GigE camera IP address")
    parser.add_argument('--threshold', type=int, default=100)
    args = parser.parse_args()

    meter = RPMMeterGUI(camera_address=args.camera)
    meter.threshold = args.threshold
    meter.run()


if __name__ == '__main__':
    main()
