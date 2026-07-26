# -*- coding: utf-8 -*-
"""海康触发采集与 TensorRT YOLO 推理。
Hikvision triggered acquisition and TensorRT YOLO inference.
"""

import argparse
import sys
import ctypes
import threading
import time
import socket
import os
import queue
from ctypes import *
from datetime import datetime
from pathlib import Path
import numpy as np
import cv2
import json
from ultralytics import YOLO

winfun_ctype = CFUNCTYPE
MVS_PYTHON_PATH = Path(
    os.environ.get("MVS_PYTHON_PATH", "/opt/MVS/Samples/64/Python/MvImport")
).expanduser()
if not MVS_PYTHON_PATH.is_dir():
    raise ImportError(
        "未找到海康 MVS Python SDK。请安装官方 MVS SDK，或通过 "
        "MVS_PYTHON_PATH 指定 MvImport 目录。"
    )
sys.path.append(str(MVS_PYTHON_PATH))
from MvCameraControl_class import *

class HikVisionCamera:
    def __init__(self, device_id=0, engine_path="./best.engine", udp_host='127.0.0.1', udp_port=11000,
                 conf_threshold=0.5, show_result=False, save_image=False, save_annotated=False,
                 save_directory=None, trigger_wait_timeout_s=0.5):
        self.device_id = device_id
        self.cam = None
        self.is_grabbing = False
        self.frame_count = 0
        
        MvCamera.MV_CC_Initialize()
        
        stFrame = POINTER(MV_FRAME_OUT)
        self.FrameCallback = winfun_ctype(None, stFrame, c_void_p, c_bool)
        self.callback_func = self.FrameCallback(self._image_callback)
        
        self.model = YOLO(engine_path)
        self.conf_threshold = conf_threshold
        self.class_names = ['drone']
        self.show_result = show_result
        self.save_image = save_image
        self.save_annotated = save_annotated
        self.save_directory = os.path.abspath(save_directory or "image")
        
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_target = (udp_host, udp_port)
        self.udp_send_lock = threading.Lock()

        # 单任务触发状态 / Single in-flight trigger state.
        self.trigger_lock = threading.Lock()
        self.trigger_state = "idle"
        self.current_trigger_metadata = None
        self.trigger_state_since = 0.0
        self.trigger_wait_timeout_s = trigger_wait_timeout_s
        self.last_busy_warning_monotonic = 0.0
        self.legacy_trigger_sequence = 0
        self.last_camera_frame_id = None

        # 异步图像保存 / Asynchronous image saving.
        self.image_save_queue = None
        self.image_save_thread = None
        
        print(f"TensorRT引擎加载: {engine_path}")
        print(f"实时显示: {'开启' if show_result else '关闭'}")
        print(f"图像保存: {'开启' if save_image else '关闭'}")
        
        if self.save_image:
            os.makedirs(self.save_directory, exist_ok=True)
            try:
                os.chmod(self.save_directory, 0o775)
            except OSError as error:
                print(f"警告: 无法调整图片目录权限: {error}")
            self.image_save_queue = queue.Queue(maxsize=2)
            self.image_save_thread = threading.Thread(
                target=self._image_save_worker, name="image-save-worker", daemon=False)
            self.image_save_thread.start()
            print(f"图片保存目录: {self.save_directory}")
            print("图片异步保存线程已启动")

    @staticmethod
    def _unix_timestamp_us():
        return time.time_ns() // 1000

    def make_legacy_trigger_metadata(self):
        """生成兼容触发元数据 / Build legacy trigger metadata."""
        with self.trigger_lock:
            self.legacy_trigger_sequence += 1
            trigger_sequence = self.legacy_trigger_sequence

        return {
            "schema_version": 2,
            "trigger_sequence": trigger_sequence,
            "lidar_end_timestamp_us": 0,
            "trigger_source": "legacy"
        }

    def _take_trigger_metadata(self):
        """领取触发元数据 / Claim trigger metadata."""
        with self.trigger_lock:
            if self.trigger_state != "wait_frame" or self.current_trigger_metadata is None:
                return None
            metadata = self.current_trigger_metadata
            self.trigger_state = "processing"
            self.trigger_state_since = time.monotonic()
            return metadata

    def _finish_trigger(self, metadata):
        with self.trigger_lock:
            if self.trigger_state == "processing" and self.current_trigger_metadata is metadata:
                self.current_trigger_metadata = None
                self.trigger_state = "idle"
                self.trigger_state_since = 0.0

    def _restart_capture_stream(self):
        """重启采集流 / Restart the capture stream."""
        stop_ret = self.cam.MV_CC_StopGrabbing()
        if stop_ret != 0:
            print(f"警告: 超时恢复时停止取流失败! ret[0x{stop_ret:x}]")

        start_ret = self.cam.MV_CC_StartGrabbing()
        if start_ret != 0:
            print(f"警告: 超时恢复时重新开始取流失败! ret[0x{start_ret:x}]")
            return False

        print("相机触发等待超时，采集流已重新启动")
        return True

    def _recover_wait_timeout(self, incoming_metadata):
        """恢复触发超时 / Recover a trigger timeout."""
        now = time.monotonic()
        with self.trigger_lock:
            if self.trigger_state != "wait_frame" or self.current_trigger_metadata is None:
                return False

            elapsed_s = now - self.trigger_state_since
            if elapsed_s <= self.trigger_wait_timeout_s:
                return False

            expired_metadata = self.current_trigger_metadata
            self.current_trigger_metadata = None
            self.trigger_state = "recovering"
            self.trigger_state_since = now

        reason = (f"等待相机图像回调超时 {elapsed_s:.3f}s，"
                  f"sequence={expired_metadata.get('trigger_sequence', 0)}")
        self._send_trigger_failure(expired_metadata, "frame_timeout", reason)
        recovered = self._restart_capture_stream()

        with self.trigger_lock:
            if self.trigger_state == "recovering":
                self.trigger_state = "idle"
                self.trigger_state_since = 0.0

        current_reason = "相机采集流刚完成超时恢复，本次触发跳过"
        if not recovered:
            current_reason = "相机采集流超时恢复失败，本次触发跳过"
        self._send_trigger_failure(incoming_metadata, "capture_recovering", current_reason)
        return True

    def _build_detection_message(self, metadata, source_timestamp_us, host_callback_timestamp_us,
                                 inference_start_timestamp_us, inference_end_timestamp_us,
                                 inference_done_timestamp_us, camera_frame_id, detections):
        metadata = metadata or {}
        return {
            "schema_version": 2,
            "trigger_sequence": int(metadata.get("trigger_sequence", 0)),
            "lidar_end_timestamp_us": int(metadata.get("lidar_end_timestamp_us", 0)),
            "lidar_callback_timestamp_us": int(metadata.get("lidar_callback_timestamp_us", 0)),
            "trigger_sent_timestamp_us": int(metadata.get("trigger_sent_timestamp_us", 0)),
            "trigger_received_timestamp_us": int(metadata.get("trigger_received_timestamp_us", 0)),
            "trigger_command_timestamp_us": int(metadata.get("trigger_command_timestamp_us", 0)),
            # 兼容时间戳 / Compatibility timestamp.
            "ts": int(source_timestamp_us),
            "source_timestamp_us": int(source_timestamp_us),
            "host_callback_timestamp_us": int(host_callback_timestamp_us),
            "inference_start_timestamp_us": int(inference_start_timestamp_us),
            "inference_end_timestamp_us": int(inference_end_timestamp_us),
            "inference_done_timestamp_us": int(inference_done_timestamp_us),
            "camera_frame_id": int(camera_frame_id),
            "d": detections
        }

    def _send_trigger_failure(self, metadata, status, error_message):
        now_us = self._unix_timestamp_us()
        data = self._build_detection_message(metadata, now_us, now_us, 0, 0, now_us, 0, [])
        data["status"] = status
        data["error"] = str(error_message)
        self._send_udp(data)

    def _image_save_worker(self):
        """异步保存图像 / Save images asynchronously."""
        save_queue = self.image_save_queue
        while True:
            save_task = save_queue.get()
            try:
                if save_task is None:
                    return

                filename, image = save_task
                if not cv2.imwrite(filename, image):
                    print(f"图像保存失败: {filename}")
                else:
                    try:
                        os.chmod(filename, 0o664)
                    except OSError as error:
                        print(f"警告: 无法调整图片文件权限: {error}")
            except Exception as error:
                print(f"图像保存异常: {error}")
            finally:
                save_queue.task_done()

    def _enqueue_image_save(self, filename, image):
        """提交保存任务 / Queue an image-save task."""
        if self.image_save_queue is None:
            raise RuntimeError("图片保存线程未启动")
        self.image_save_queue.put((filename, image))

    def _stop_image_save_worker(self):
        """停止保存线程 / Stop the image-save worker."""
        save_queue = self.image_save_queue
        save_thread = self.image_save_thread
        if save_queue is None or save_thread is None:
            return

        save_queue.join()
        save_queue.put(None)
        save_thread.join()
        self.image_save_queue = None
        self.image_save_thread = None
        print("图片异步保存线程已停止，待保存图片已全部写入")
    
    def _image_callback(self, frame_ptr, user_ptr, auto_free):
        frame = None
        user_obj = None
        rgb_image = None
        detections = []
        camera_frame_id = 0
        camera_host_timestamp_raw = 0
        host_callback_timestamp_us = self._unix_timestamp_us()
        source_timestamp_us = host_callback_timestamp_us
        inference_start_timestamp_us = 0
        inference_end_timestamp_us = 0
        status = "ok"
        error_message = ""
        metadata = self._take_trigger_metadata()

        if metadata is None:
            status = "unmatched_frame"
            error_message = "收到没有对应触发元数据的相机帧"
            print(f"警告: {error_message}，本帧不参与 LiDAR 精确匹配")
            metadata = {
                "schema_version": 2,
                "trigger_sequence": 0,
                "lidar_end_timestamp_us": 0,
                "trigger_source": "unmatched"
            }

        try:
            frame = cast(frame_ptr, POINTER(MV_FRAME_OUT)).contents
            user_obj = ctypes.cast(user_ptr, ctypes.py_object).value

            self.frame_count += 1
            camera_frame_id = int(frame.stFrameInfo.nFrameNum)
            camera_host_timestamp_raw = int(getattr(frame.stFrameInfo, "nHostTimeStamp", 0))

            if self.last_camera_frame_id is not None:
                expected_frame_id = (self.last_camera_frame_id + 1) & 0xffffffff
                if camera_frame_id != expected_frame_id:
                    print(f"警告: 相机帧号不连续，上一帧={self.last_camera_frame_id}，"
                          f"当前帧={camera_frame_id}")
            self.last_camera_frame_id = camera_frame_id

            raw_bytes = string_at(frame.pBufAddr, frame.stFrameInfo.nFrameLen)
            width = frame.stFrameInfo.nWidth
            height = frame.stFrameInfo.nHeight

            bayer_image = np.frombuffer(raw_bytes, dtype=np.uint8).reshape(height, width)
            rgb_image = cv2.cvtColor(bayer_image, cv2.COLOR_BAYER_RG2RGB)

            inference_start_timestamp_us = self._unix_timestamp_us()
            results = self.model.predict(rgb_image, conf=self.conf_threshold, verbose=False)
            inference_end_timestamp_us = self._unix_timestamp_us()
            result = results[0]

            if result.boxes is not None and len(result.boxes) > 0:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf.cpu().numpy()[0])
                    cls_id = int(box.cls.cpu().numpy()[0])
                    detections.append({
                        "c": cls_id,
                        "cn": self.class_names[cls_id] if cls_id < len(self.class_names) else str(cls_id),
                        "conf": round(conf, 3),
                        "bbox": [int(x1), int(y1), int(x2), int(y2)]
                    })

        except Exception as e:
            if status == "ok":
                status = "inference_failed"
                error_message = str(e)
            else:
                error_message = f"{error_message}; 推理异常: {e}"
            detections = []
            print(f"图像回调或推理异常: {e}")
        finally:
            # 每帧均发送结果 / Emit one result per frame.
            inference_done_timestamp_us = self._unix_timestamp_us()
            data = self._build_detection_message(metadata,
                                                 source_timestamp_us,
                                                 host_callback_timestamp_us,
                                                 inference_start_timestamp_us,
                                                 inference_end_timestamp_us,
                                                 inference_done_timestamp_us,
                                                 camera_frame_id,
                                                 detections)
            data["status"] = status
            data["camera_host_timestamp_raw"] = camera_host_timestamp_raw
            if error_message:
                data["error"] = error_message
            self._send_udp(data)

        try:
            print(f"[{source_timestamp_us}] 检测到 {len(detections)} 个目标 "
                  f"(FrameID: {camera_frame_id}, TriggerSequence: {metadata.get('trigger_sequence', 0)})")
            for det in detections:
                print(f"  - {det['cn']}: 置信度={det['conf']:.3f}, bbox={det['bbox']}")

            annotated_image = None
            needs_annotated_image = self.show_result or (self.save_image and self.save_annotated)
            if rgb_image is not None and needs_annotated_image:
                annotated_image = rgb_image.copy()
                for det in detections:
                    x1, y1, x2, y2 = det['bbox']
                    conf = det['conf']
                    label = f"{det['cn']} {conf:.2f}"
                    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            if self.show_result and annotated_image is not None:
                display_image = cv2.resize(annotated_image, (0, 0), fx=0.5, fy=0.5)
                cv2.imshow("YOLO检测结果", display_image)
                cv2.waitKey(1)

            if self.save_image and rgb_image is not None:
                timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                filename = os.path.join(
                    self.save_directory, f"detect_{timestamp_str}_{camera_frame_id}.jpg")
                image_to_save = annotated_image if self.save_annotated else rgb_image
                self._enqueue_image_save(filename, image_to_save)

        except Exception as e:
            print(f"图像显示或保存异常: {e}")
        finally:
            if frame is not None and not auto_free and user_ptr is not None and user_obj is not None:
                user_obj.MV_CC_FreeImageBuffer(frame)
            self._finish_trigger(metadata)
    
    def _send_udp(self, data):
        try:
            message = json.dumps(data, separators=(',', ':')).encode('utf-8')
            with self.udp_send_lock:
                self.udp_socket.sendto(message, self.udp_target)
        except Exception as e:
            print(f"UDP发送失败: {e}")
    
    def connect(self):
        device_list = MV_CC_DEVICE_INFO_LIST()
        layer_type = (MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE |
                      MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE)
        
        ret = MvCamera.MV_CC_EnumDevices(layer_type, device_list)
        if ret != 0:
            raise Exception(f"枚举设备失败! ret[0x{ret:x}]")
        
        if device_list.nDeviceNum == 0:
            raise Exception("未找到任何设备!")
        
        if self.device_id >= device_list.nDeviceNum:
            raise Exception(f"设备ID {self.device_id} 超出范围! 共找到 {device_list.nDeviceNum} 个设备")
        
        print(f"找到 {device_list.nDeviceNum} 个设备，正在连接设备 {self.device_id}")
        
        self.cam = MvCamera()
        device_info = cast(device_list.pDeviceInfo[self.device_id], POINTER(MV_CC_DEVICE_INFO)).contents
        ret = self.cam.MV_CC_CreateHandle(device_info)
        if ret != 0:
            raise Exception(f"创建相机句柄失败! ret[0x{ret:x}]")
        
        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            raise Exception(f"打开设备失败! ret[0x{ret:x}]")
        
        if (device_info.nTLayerType == MV_GIGE_DEVICE or 
            device_info.nTLayerType == MV_GENTL_GIGE_DEVICE):
            packet_size = self.cam.MV_CC_GetOptimalPacketSize()
            if packet_size > 0:
                ret = self.cam.MV_CC_SetIntValue("GevSCPSPacketSize", packet_size)
                if ret != 0:
                    print(f"警告: 设置包大小失败! ret[0x{ret:x}]")
        
        ret = self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_ON)
        if ret != 0:
            raise Exception(f"设置触发模式失败! ret[0x{ret:x}]")
        
        ret = self.cam.MV_CC_SetEnumValueByString("TriggerSource", "Software")
        if ret != 0:
            raise Exception(f"设置触发源失败! ret[0x{ret:x}]")
        
        print("相机连接成功 (软件触发模式)")

    def warm_up(self):
        """预热推理引擎 / Warm up the inference engine."""
        print("正在预热 TensorRT 引擎...")
        warmup_image = np.zeros((640, 640, 3), dtype=np.uint8)
        self.model.predict(warmup_image, conf=self.conf_threshold, verbose=False)
        print("TensorRT 引擎预热完成")
    
    def trigger_capture(self, metadata=None):
        if self.cam is None:
            raise Exception("相机未连接!")

        if not self.is_grabbing:
            raise Exception("相机未开始取流!")

        if metadata is None:
            metadata = self.make_legacy_trigger_metadata()

        if self._recover_wait_timeout(metadata):
            return False

        now = time.monotonic()
        with self.trigger_lock:
            if self.trigger_state != "idle":
                state = self.trigger_state
                elapsed_s = now - self.trigger_state_since
                accepted_metadata = None
            else:
                accepted_metadata = dict(metadata)
                self.current_trigger_metadata = accepted_metadata
                self.trigger_state = "wait_frame"
                self.trigger_state_since = now

        if accepted_metadata is None:
            message = f"相机忙碌，状态={state}，持续={elapsed_s:.3f}s"
            self._send_trigger_failure(metadata, "camera_busy", message)
            if now - self.last_busy_warning_monotonic >= 1.0:
                print(f"提示: {message}，跳过新的硬件触发")
                self.last_busy_warning_monotonic = now
            return False

        accepted_metadata["trigger_command_timestamp_us"] = self._unix_timestamp_us()
        ret = self.cam.MV_CC_SetCommandValue("TriggerSoftware")
        if ret != 0:
            with self.trigger_lock:
                if self.trigger_state == "wait_frame" and self.current_trigger_metadata is accepted_metadata:
                    self.current_trigger_metadata = None
                    self.trigger_state = "idle"
                    self.trigger_state_since = 0.0
            self._send_trigger_failure(metadata, "trigger_failed", f"ret[0x{ret:x}]")
            raise Exception(f"发送触发命令失败! ret[0x{ret:x}]")
        return True
    
    def start_capture(self):
        if self.cam is None:
            raise Exception("请先连接相机!")
        
        ret = self.cam.MV_CC_RegisterImageCallBackEx2(self.callback_func, ctypes.py_object(self.cam), True)
        if ret != 0:
            raise Exception(f"注册图像回调失败! ret[0x{ret:x}]")
        
        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise Exception(f"开始取流失败! ret[0x{ret:x}]")
        
        self.is_grabbing = True
        print("开始图像采集 (等待触发信号)")
    
    def stop_capture(self):
        if not self.is_grabbing:
            return
        
        ret = self.cam.MV_CC_StopGrabbing()
        if ret != 0:
            print(f"停止取流失败! ret[0x{ret:x}]")
        
        self.is_grabbing = False
        print("图像采集已停止")
    
    def disconnect(self):
        if self.is_grabbing:
            self.stop_capture()

        self._stop_image_save_worker()
        
        if self.show_result:
            cv2.destroyAllWindows()
        
        if self.cam is not None:
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
            self.cam = None
        
        print("相机连接已断开")
    
    def __del__(self):
        self.disconnect()
        if hasattr(self, 'udp_socket'):
            self.udp_socket.close()
        MvCamera.MV_CC_Finalize()

def udp_listener(camera, host='127.0.0.1', port=9999):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    print(f"UDP监听已启动: {host}:{port}")
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            trigger_received_timestamp_us = camera._unix_timestamp_us()
            message = data.decode('utf-8').strip()

            if message == "1":
                metadata = camera.make_legacy_trigger_metadata()
            else:
                try:
                    payload = json.loads(message)
                    if not isinstance(payload, dict):
                        raise ValueError("JSON 顶层必须是对象")

                    schema_version = int(payload.get("schema_version", 0))
                    trigger_sequence = int(payload["trigger_sequence"])
                    lidar_end_timestamp_us = int(payload["lidar_end_timestamp_us"])
                    if schema_version != 2:
                        raise ValueError(f"不支持的 schema_version={schema_version}")
                    if trigger_sequence <= 0:
                        raise ValueError("trigger_sequence 必须为正整数")
                    if lidar_end_timestamp_us <= 0:
                        raise ValueError("lidar_end_timestamp_us 必须为正整数")

                    metadata = {
                        "schema_version": schema_version,
                        "trigger_sequence": trigger_sequence,
                        "lidar_end_timestamp_us": lidar_end_timestamp_us,
                        "lidar_callback_timestamp_us": int(payload.get("lidar_callback_timestamp_us", 0)),
                        "trigger_sent_timestamp_us": int(payload.get("trigger_sent_timestamp_us", 0)),
                        "trigger_source": "lidar_v2"
                    }
                except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
                    print(f"忽略无效相机触发消息: {e}; 原始消息={message[:200]!r}")
                    continue

            metadata["trigger_received_timestamp_us"] = trigger_received_timestamp_us

            try:
                if camera.trigger_capture(metadata):
                    print(f"[{datetime.now().strftime('%H:%M:%S.%f')}] 收到触发信号，执行拍摄 "
                          f"(sequence={metadata['trigger_sequence']}, "
                          f"lidar_end_us={metadata['lidar_end_timestamp_us']})")
            except Exception as e:
                print(f"触发失败: {e}")
        except Exception as e:
            print(f"UDP监听异常: {e}")
            break
    
    sock.close()

def load_detection_settings(path):
    """读取检测配置 / Load detection settings."""
    defaults = {
        "udp_port": 11000,
        "minimum_confidence": 0.5,
        "trigger_wait_timeout_s": 0.5,
    }
    try:
        with open(path, "r", encoding="utf-8") as stream:
            root = json.load(stream)
        detection = root.get("detection", {})
        settings = {key: detection.get(key, value) for key, value in defaults.items()}
        settings["udp_port"] = int(settings["udp_port"])
        settings["minimum_confidence"] = float(settings["minimum_confidence"])
        settings["trigger_wait_timeout_s"] = float(settings["trigger_wait_timeout_s"])
        if not (1 <= settings["udp_port"] <= 65535):
            raise ValueError("udp_port 越界")
        if not (0.0 <= settings["minimum_confidence"] <= 1.0):
            raise ValueError("minimum_confidence 越界")
        if settings["trigger_wait_timeout_s"] <= 0.0:
            raise ValueError("触发等待超时必须为正数")
        return settings
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(f"警告: 无法加载 {path}: {error}；使用安全默认检测参数")
        return defaults

def _requires_real_mount(path):
    normalized = os.path.abspath(path)
    return normalized in ("/media", "/mnt", "/run/media") \
        or normalized.startswith("/media/") \
        or normalized.startswith("/mnt/") \
        or normalized.startswith("/run/media/")

def _has_non_root_mount_ancestor(path):
    current = os.path.abspath(path)
    while current != os.path.dirname(current):
        if current != "/" and os.path.ismount(current):
            return True
        current = os.path.dirname(current)
    return False

def resolve_image_save_directory(app_config_path):
    """解析图像目录 / Resolve the image directory."""
    default_data_base_dir = "data"
    configured = default_data_base_dir

    try:
        with open(app_config_path, "r", encoding="utf-8") as stream:
            root = json.load(stream)
        if not isinstance(root, dict):
            raise ValueError("JSON 顶层必须是对象")
        configured_value = root.get("data_base_dir", configured)
        if not isinstance(configured_value, str) or not configured_value.strip():
            raise ValueError("data_base_dir 为空或类型错误")
        configured = configured_value
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(f"警告: 无法加载 {app_config_path}: {error}；使用默认数据目录")

    data_base_dir = Path(configured.strip()).expanduser()
    if not data_base_dir.is_absolute():
        project_directory = Path(app_config_path).resolve().parent.parent
        data_base_dir = project_directory / data_base_dir
    data_base_dir = str(data_base_dir.resolve())
    if _requires_real_mount(data_base_dir) and not _has_non_root_mount_ancestor(data_base_dir):
        raise RuntimeError(f"配置的数据目录不在有效挂载点下: {data_base_dir}")

    session_suffix = datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(data_base_dir, "camera_images", session_suffix)


def parse_args():
    project_directory = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description="海康相机触发与 TensorRT YOLO 检测")
    parser.add_argument(
        "--engine",
        type=Path,
        default=Path(__file__).resolve().with_name("best_20260715.engine"),
        help="TensorRT engine 路径；可用 convert_to_engine.py 从公开 PT 权重导出",
    )
    parser.add_argument(
        "--tracker-config",
        type=Path,
        default=project_directory / "config" / "tracker.json",
        help="跟踪参数 JSON 路径",
    )
    parser.add_argument(
        "--app-config",
        type=Path,
        default=project_directory / "config" / "app_config.json",
        help="数据目录配置 JSON 路径",
    )
    parser.add_argument("--device-id", type=int, default=0, help="海康相机设备编号")
    parser.add_argument("--udp-host", default="127.0.0.1", help="检测结果接收地址")
    parser.add_argument("--show-result", action="store_true", help="显示实时检测画面")
    parser.add_argument("--no-save-image", action="store_true", help="不保存相机图像")
    parser.add_argument("--image-dir", type=Path, help="覆盖相机图像保存目录")
    return parser.parse_args()

def main():
    camera = None
    try:
        args = parse_args()
        engine_path = args.engine.expanduser().resolve()
        tracker_config_path = args.tracker_config.expanduser().resolve()
        app_config_path = args.app_config.expanduser().resolve()
        if not engine_path.is_file():
            raise FileNotFoundError(
                f"TensorRT engine 不存在: {engine_path}；请先运行 convert_to_engine.py"
            )

        detection_settings = load_detection_settings(str(tracker_config_path))
        save_image = not args.no_save_image
        if args.image_dir is not None:
            image_save_directory = str(args.image_dir.expanduser().resolve())
        else:
            image_save_directory = (
                resolve_image_save_directory(str(app_config_path)) if save_image else None
            )
        
        camera = HikVisionCamera(
            device_id=args.device_id,
            engine_path=str(engine_path),
            udp_host=args.udp_host,
            udp_port=detection_settings["udp_port"],
            conf_threshold=detection_settings["minimum_confidence"],
            show_result=args.show_result,
            save_image=save_image,
            save_directory=image_save_directory,
            trigger_wait_timeout_s=detection_settings["trigger_wait_timeout_s"])
        camera.warm_up()
        camera.connect()
        camera.start_capture()
        
        udp_thread = threading.Thread(target=udp_listener, args=(camera,), daemon=True)
        udp_thread.start()
        
        print("程序运行中... 按Ctrl+C停止")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n收到退出信号")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        if camera:
            camera.disconnect()
        print("程序结束")

if __name__ == "__main__":
    main()
