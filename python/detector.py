"""物品检测：Arduino 官方 object_detection brick + 单线程推理。

与官方 brick 的关系
    `arduino.app_bricks.object_detection.ObjectDetection` 通过 HTTP 把 JPEG
    发给同 compose 网络里的 ei-models-runner 容器（1337 端口），模型是
    Edge Impulse 的 `yolo-x-nano.eim`。本模块只负责"喂帧、限速、缓存结果"，
    模型与推理都在那个容器里。

为什么要限速
    实测单帧推理 0.55~0.8 s（含 JPEG 解码与 HTTP 往返，整机满栈运行时更慢），
    不可能跟 15 fps 的视频同频。这里只取最新
    一帧，推理期间产生的帧直接丢弃 —— 宁可结果旧一点，也不要排队积压，
    否则画面在动、框停在几秒前，看起来像"检测失灵"。

为什么失败不抛异常
    runner 容器没起来、模型还在加载时 ObjectDetection() 会抛。这里只记录
    错误并按 RETRY_S 退避重试，绝不把异常冒到 main 的启动流程：底盘控制
    和摄像头不该因为检测不可用而起不来。
"""

import threading
import time

from camera import CAMERA

# 两次推理之间的最小间隔。给控制环和视频编码留出 CPU。
MIN_INTERVAL_S = 0.2
# 初始化/推理失败后的重试间隔。runner 启动时要加载模型，可能十几秒。
RETRY_S = 5.0
# 置信度下限。0.35 对演示比较合适：既能滤掉大部分误检，又不至于漏掉
# 桌面上偏小的物体。
CONFIDENCE = 0.35


class DetectorService:
    """检测服务。进程内单例，由 main 负责 start/stop。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._motion_active = threading.Event()
        self._thread = None
        self._engine = None

        self._objects = []      # [{label, score, box:[x1,y1,x2,y2]}]，像素坐标
        self._width = 0         # 检测所用图像的宽（= camera 输出宽）
        self._height = 0
        self._at = 0.0          # 最近一次成功推理的时刻（time.time）
        self._ms = 0            # 最近一次推理耗时
        self._frames = 0        # 累计推理帧数
        self._last_seq = -1     # 已推理过的相机帧号，避免重复算同一帧
        self._error = ""
        self._ready = False
        self._labels = []

    # ---------------------------------------------------------------- 生命周期
    def start(self):
        """启动推理线程；重复调用无副作用。"""
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="detector",
                                        daemon=True)
        self._thread.start()

    def stop(self):
        """请求线程退出并等待其结束。"""
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    # ---------------------------------------------------------------- 对外读取
    def set_motion_active(self, active):
        if active:
            self._motion_active.set()
        else:
            self._motion_active.clear()

    def status(self):
        """返回检测结果与状态，供 /api/detect 使用。"""
        with self._lock:
            age = round(time.time() - self._at, 2) if self._at else None
            paused = self._motion_active.is_set()
            return {
                "ok": self._ready and not self._error,
                "paused": paused,
                "ready": self._ready,
                "model": "yolo-x-nano",
                "labels": list(self._labels),
                "objects": [] if paused else list(self._objects),
                "width": self._width,
                "height": self._height,
                "age_s": age,
                "infer_ms": self._ms,
                "frames": self._frames,
                "error": self._error,
            }

    # ---------------------------------------------------------------- 推理线程
    def _run(self):
        while not self._stop.is_set():
            if self._motion_active.is_set():
                self._stop.wait(0.15)
                continue
            engine = self._ensure_engine()
            if engine is None:
                self._stop.wait(RETRY_S)
                continue

            jpeg, seq, _age = CAMERA.snapshot()
            if jpeg is None or seq == self._last_seq:
                # 没有新画面：等下一轮，不做无意义的重复推理。
                self._stop.wait(0.05)
                continue
            if self._motion_active.is_set():
                continue
            self._last_seq = seq

            started = time.monotonic()
            try:
                result = engine.detect(jpeg, "jpg", confidence=CONFIDENCE)
            except Exception as exc:
                # 推理服务重启、模型卸载都会走到这里。丢掉引擎让下一轮重建，
                # 而不是让线程带着一个坏连接一直空转。
                self._set_error("推理失败：%s" % exc)
                with self._lock:
                    self._engine = None
                    self._ready = False
                self._stop.wait(RETRY_S)
                continue

            self._publish(result, int((time.monotonic() - started) * 1000))
            time.sleep(MIN_INTERVAL_S)

    def _ensure_engine(self):
        """创建 ObjectDetection 实例；不可用时返回 None 并记录原因。"""
        if self._engine is not None:
            return self._engine
        try:
            # 延迟导入：runner 不可用时这里会抛，但不该影响模块导入。
            from arduino.app_bricks.object_detection import ObjectDetection

            engine = ObjectDetection(confidence=CONFIDENCE)
        except Exception as exc:
            self._set_error("检测服务不可用：%s" % exc)
            return None

        labels = []
        try:
            info = engine.get_model_info()
            labels = list(getattr(info, "classes", None) or [])
        except Exception:
            # 模型信息只用于页面展示，拿不到不影响推理。
            labels = []

        with self._lock:
            self._engine = engine
            self._labels = labels
            self._ready = True
            self._error = ""
        return engine

    def _publish(self, result, elapsed_ms):
        """把 brick 的返回值折成前端友好的结构。

        brick 返回：
            {"detection": [{"class_name": "person",
                            "confidence": "85.32",       # 百分比字符串
                            "bounding_box_xyxy": [x1,y1,x2,y2]}]}
        坐标是相对**输入图像**的像素；输入就是 camera 输出的 360x640，
        所以前端直接把 box 除以 width/height 即可。
        """
        objects = []
        if isinstance(result, dict):
            for item in result.get("detection") or []:
                box = item.get("bounding_box_xyxy") or []
                if len(box) != 4:
                    continue
                try:
                    score = float(item.get("confidence", 0)) / 100.0
                    coords = [round(float(value), 1) for value in box]
                except (TypeError, ValueError):
                    continue
                objects.append({
                    "label": str(item.get("class_name") or "?"),
                    "score": round(score, 3),
                    "box": coords,
                })

        camera = CAMERA.status()
        with self._lock:
            self._objects = objects
            self._width = camera["width"]
            self._height = camera["height"]
            self._at = time.time()
            self._ms = elapsed_ms
            self._frames += 1
            self._error = ""

    def _set_error(self, message):
        with self._lock:
            self._error = message


# 进程内单例。线程由 main 控制启停，模块导入本身不产生副作用。
DETECTOR = DetectorService()
