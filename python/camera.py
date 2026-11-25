"""摄像头采集：一个后台线程 + 最新一帧 JPEG，供 MJPEG 流与单帧快照共用。

为什么不用 WebUI.expose_camera
    expose_camera 为每个 <img> 客户端各跑一个 capture() 生成器。摄像头是独占
    设备，开两个标签页就会互相抢帧、帧率减半；而且它不暴露"设备被占用/没插"
    的状态，页面只能看到一片空白。这里统一采集一次，所有客户端读同一份帧。

线程模型
    采集线程：read -> imencode -> 写 latest（持 Condition）
    HTTP 线程：frames() 等到 seq 变化才 yield；客户端慢就跳过中间帧，
               服务端不排队、不积压。

失败处理
    UVC 在热插拔、总线复位或带宽不足后会一直 read 失败。这里按 REOPEN_S
    重新打开设备，并把原因写进 status()，页面据此显示"摄像头不可用"。
"""

import os
import threading
import time

import cv2

# 采集与输出尺寸。
#
# 这枚 UVC 摄像头只枚举 1920x1080 和 1280x720 两档 MJPG，没有 640x480：
# 请求 640x480 时驱动会退到 1280x720。所以采集按硬件的档位来，
# 编码前再用 INTER_AREA 缩到 640x360（16:9）。
#
# 为什么要缩：1280x720 的 JPEG 每帧 100~200 KB，15 fps 就是 12~24 Mbit/s，
# 板子的 WiFi 和浏览器解码都吃紧；640x360 每帧 20~40 KB，约 3~5 Mbit/s，
# 而卡片本身只有 360px 宽，缩小后肉眼看不出差别。
DEVICE = "/dev/video0"
CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 720

# 安装方向：这枚摄像头被线束逼着装成侧向，画面需要顺时针转 90° 才是正立。
# 取值 0 / 90 / 180 / 270（顺时针角度）。旋转后宽高互换，所以输出尺寸
# 不能写死宽高，只能按"长边 = OUTPUT_LONG"算。
ROTATE_DEG = 90

# 输出长边（像素）。640 对 360px 宽的卡片足够；短边按比例算，
# 旋转 90° 后是 360x640（竖屏），不旋转时是 640x360。
OUTPUT_LONG = 640

FPS = 15
JPEG_QUALITY = 70

# 连续失败多久重开设备（秒）。
REOPEN_S = 2.0
# 单次 read 的看门狗：超过这个时间没有新帧就认为设备卡死，重开。
READ_TIMEOUT_S = 5.0
# frames() 等新帧的上限（秒）。超时发一个空 boundary 保活，不发旧帧 ——
# 发旧帧会让页面上的画面看起来"还在动"，其实早就断了。
FRAME_WAIT_S = 2.0


class CameraService:
    """摄像头采集与分发。进程内单例，由 main 负责 start/stop。"""

    def __init__(self):
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._thread = None

        self._jpeg = None          # 最新一帧的 JPEG 字节
        self._seq = 0              # 帧序号，每采集一帧 +1
        self._at = 0.0             # 该帧的采集时刻（monotonic）
        self._size = (0, 0)        # 实际分辨率，可能与请求值不同
        self._fps = 0.0            # 实测帧率（指数平均）
        self._clients = 0          # 当前 MJPEG 客户端数
        self._error = ""           # 最近一次失败原因，成功后清空
        self._opened = False

    # ---------------------------------------------------------------- 生命周期
    def start(self):
        """启动采集线程；重复调用无副作用。"""
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="camera",
                                        daemon=True)
        self._thread.start()

    def stop(self):
        """请求采集线程退出并等待其结束。"""
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        if self._thread:
            self._thread.join(timeout=3.0)

    # ---------------------------------------------------------------- 对外读取
    def status(self):
        """返回摄像头状态，供 /api/camera/status 与页面显示。"""
        with self._cond:
            age = round(time.monotonic() - self._at, 2) if self._at else None
            return {
                "ok": self._jpeg is not None and not self._error,
                "opened": self._opened,
                "device": DEVICE,
                "width": self._size[0],
                "height": self._size[1],
                "fps": round(self._fps, 1),
                "clients": self._clients,
                "age_s": age,
                "error": self._error,
            }

    def snapshot(self):
        """返回 (jpeg, seq, age)；还没有帧时 jpeg 为 None。"""
        with self._cond:
            age = time.monotonic() - self._at if self._at else None
            return self._jpeg, self._seq, age

    def frames(self):
        """MJPEG 生成器：每来一帧新画面 yield 一个 multipart 分段。"""
        with self._cond:
            self._clients += 1
        try:
            last = -1
            while not self._stop.is_set():
                jpeg, seq = self._wait_frame(last, FRAME_WAIT_S)
                if jpeg is None:
                    # 断流保活：空 boundary 让浏览器保持连接，等设备恢复。
                    yield b"\r\n"
                    continue
                last = seq
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n"
                       b"Content-Length: " + str(len(jpeg)).encode() +
                       b"\r\n\r\n" + jpeg + b"\r\n")
        finally:
            with self._cond:
                self._clients -= 1

    def _wait_frame(self, last, timeout):
        """等到 seq 与 last 不同，或超时。返回 (jpeg, seq)。"""
        deadline = time.monotonic() + timeout
        with self._cond:
            while self._seq == last and not self._stop.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cond.wait(remaining)
            return self._jpeg, self._seq

    # ---------------------------------------------------------------- 采集线程
    def _run(self):
        cap = None
        last_ok = time.monotonic()
        while not self._stop.is_set():
            if cap is None:
                cap = self._open()
                if cap is None:
                    self._stop.wait(REOPEN_S)
                    continue
                last_ok = time.monotonic()

            ok, frame = cap.read()
            now = time.monotonic()
            if not ok or frame is None:
                # 单次失败很常见（带宽抖动）；连续失败超过看门狗才重开，
                # 否则会陷入"打开-失败-再打开"的抖动，反而一帧都出不来。
                if now - last_ok > READ_TIMEOUT_S:
                    self._set_error("读取超时，正在重开设备")
                    cap.release()
                    cap = None
                    last_ok = now
                else:
                    time.sleep(0.01)
                continue

            last_ok = now
            prepared, width, height = self._prepare(frame)
            encoded = self._encode(prepared)
            if encoded is not None:
                # 上报输出尺寸（旋转并缩放后），不是采集尺寸：
                # 页面上的尺寸才是它真正收到的画面。
                self._publish(encoded, width, height, now)
        if cap is not None:
            cap.release()
        with self._cond:
            self._opened = False

    def _open(self):
        """打开设备并按 MJPG/分辨率/帧率配置；失败时写 error 并返回 None。"""
        if not os.path.exists(DEVICE):
            self._set_error("找不到 %s（摄像头未插入或被占用）" % DEVICE)
            return None
        try:
            cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
        except Exception as exc:                     # OpenCV 抛出的是 cv2.error
            self._set_error("打开失败：%s" % exc)
            return None
        if not cap.isOpened():
            cap.release()
            self._set_error("设备打不开（可能被别的进程占用）")
            return None

        # FOURCC 必须在分辨率之前设置：UVC 会按当前格式挑选最接近的尺寸。
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FPS)
        # 只保留最新一帧：默认缓冲区会把旧帧排在前面，画面延迟能到几百毫秒。
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        with self._cond:
            self._opened = True
            self._error = ""
            self._size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                          int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        return cap

    def _prepare(self, frame):
        """按安装方向旋转，再按长边缩放到输出尺寸。返回 (frame, w, h)。

        旋转放在缩放之前：ROTATE_90_* 是整块内存重排，成本与像素数成正比；
        先转再缩和先缩再转的总开销接近，但先转能保证缩放始终是"长边对齐"，
        不必为横竖两种方向各写一套目标尺寸。
        """
        if ROTATE_DEG == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif ROTATE_DEG == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif ROTATE_DEG == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        height, width = frame.shape[:2]
        longest = max(width, height)
        if longest != OUTPUT_LONG:
            scale = OUTPUT_LONG / float(longest)
            size = (max(1, round(width * scale)), max(1, round(height * scale)))
            frame = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
        return frame, frame.shape[1], frame.shape[0]

    def _encode(self, frame):
        """把准备好的帧编码成 JPEG；失败返回 None（不影响下一帧）。

        缩放放在服务端而不是交给前端 CSS：MJPEG 是逐帧完整 JPEG，
        传输量由编码尺寸决定，CSS 缩小只省显示像素、不省带宽。
        """
        try:
            ok, buffer = cv2.imencode(
                ".jpg", frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        except cv2.error:
            return None
        return buffer.tobytes() if ok else None

    def _publish(self, jpeg, width, height, now):
        """发布一帧，并更新实测帧率。"""
        with self._cond:
            if self._at:
                dt = now - self._at
                if dt > 0:
                    # 指数平均：单帧抖动不该让页面上的 fps 上下跳。
                    inst = 1.0 / dt
                    self._fps = inst if not self._fps else self._fps * 0.85 + inst * 0.15
            self._jpeg = jpeg
            self._seq += 1
            self._at = now
            self._size = (width, height)
            self._error = ""
            self._cond.notify_all()

    def _set_error(self, message):
        with self._cond:
            self._error = message
            self._opened = False
            self._cond.notify_all()


# 进程内单例。采集线程由 main 控制启停，模块导入本身不产生副作用。
CAMERA = CameraService()
