"""libmedia_codec 真解码器实现 (aarch64 无官方 .so, 用 PyAV 代替)。

复制到板子的 SDK 用户目录使用 (见 real/README.md):
    cp real/libmedia_codec_real.py ~/.local/lib/python3.10/site-packages/libmedia_codec.py

H264Decoder 接口与官方 C 库一致:
    decode(data) -> [(frame_bytes, width, height, line_size), ...]
    frame_bytes 为 height*width*3 字节的 RGB24 数据,
    SDK media.py 用 numpy.fromstring(frame).reshape((h, w, 3)) 还原图像。

OpusDecoder 暂不需要 (只做视频显示), 保持空实现。

依赖: python3 -m pip install --user av
"""
import av


class H264Decoder:
    def __init__(self, *args, **kwargs):
        self._codec = av.CodecContext.create("h264", "r")
        self._codec.open()
        self._pts = 0

    def decode(self, data):
        """解码一段 H264 字节流, 返回 [(rgb_bytes, w, h, ls), ...]"""
        results = []
        try:
            for packet in self._codec.parse(data):
                # PyAV 17 (新版 FFmpeg) 的 h264 解码器会按时间戳重排帧,
                # parse() 产出的包没有 pts/dts, 帧会一直憋在重排缓冲里
                # 出不来 —— 必须手工赋递增时间戳。
                packet.pts = self._pts
                packet.dts = self._pts
                self._pts += 1
                for frame in self._codec.decode(packet):
                    img = frame.to_ndarray(format="rgb24")  # (h, w, 3)
                    h, w, _ = img.shape
                    results.append((img.tobytes(), w, h, 0))
        except Exception:
            # 单个坏包不影响后续解码
            pass
        return results


class OpusDecoder:
    def __init__(self, *args, **kwargs):
        pass

    def decode(self, data):
        return []
