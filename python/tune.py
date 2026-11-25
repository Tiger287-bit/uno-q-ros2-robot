"""底盘调试参数：元数据、编解码与在线写入。

所有参数都在 MCU 的运行期变量里，经 ros_tune_get / ros_tune_set 读写，
索引见 sketch/motion_controller.h 的 motion::TuneParam。默认值就是当前
上板标定值，"恢复默认"只把数字拨回去，不改任何控制逻辑。

对外一律用**物理值**（2.0、6000、0.15），缩放只在写入时按 scale 折算成整数码，
这样前端不必知道任何一个参数的编码规则。

本模块不碰 Bridge：读写都由控制线程执行，HTTP 线程只读 STATE 里的快照。
"""

from collections import namedtuple

Param = namedtuple(
    "Param", "key group label unit minimum maximum step default scale index")

# index 必须与固件 motion::TuneParam 的枚举顺序逐项一致，改顺序会让参数写错位置。
PARAMS = (
    Param("speed_kp", "speed", "速度环 Kp", "1/s", 0.0, 10.0, 0.05, 2.0, 10000, 0),
    Param("speed_ki", "speed", "速度环 Ki", "1/s", 0.0, 5.0, 0.01, 0.5, 10000, 1),
    Param("speed_kd", "speed", "速度环 Kd", "s", 0.0, 2.0, 0.01, 0.0, 10000, 2),
    Param("ramp_cps", "speed", "斜坡", "cps/s", 500, 6000, 100, 6000, 1, 3),
    Param("ff_offset", "feedforward", "前馈截距", "码", 0.0, 100.0, 0.5, 13.0, 10, 4),
    Param("ff_slope", "feedforward", "前馈斜率", "cps/码", 1.0, 100.0, 0.1, 27.3, 100, 5),
    Param("ff_safety", "feedforward", "前馈系数", "", 0.0, 1.5, 0.01, 0.95, 1000, 6),
    Param("breakaway_duty", "feedforward", "起转占空比", "码", 0, 255, 1, 60, 1, 7),
    Param("breakaway_boost", "feedforward", "起转抬升上限", "码", 0, 255, 1, 30, 1, 8),
    Param("heading_kp", "heading", "航向 Kp", "1/s", 0.0, 20.0, 0.05, 1.0, 1000, 9),
    Param("heading_kd", "heading", "航向 Kd", "s", 0.0, 10.0, 0.05, 0.50, 1000, 10),
    Param("heading_limit", "heading", "修正上限", "cps", 0, 3200, 10, 220, 1, 11),
)

GROUPS = (
    ("speed", "轮速环（MCU）", "10 ms 增量式 PI，直接作用于 PWM"),
    ("feedforward", "前馈与起转（MCU）", "稳态占空比模型 13 + |v|/27.3 与静摩擦突破"),
    ("heading", "航向环（MCU）", "直行保向，Kp 1.0 约等于 43.4 cps/°；回正限幅 ±4°/s 在固件里"),
)

BY_KEY = {p.key: p for p in PARAMS}
COUNT = len(PARAMS)


def defaults():
    """全部参数的出厂标定值（物理量）。"""
    return {p.key: p.default for p in PARAMS}


def decode(raw):
    """把 ros_tune_get 的整数数组还原成物理值，长度不足返回 None。"""
    if not isinstance(raw, (list, tuple)) or len(raw) < COUNT:
        return None
    values = {}
    for param in PARAMS:
        code = raw[param.index]
        if not isinstance(code, (int, float)):
            return None
        values[param.key] = round(float(code) / param.scale, 4)
    return values


def encode(key, value):
    """校验并量化单个参数，返回 (整数值, 错误文本)。"""
    param = BY_KEY.get(key)
    if param is None:
        return None, "未知参数：%s" % key
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None, "%s 不是数字" % key
    if not (param.minimum <= number <= param.maximum):
        return None, "%s 超出范围 %.4g ~ %.4g" % (key, param.minimum, param.maximum)
    return int(round(number * param.scale)), ""


def describe(snapshot):
    """组装给前端的完整描述：分组、范围、当前值与默认值。"""
    values = dict(defaults())
    values.update(snapshot or {})
    items = []
    for param in PARAMS:
        items.append({
            "key": param.key,
            "group": param.group,
            "label": param.label,
            "unit": param.unit,
            "min": param.minimum,
            "max": param.maximum,
            "step": param.step,
            "default": param.default,
            "value": round(float(values.get(param.key, param.default)), 4),
        })
    return {
        "groups": [{"key": key, "label": label, "note": note}
                   for key, label, note in GROUPS],
        "params": items,
        "count": COUNT,
    }
