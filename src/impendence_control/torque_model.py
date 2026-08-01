"""电机电流—力矩线性模型计算。

输入电流单位为 A，斜率单位为 N·m/A，截距和输出力矩单位为 N·m。
每个关节的斜率和截距由 ``JOINT_CONFIGS`` 提供。
"""


def current_to_torque(
    current_a: float,
    slope_nm_per_a: float,
    intercept_nm: float,
) -> float:
    """使用指定关节的标定参数将电流转换为力矩。"""
    return float(slope_nm_per_a) * float(current_a) + float(intercept_nm)
