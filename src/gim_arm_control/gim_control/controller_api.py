"""Giao diện nhỏ giữa runner dùng chung và các bộ điều khiển mô-men."""

from abc import ABC, abstractmethod
from typing import Protocol

import numpy as np


class TorqueController(Protocol):
    """Mọi thuật toán phải trả về mô-men tổng tại khớp, đơn vị Nm."""

    last: dict

    def reset(self) -> None:
        ...

    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt: float) -> np.ndarray:
        ...

    def describe(self, q_nominal=None) -> str:
        ...


class ControllerFactory(ABC):
    """Adapter để package thuật toán khai tham số và dựng controller."""

    algorithm_name = "unknown"

    @abstractmethod
    def declare_parameters(self, node) -> None:
        """Khai các ROS parameter riêng của thuật toán."""

    @abstractmethod
    def build(self, node, dynamics, tau_limit, control_hz) -> TorqueController:
        """Dựng controller sau khi URDF và giới hạn mô-men đã sẵn sàng."""


def vector_parameter(
        node, name: str, size: int, *, positive=False,
        nonnegative=False) -> np.ndarray:
    """Đọc parameter vector, cho phép một số vô hướng dùng chung mọi khớp."""
    value = np.asarray(node.get_parameter(name).value, dtype=float)
    if value.size == 1:
        value = np.full(size, float(value.ravel()[0]))
    if value.size != size or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} phải là số hoặc vector {size} phần tử hữu hạn")
    if positive and np.any(value <= 0.0):
        raise ValueError(f"mọi phần tử của {name} phải > 0")
    if nonnegative and np.any(value < 0.0):
        raise ValueError(f"mọi phần tử của {name} phải >= 0")
    return value
