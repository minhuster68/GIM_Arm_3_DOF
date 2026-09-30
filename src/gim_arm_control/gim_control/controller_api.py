"""Giao diện dùng chung giữa runner và các bộ điều khiển mô-men."""

from abc import ABC, abstractmethod
from typing import Protocol

import numpy as np


class TorqueController(Protocol):
    """Giao diện mà PID, LQR, MPC và SMC phải triển khai."""

    last: dict

    def reset(self) -> None:
        ...

    def compute(self, q, qd, q_ref, qd_ref, qdd_ref, dt: float) -> np.ndarray:
        ...

    def describe(self, q_nominal=None) -> str:
        ...


class ControllerFactory(ABC):
    """Adapter để mỗi package khai tham số và dựng controller của nó."""

    algorithm_name = "unknown"

    @abstractmethod
    def declare_parameters(self, node) -> None:
        """Khai các ROS parameter riêng của thuật toán."""

    @abstractmethod
    def build(self, node, dynamics, tau_limit, control_hz) -> TorqueController:
        """Dựng controller sau khi mô hình và giới hạn mô-men sẵn sàng."""


def vector_parameter(
        node, name: str, size: int, *, positive=False,
        nonnegative=False) -> np.ndarray:
    """Đọc vector parameter; cho phép dùng một số cho mọi khớp."""
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
