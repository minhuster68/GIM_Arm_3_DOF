#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import can
import struct

class MotorTestNodePython(Node):
    def __init__(self):
        super().__init__('motor_test_all_modes_python')
        
        # Khai báo ID động cơ (mặc định là 0)
        self.declare_parameter('motor_id', 1)
        self.motor_id = self.get_parameter('motor_id').value
        
        # 1. Khởi tạo kết nối SocketCAN trực tiếp vào cổng can0
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.get_logger().info("Đã kết nối SocketCAN (can0) thành công!")
        except Exception as e:
            self.get_logger().error(f"Lỗi kết nối CAN: {e}")
            return

        self.current_mode = "none"

        # 2. Khởi tạo Subscriber
        self.mode_sub = self.create_subscription(
            String, '/motor/set_mode', self.mode_callback, 10)
        self.target_sub = self.create_subscription(
            Float32, '/motor/set_target', self.target_callback, 10)
            
        self.get_logger().info(f"Node Python test động cơ {self.motor_id} đã sẵn sàng!")

    def send_can_msg(self, cmd_id, data):
        """Hàm trợ giúp: Đóng gói và gửi frame CAN theo chuẩn ODrive"""
        # Node ID chiếm 6 bit cao, Command ID chiếm 5 bit thấp[cite: 1]
        can_id = (self.motor_id << 5) | cmd_id
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def mode_callback(self, msg):
        mode = msg.data.lower()
        
        # Bắt buộc đưa động cơ vào trạng thái CLOSED_LOOP_CONTROL (State = 8)[cite: 1]
        # Dùng <I để đóng gói kiểu uint32_t (Little Endian)[cite: 1]
        self.send_can_msg(0x007, struct.pack('<I', 8))
        
        if mode == "position":
            # CMD 0x00B: Set_Controller_Mode. Control Mode = 3 (Pos), Input Mode = 1 (Passthrough)[cite: 1]
            self.send_can_msg(0x00B, struct.pack('<II', 3, 1))
            self.current_mode = "position"
            self.get_logger().info(f"[Node {self.motor_id}] Chế độ POSITION CONTROL")
            
        elif mode == "velocity":
            # CMD 0x00B: Set_Controller_Mode. Control Mode = 2 (Vel), Input Mode = 1 (Passthrough)[cite: 1]
            self.send_can_msg(0x00B, struct.pack('<II', 2, 1))
            self.current_mode = "velocity"
            self.get_logger().info(f"[Node {self.motor_id}] Chế độ VELOCITY CONTROL")
            
        elif mode == "torque":
            # CMD 0x00B: Set_Controller_Mode. Control Mode = 1 (Torque), Input Mode = 1 (Passthrough)[cite: 1]
            self.send_can_msg(0x00B, struct.pack('<II', 1, 1))
            self.current_mode = "torque"
            self.get_logger().info(f"[Node {self.motor_id}] Chế độ TORQUE CONTROL")

    def target_callback(self, msg):
        target = msg.data
        
        if self.current_mode == "position":
            # CMD 0x00C (Set_Input_Pos): float32 Pos, int16 Vel_FF, int16 Torque_FF[cite: 1]
            # Format '<fhh' tương ứng với: 1 float (4 bytes) + 2 short (2x2 bytes) = 8 bytes[cite: 1]
            self.send_can_msg(0x00C, struct.pack('<fhh', target, 0, 0))
            self.get_logger().info(f"=> Lệnh Vị trí: {target} turns")
            
        elif self.current_mode == "velocity":
            # CMD 0x00D (Set_Input_Vel): float32 Vel, float32 Torque_FF[cite: 1]
            # Format '<ff' tương ứng với: 2 float = 8 bytes[cite: 1]
            self.send_can_msg(0x00D, struct.pack('<ff', target, 0.0))
            self.get_logger().info(f"=> Lệnh Vận tốc: {target} turns/s")
            
        elif self.current_mode == "torque":
            # CMD 0x00E (Set_Input_Torque): float32 Torque[cite: 1]
            # Format '<f' tương ứng với: 1 float = 4 bytes[cite: 1]
            self.send_can_msg(0x00E, struct.pack('<f', target))
            self.get_logger().info(f"=> Lệnh Mô-men: {target} Nm")

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNodePython()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()