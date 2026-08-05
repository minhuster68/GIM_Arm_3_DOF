import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can
import struct
import math
import time

class MoveItToCANBridge(Node):
    def __init__(self):
        super().__init__('moveit_to_can_bridge')
        
        # 1. Khởi tạo kết nối SocketCAN
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.get_logger().info("Đã kết nối SocketCAN (can0) thành công!")
        except Exception as e:
            self.get_logger().error(f"Lỗi kết nối CAN: {e}")
            return

        # 2. Cấu hình ID và Tỷ số truyền
        self.joint_config = {
            'base_joint':     {'id': 1, 'gear_ratio': 8.0},
            'shoulder_joint': {'id': 2, 'gear_ratio': 8.0},
            'elbow_joint':    {'id': 3, 'gear_ratio': 8.0}
        }
        
        # 3. Thực hiện Set Zero và Kích hoạt động cơ
        self.hardware_set_zero_and_start()

        # 4. Tạo Subscriber lắng nghe MoveIt
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

    def send_can_raw(self, motor_id, cmd_id, data):
        can_id = (motor_id << 5) | cmd_id
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def hardware_set_zero_and_start(self):
        """Reset Encoder bằng phần cứng, sau đó khóa trục"""
        self.get_logger().info("Đang Reset Encoder về 0 và kích hoạt động cơ...")
        
        for joint, config in self.joint_config.items():
            m_id = config['id']
            
            # Bước 1: Gửi lệnh Set_Linear_Count (0x011) với giá trị 0 (kiểu int32)
            self.send_can_raw(m_id, 0x011, struct.pack('<i', 0))
            
            # Cho phần cứng 10ms để xử lý lệnh reset trước khi nhận lệnh khác
            time.sleep(0.5)
            
            # Bước 2: Đưa mục tiêu vị trí về chính xác 0.0 (chống giật)
            self.send_can_raw(m_id, 0x00C, struct.pack('<fhh', 0.0, 0, 0))
            
            # Bước 3: Chuyển mode POSITION CONTROL
            self.send_can_raw(m_id, 0x00B, struct.pack('<II', 3, 1))
            
            # Bước 4: Bật CLOSED_LOOP_CONTROL
            self.send_can_raw(m_id, 0x007, struct.pack('<I', 8))
            
        self.get_logger().info("HỆ THỐNG ĐÃ KHÓA TRỤC TẠI ZERO. SẴN SÀNG NHẬN LỆNH!")

    def joint_states_callback(self, msg):
        for index, joint_name in enumerate(msg.name):
            if joint_name in self.joint_config:
                radian_angle = msg.position[index]
                config = self.joint_config[joint_name]
                motor_id = config['id']
                
                # Tính toán mục tiêu cực kỳ sạch: Chỉ quy đổi từ Radian sang Vòng quay
                target_pos = (radian_angle / (2.0 * math.pi)) * config['gear_ratio']
                
                # Gửi thẳng xuống động cơ, không cần bù trừ gì thêm
                self.send_can_raw(motor_id, 0x00C, struct.pack('<fhh', float(target_pos), 0, 0))

def main(args=None):
    rclpy.init(args=args)
    node = MoveItToCANBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()