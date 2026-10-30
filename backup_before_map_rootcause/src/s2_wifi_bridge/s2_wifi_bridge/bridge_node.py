import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import threading
import queue
import math

class S2WifiBridgeNode(Node):
    def __init__(self):
        super().__init__('s2_wifi_bridge')
        
        # Khai báo và lấy các tham số
        self.declare_parameter('esp32_ip', '10.45.109.173')
        self.declare_parameter('esp32_port', 8888)
        
        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.esp32_port = self.get_parameter('esp32_port').value
        
        # Publisher cho dữ liệu LaserScan
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        
        # Hàng đợi trung gian an toàn luồng (thread-safe queue)
        self.scan_queue = queue.Queue()
        self.running = True
        
        # Khởi động luồng chạy client TCP kết nối tới ESP32
        self.tcp_thread = threading.Thread(target=self.tcp_client_loop, daemon=True)
        self.tcp_thread.start()
        
        # Timer kiểm tra hàng đợi và publish dữ liệu lên ROS 2 (tần số 100Hz)
        self.create_timer(0.01, self.check_queue_and_publish)
        
        self.get_logger().info(f"Khởi tạo Bridge. Đang chuẩn bị kết nối tới ESP32: {self.esp32_ip}:{self.esp32_port}")

    def tcp_client_loop(self):
        """Luồng TCP Client chạy bất đồng bộ để kết nối và nhận dữ liệu từ ESP32"""
        while self.running:
            self.get_logger().info(f"Đang kết nối tới ESP32 tại {self.esp32_ip}:{self.esp32_port}...")
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5.0)  # Timeout kết nối 5 giây
                s.connect((self.esp32_ip, self.esp32_port))
                self.get_logger().info("Đã kết nối thành công tới ESP32!")
                
                buffer = ""
                accumulating = False
                current_points = []
                
                while self.running:
                    data = s.recv(8192)
                    if not data:
                        self.get_logger().warn("Mất kết nối với ESP32 TCP Server.")
                        break
                    
                    buffer += data.decode('utf-8', errors='ignore')
                    
                    # Tách các dòng dữ liệu bằng ký tự xuống dòng
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line == "SCAN_START":
                            accumulating = True
                            current_points = []
                        elif line == "SCAN_END":
                            if accumulating and current_points:
                                # Đẩy dữ liệu vòng quét hoàn chỉnh vào hàng đợi
                                self.scan_queue.put(current_points)
                            accumulating = False
                        elif accumulating:
                            if ',' in line:
                                try:
                                    angle_str, dist_str = line.split(',')
                                    angle = float(angle_str)
                                    dist = int(dist_str)
                                    current_points.append((angle, dist))
                                except ValueError:
                                    pass
            except Exception as e:
                self.get_logger().error(f"Lỗi kết nối Socket: {e}. Sẽ thử kết nối lại sau 2 giây...")
                threading.Event().wait(2.0)
            finally:
                try:
                    s.close()
                except:
                    pass

    def check_queue_and_publish(self):
        """Timer callback định kỳ kiểm tra hàng đợi và publish LaserScan"""
        while not self.scan_queue.empty():
            try:
                points = self.scan_queue.get_nowait()
                self.publish_scan(points)
            except queue.Empty:
                break

    def publish_scan(self, points):
        """Xử lý danh sách các điểm đo và publish lên topic /scan"""
        # S2 sử dụng 720 bins (độ phân giải 0.5 độ trên 360 độ)
        num_bins = 720
        angle_increment = 2.0 * math.pi / num_bins
        
        # Khởi tạo mảng ranges với giá trị vô cực (inf)
        ranges = [float('inf')] * num_bins
        
        for angle_deg, dist_mm in points:
            # Chuyển đổi milimet sang mét
            dist_m = dist_mm / 1000.0
            
            # Kiểm tra dải đo hợp lệ của S2 (0.11m đến 8.0m)
            if dist_m < 0.11 or dist_m > 8.0:
                continue
            
            # Chuyển đổi góc từ độ sang radian và chuẩn hóa về dải [0, 2*pi)
            angle_rad = math.radians(angle_deg)
            angle_rad = angle_rad % (2.0 * math.pi)
            
            # Tìm bin tương ứng gần nhất
            bin_idx = int(round(angle_rad / angle_increment)) % num_bins
            
            # Nếu có nhiều điểm rơi vào cùng 1 bin, ưu tiên điểm gần nhất
            if dist_m < ranges[bin_idx]:
                ranges[bin_idx] = dist_m
                
        # Tạo thông điệp LaserScan
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi * (num_bins - 1) / num_bins
        msg.angle_increment = angle_increment
        
        # S2 quay mặc định khoảng 6Hz, do đó thời gian quét là ~0.166 giây
        msg.scan_time = 0.5
        msg.time_increment = msg.scan_time / num_bins
        
        msg.range_min = 0.11
        msg.range_max = 8.0
        msg.ranges = ranges
        
        self.publisher.publish(msg)

    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = S2WifiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
