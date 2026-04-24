import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import scrolledtext, ttk
import threading

class Ros2GuiNode(Node):
    def __init__(self):
        super().__init__("ros2_gui_node")
        self.get_logger().info("✅ ROS2 GUI 节点已启动")

        # GUI 主窗口（必须在主线程）
        self.root = tk.Tk()
        self.root.title("ROS2 控制界面")
        self.root.geometry("500x400")

        # 标题标签
        ttk.Label(self.root, text="ROS2 控制面板", font=("Arial", 16)).pack(pady=10)

        # 状态显示
        self.status_label = ttk.Label(self.root, text="📶 状态：等待操作", font=("Arial", 12))
        self.status_label.pack(pady=5)

        # 按钮框架
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(pady=10)

        # 按钮
        self.start_btn = ttk.Button(btn_frame, text="启动摄像头", command=self.start_camera)
        self.start_btn.grid(row=0, column=0, padx=10)

        self.stop_btn = ttk.Button(btn_frame, text="停止摄像头", command=self.stop_camera, state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=1, padx=10)

        # 日志框
        ttk.Label(self.root, text="日志输出：").pack()
        self.log_box = scrolledtext.ScrolledText(self.root, width=60, height=15)
        self.log_box.pack(pady=5, padx=10)

        # 初始日志
        self.log("✅ GUI 界面加载完成")
        self.log("✅ ROS2 节点初始化成功")

        # 摄像头运行标记
        self.camera_running = False

    # 日志输出到界面
    def log(self, msg):
        self.log_box.insert(tk.END, msg + "\n")
        self.log_box.see(tk.END)  # 自动滚动到底部

    # 启动摄像头（按钮事件）
    def start_camera(self):
        if not self.camera_running:
            self.camera_running = True
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            self.status_label.config(text="📶 状态：摄像头运行中")
            self.log("▶️  正在启动 USB 摄像头...")
            
            # 这里后面可以对接真正的摄像头节点
            self.log("✅ 摄像头已启动，发布图像话题 /image_raw")

    # 停止摄像头
    def stop_camera(self):
        if self.camera_running:
            self.camera_running = False
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)
            self.status_label.config(text="📶 状态：已停止")
            self.log("⏹️  摄像头已停止")

    # 运行 GUI
    def run_gui(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = Ros2GuiNode()

    # ROS2 自旋放在子线程，避免卡住 GUI
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # 运行 GUI
    node.run_gui()

    # 关闭
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()