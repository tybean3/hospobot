#!/usr/bin/env python3
import sys
import os
import time
import subprocess

# Force X11/XWayland backend to bypass strict Wayland fullscreen security policies
# Wayland natively blocks applications from auto-fullscreening without a mouse/keyboard click.
os.environ["QT_QPA_PLATFORM"] = "xcb"
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QListWidget, QFrame, QHBoxLayout, QScrollArea, QScroller
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont

class SystemStatusWorker(QThread):
    update_signal = pyqtSignal(dict)

    def run(self):
        while True:
            # 1. Get ROS Topics via CLI
            # We use the native Linux 'timeout' tool to prevent Python's subprocess pipe-hanging bug
            topics_str = subprocess.getoutput('timeout 1.5 bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"')
            topics = [t for t in topics_str.split('\n') if t.strip() and not t.startswith('WARNING')]

            # 2. Check RealSense
            realsense_ok = any(t.startswith('/camera/') for t in topics)

            # 3. Get WiFi connection
            wifi_ssid = subprocess.getoutput("timeout 1 iwgetid -r").strip()
            if not wifi_ssid:
                wifi_ssid = subprocess.getoutput("timeout 1 nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d: -f2").strip()
            
            if not wifi_ssid or "not found" in wifi_ssid.lower() or "no such" in wifi_ssid.lower():
                wifi_status = "Disconnected"
            else:
                wifi_status = f"{wifi_ssid}"

            # 4. Get SSH connections (looking for established connections on port 22)
            ssh_out = subprocess.getoutput("timeout 1 ss -tn state established | grep :22")
            ssh_count = len([line for line in ssh_out.split('\n') if line.strip()]) - 1
            ssh_status = f"{ssh_count} active" if ssh_count > 0 else "None"
            
            data = {
                'topics': topics,
                'realsense': realsense_ok,
                'wifi': wifi_status,
                'ssh': ssh_status
            }
            self.update_signal.emit(data)
            
            # Refresh every 1 second
            time.sleep(1)

class KioskApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        
        # Start background worker for system stats
        self.worker = SystemStatusWorker()
        self.worker.update_signal.connect(self.update_ui)
        self.worker.start()

    def initUI(self):
        # Premium dark mode aesthetics
        self.setStyleSheet("""
            QMainWindow, QWidget#CentralWidget {
                background-color: #0d1117;
            }
            QLabel {
                color: #e6edf3;
            }
            QFrame {
                background-color: #161b22;
                border-radius: 20px;
                border: 1px solid #30363d;
            }
            QListWidget {
                background-color: #161b22;
                color: #8b949e;
                border: 1px solid #30363d;
                font-size: 16px;
                border-radius: 15px;
                padding: 10px;
            }
            QListWidget::item {
                padding: 6px;
                border-bottom: 1px solid #21262d;
            }
            QListWidget::item:selected {
                background-color: #1f6feb;
                color: #ffffff;
                border-radius: 5px;
            }
        """)

        central_widget = QWidget()
        central_widget.setObjectName("CentralWidget")
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

        # Title
        title = QLabel("Hospobot Operating System")
        title.setFont(QFont("Inter", 32, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #58a6ff;")
        main_layout.addWidget(title)

        # Main Content Layout (Left: Cards, Right: Topics)
        content_layout = QHBoxLayout()
        content_layout.setSpacing(20)

        # --- Left Column: System Status Cards ---
        sys_info_layout = QVBoxLayout()
        sys_info_layout.setSpacing(10)

        def create_card(title_text, value_text, value_color):
            card = QFrame()
            layout = QVBoxLayout(card)
            layout.setContentsMargins(15, 15, 15, 15)
            
            title_lbl = QLabel(title_text)
            title_lbl.setFont(QFont("Inter", 14, QFont.DemiBold))
            title_lbl.setStyleSheet("color: #8b949e; border: none; background: transparent;")
            title_lbl.setAlignment(Qt.AlignCenter)
            
            val_lbl = QLabel(value_text)
            val_lbl.setFont(QFont("Inter", 22, QFont.Bold))
            val_lbl.setStyleSheet(f"color: {value_color}; border: none; background: transparent;")
            val_lbl.setAlignment(Qt.AlignCenter)
            
            layout.addWidget(title_lbl)
            layout.addWidget(val_lbl)
            return card, val_lbl

        # Create Cards
        status_card, self.status_value = create_card("System Status", "STANDBY", "#3fb950")
        charge_card, self.charge_value = create_card("Battery Level", "98%", "#d2a8ff")
        wifi_card, self.wifi_value = create_card("WiFi Network", "Checking...", "#58a6ff")
        ssh_card, self.ssh_value = create_card("SSH Connections", "Checking...", "#ff7b72")
        cam_card, self.cam_value = create_card("RealSense Camera", "Checking...", "#ffa657")

        sys_info_layout.addWidget(status_card)
        sys_info_layout.addWidget(charge_card)
        sys_info_layout.addWidget(wifi_card)
        sys_info_layout.addWidget(ssh_card)
        sys_info_layout.addWidget(cam_card)
        
        # Add stretch to keep cards tight at the top
        sys_info_layout.addStretch()

        left_container = QWidget()
        left_container.setLayout(sys_info_layout)
        left_container.setStyleSheet("QWidget { background: transparent; }")

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(left_container)
        scroll_area.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        scroll_area.setMinimumWidth(300)
        scroll_area.setMaximumWidth(400)

        content_layout.addWidget(scroll_area, 1)

        # --- Right Column: Active ROS Topics ---
        right_layout = QVBoxLayout()
        topics_label = QLabel("Active ROS Topics")
        topics_label.setFont(QFont("Inter", 22, QFont.Bold))
        topics_label.setStyleSheet("color: #79c0ff;")
        right_layout.addWidget(topics_label)

        self.topics_list = QListWidget()
        self.topics_list.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.topics_list.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        QScroller.grabGesture(self.topics_list.viewport(), QScroller.LeftMouseButtonGesture)
        right_layout.addWidget(self.topics_list)

        right_container = QWidget()
        right_container.setLayout(right_layout)

        content_layout.addWidget(right_container, 2)

        main_layout.addLayout(content_layout)

    def update_ui(self, data):
        # 1. Update topics list efficiently
        current_topics = [self.topics_list.item(i).text() for i in range(self.topics_list.count())]
        new_topics = sorted(data['topics'])
        
        if current_topics != new_topics:
            self.topics_list.clear()
            for topic in new_topics:
                self.topics_list.addItem(topic)
                
        # 2. Update WiFi
        self.wifi_value.setText(data['wifi'])
        if "Disconnected" in data['wifi']:
            self.wifi_value.setStyleSheet("color: #ff7b72; border: none; background: transparent;")
        else:
            self.wifi_value.setStyleSheet("color: #58a6ff; border: none; background: transparent;")
            
        # 3. Update SSH
        self.ssh_value.setText(data['ssh'])
        if data['ssh'] != "None":
            self.ssh_value.setStyleSheet("color: #ff7b72; border: none; background: transparent;")
        else:
            self.ssh_value.setStyleSheet("color: #8b949e; border: none; background: transparent;")

        # 4. Update RealSense
        if data['realsense']:
            self.cam_value.setText("ACTIVE")
            self.cam_value.setStyleSheet("color: #3fb950; border: none; background: transparent;")
        else:
            self.cam_value.setText("INACTIVE")
            self.cam_value.setStyleSheet("color: #ff7b72; border: none; background: transparent;")

def main():
    app = QApplication(sys.argv)
    
    # Enable High DPI scaling for crisp rendering on modern displays
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    ex = KioskApp()
    ex.setWindowFlags(Qt.Window | Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
    
    # Explicitly resize to screen to fix Wayland/X11 bugs where showFullScreen doesn't stretch
    screen_rect = app.primaryScreen().geometry()
    ex.setGeometry(screen_rect)
    ex.showFullScreen()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
