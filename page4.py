from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
import sys
import serial
import threading


class Page4(QWidget):
    def setupUi(self):

        # UI 요소들의 위치와 크기 설정
        sizePolicy = QSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sizePolicy().hasHeightForWidth())
        self.setSizePolicy(sizePolicy)

        # 메인 위젯 및 레이아웃 설정
        self.central_widget = QWidget(self)  # self의 중앙 위젯
        self.main_layout = QVBoxLayout(self.central_widget)  # 중앙 위젯의 레이아웃
        self.setLayout(self.main_layout)  # self에 레이아웃 설정

        # 라벨 추가
        self.label = QLabel("컨베이어 작동 여부")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 20px;")
        self.main_layout.addWidget(self.label)

        # 버튼 레이아웃
        self.button_layout = QHBoxLayout()

        # 시작 버튼
        self.pushButton = QPushButton("시작")
        self.pushButton.setStyleSheet("font-size: 20px; background-color: green;")
        self.pushButton.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.button_layout.addWidget(self.pushButton)

        # 정지 버튼
        self.pushButton_2 = QPushButton("정지")
        self.pushButton_2.setStyleSheet("font-size: 20px; background-color: red;")
        self.pushButton_2.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.button_layout.addWidget(self.pushButton_2)

        self.main_layout.addLayout(self.button_layout)

        # 하단에 "작업화면으로 돌아가기" 버튼
        self.back_button = QPushButton("작업화면으로 돌아가기")
        self.back_button.setStyleSheet(
            "font-size: 16px; background-color: blue; color: white;"
        )
        self.back_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.main_layout.addWidget(self.back_button)
        self.conveyor = Conveyor()
        self.back_button.clicked.connect(self.back_button_clicked)
        self.pushButton.clicked.connect(self.button1_clicked)
        self.pushButton_2.clicked.connect(self.button2_clicked)

    # "작업화면으로 돌아가기" 버튼 클릭 이벤트 핸들러
    def back_button_clicked(self):
        self.label.setText("컨베이어 작동 여부")  # 라벨 초기화

    def button1_clicked(self):
        if self.label is not None:
            self.label.setText("컨베이어 작동중")
            self.conveyor.start_conveyor(True)

    def button2_clicked(self):
        if self.label is not None:
            self.label.setText("컨베이어 작동 중지")
            self.conveyor.start_conveyor(False)


class Conveyor:
    def __init__(self):
        try:
            self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
            self.running = False  # 스레드 실행 여부
            self.thread = None  # 스레드 객체
        except Exception as e:
            print(f"시리얼 포트를 열지 못했습니다: {e}")

    def start_conveyor(self, run_conveyor):
        if run_conveyor and not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.run, args=(True,))
            self.thread.start()
        elif not run_conveyor and self.running:
            self.running = False
            self.thread = threading.Thread(target=self.run, args=(False,))
            self.thread.start()

    def run(self, run_command):
        try:
            command = "1\n" if run_command else "0\n"
            self.ser.write(command.encode("utf-8"))
            response = self.ser.read().decode("utf-8")
            print(f"아두이노 응답: {response}")
            self.ser.flush()
        except Exception as e:
            print(f"오류가 발생했습니다: {e}")
