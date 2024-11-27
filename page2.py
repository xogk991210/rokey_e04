from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
import sys


class Page2(QWidget):

    def __init__(self):
        super().__init__()

    def setupUi(self):
        self.resize(1200, 800)

        # UI 요소들의 위치와 크기 설정
        sizePolicy = QSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sizePolicy().hasHeightForWidth())
        self.setSizePolicy(sizePolicy)

        self.verticalLayoutWidget = QWidget(self)
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayoutWidget.setGeometry(QRect(40, 70, 1111, 171))
        self.verticalLayout = QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)  # 셀 사이 여백 제거
        self.task_list = QLabel(self.verticalLayoutWidget)
        self.task_list.setObjectName("task_list")
        self.task_list.setAlignment(Qt.AlignCenter)

        self.verticalLayout.addWidget(self.task_list)

        self.task_1 = QPushButton(self.verticalLayoutWidget)
        self.task_1.setObjectName("task_1")
        sizePolicy = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.MinimumExpanding)
        sizePolicy.setHeightForWidth(self.task_1.sizePolicy().hasHeightForWidth())
        self.task_1.setSizePolicy(sizePolicy)

        self.verticalLayout.addWidget(self.task_1)

        self.task_2 = QPushButton(self.verticalLayoutWidget)
        self.task_2.setObjectName("task_2")
        sizePolicy.setHeightForWidth(self.task_2.sizePolicy().hasHeightForWidth())
        self.task_2.setSizePolicy(sizePolicy)

        self.verticalLayout.addWidget(self.task_2)

        self.task_3 = QPushButton(self.verticalLayoutWidget)
        self.task_3.setObjectName("task_3")
        sizePolicy.setHeightForWidth(self.task_3.sizePolicy().hasHeightForWidth())
        self.task_3.setSizePolicy(sizePolicy)

        self.verticalLayout.addWidget(self.task_3)

        # 두번째 영역
        self.horizontalLayoutWidget = QWidget(self)
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayoutWidget.setGeometry(QRect(40, 360, 1111, 80))
        self.horizontalLayout = QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)  # 레이아웃 외부 여백 제거
        self.horizontalLayout.setSpacing(0)  # 셀 사이 여백 제거
        self.time_title = QLabel(self.horizontalLayoutWidget)
        self.time_title.setObjectName("time_title")
        self.time_title.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.time_title)

        self.time = QLabel(self.horizontalLayoutWidget)
        self.time.setObjectName("time")
        self.time.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.time)

        # 3번째 영역
        self.gridLayoutWidget = QWidget(self)
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayoutWidget.setGeometry(QRect(40, 260, 1111, 80))
        self.gridLayout = QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setObjectName("gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)  # 레이아웃 외부 여백 제거
        self.gridLayout.setSpacing(0)  # 셀 사이 여백 제거
        self.state_title = QLabel(self.gridLayoutWidget)
        self.state_title.setObjectName("state_title")
        self.state_title.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.state_title, 0, 0, 1, 1)

        self.state = QLabel(self.gridLayoutWidget)
        self.state.setObjectName("state")
        self.state.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.state, 1, 0, 1, 1)

        # 4번째 영역
        self.horizontalLayoutWidget_2 = QWidget(self)
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayoutWidget_2.setGeometry(QRect(40, 460, 1111, 151))
        self.horizontalLayout_3 = QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.Play_Button = QPushButton(self.horizontalLayoutWidget_2)
        self.Play_Button.setObjectName("Play_Button")
        sizePolicy1 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Expanding)
        sizePolicy1.setHeightForWidth(self.Play_Button.sizePolicy().hasHeightForWidth())
        self.Play_Button.setSizePolicy(sizePolicy1)

        self.horizontalLayout_3.addWidget(self.Play_Button)

        self.Stop_Button = QPushButton(self.horizontalLayoutWidget_2)
        self.Stop_Button.setObjectName("Stop_Button")
        sizePolicy1.setHeightForWidth(self.Stop_Button.sizePolicy().hasHeightForWidth())
        self.Stop_Button.setSizePolicy(sizePolicy1)

        self.horizontalLayout_3.addWidget(self.Stop_Button)

        self.Reset_Button = QPushButton(self.horizontalLayoutWidget_2)
        self.Reset_Button.setObjectName("Reset_Button")
        sizePolicy1.setHeightForWidth(
            self.Reset_Button.sizePolicy().hasHeightForWidth()
        )
        self.Reset_Button.setSizePolicy(sizePolicy1)

        self.horizontalLayout_3.addWidget(self.Reset_Button)

        # 5번째 영역
        self.horizontalLayoutWidget_3 = QWidget(self)
        self.horizontalLayoutWidget_3.setObjectName("horizontalLayoutWidget_3")
        self.horizontalLayoutWidget_3.setGeometry(QRect(40, 640, 1111, 121))
        self.horizontalLayout_4 = QHBoxLayout(self.horizontalLayoutWidget_3)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.Conveyor_Button = QPushButton(self.horizontalLayoutWidget_3)
        self.Conveyor_Button.setObjectName("Conveyor_Button")
        sizePolicy1.setHeightForWidth(
            self.Conveyor_Button.sizePolicy().hasHeightForWidth()
        )
        self.Conveyor_Button.setSizePolicy(sizePolicy1)

        self.horizontalLayout_4.addWidget(self.Conveyor_Button)

        self.Camera_Button = QPushButton(self.horizontalLayoutWidget_3)
        self.Camera_Button.setObjectName("Camera_Button")
        sizePolicy1.setHeightForWidth(
            self.Camera_Button.sizePolicy().hasHeightForWidth()
        )
        self.Camera_Button.setSizePolicy(sizePolicy1)

        self.horizontalLayout_4.addWidget(self.Camera_Button)

        self.Conveyor_Button.clicked.connect(self.handle_play_click)
        self.Camera_Button.clicked.connect(self.handle_stop_click)
        self.Play_Button.clicked.connect(self.handle_play_click)
        self.Stop_Button.clicked.connect(self.handle_stop_click)
        self.Reset_Button.clicked.connect(self.handle_reset_click)
        self.task_1.clicked.connect(self.handle_task_1_click)
        self.task_2.clicked.connect(self.handle_task_2_click)
        self.task_3.clicked.connect(self.handle_task_3_click)

        self.setWindowTitle(
            QCoreApplication.translate("MainWindow", "제어 모니터", None)
        )
        self.task_list.setText(
            QCoreApplication.translate("MainWindow", "작업목록", None)
        )
        self.task_list.setStyleSheet(
            "border: 1px solid black; font-size: 20px; background-color: white;"
        )

        self.task_1.setText(
            QCoreApplication.translate(
                "MainWindow", "Job1 | red*2, blue*1, goto goal 1", None
            )
        )
        self.task_2.setText(
            QCoreApplication.translate(
                "MainWindow", "Job2 | red*1, blue*2, goto goal 2", None
            )
        )
        self.task_3.setText(
            QCoreApplication.translate(
                "MainWindow", "Job3 | red*1,               goto goal 3", None
            )
        )
        self.time_title.setText(
            QCoreApplication.translate("MainWindow", "작업시간", None)
        )
        self.time_title.setStyleSheet(
            "border: 1px solid black; font-size: 20px; background-color: white;"
        )
        self.time.setText(
            QCoreApplication.translate("MainWindow", "작업시간상태", None)
        )
        self.time.setStyleSheet(
            "border: 1px solid black; font-size: 20px; background-color: white;"
        )
        self.state_title.setText(
            QCoreApplication.translate("MainWindow", "작업상태", None)
        )
        self.state_title.setStyleSheet(
            "border: 1px solid black; font-size: 20px; background-color: white;"
        )
        self.state.setText(
            QCoreApplication.translate("MainWindow", '"로봇이 작업중입니다"', None)
        )
        self.state.setStyleSheet(
            "border: 1px solid black; font-size: 20px; background-color: white;"
        )
        self.Play_Button.setText(QCoreApplication.translate("MainWindow", "PLAY", None))
        self.Stop_Button.setText(QCoreApplication.translate("MainWindow", "STOP", None))
        self.Reset_Button.setText(
            QCoreApplication.translate("MainWindow", "RESET", None)
        )
        self.Conveyor_Button.setText(
            QCoreApplication.translate("MainWindow", "컨베이어 조작", None)
        )
        self.Camera_Button.setText(
            QCoreApplication.translate("MainWindow", "실시간 영상 확인", None)
        )

    def handle_task_1_click(self):
        QMessageBox.information(None, "Task 1", "Task 1 버튼이 눌렸습니다!")

    def handle_task_2_click(self):
        QMessageBox.information(None, "Task 2", "Task 2 버튼이 눌렸습니다!")

    def handle_task_3_click(self):
        QMessageBox.information(None, "Task 3", "Task 3 버튼이 눌렸습니다!")

    def handle_play_click(self):
        QMessageBox.information(None, "Play Action", "PLAY 버튼이 눌렸습니다!")

    def handle_stop_click(self):
        QMessageBox.information(None, "Stop Action", "STOP 버튼이 눌렸습니다!")

    def handle_reset_click(self):
        QMessageBox.information(None, "Reset Action", "RESET 버튼이 눌렸습니다!")
