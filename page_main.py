from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
from page1 import *
from page2 import *
from page3 import *
from page4 import *
import sys


# MainWindow 클래스 정의
class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Four-Page Example")
        self.resize(800, 600)

        # 중앙 위젯 생성 및 설정
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 메인 레이아웃
        main_layout = QVBoxLayout(central_widget)

        # QStackedWidget 생성
        self.stacked_widget = QStackedWidget()
        main_layout.addWidget(self.stacked_widget)

        # 페이지 추가
        self.page1 = Page1()
        self.page2 = Page2()
        self.page3 = Page3()
        self.page4 = Page4()
        self.page1.setupUi()
        self.page2.setupUi()
        self.page3.setupUi()
        self.page4.setupUi()
        self.stacked_widget.addWidget(self.page1)
        self.stacked_widget.addWidget(self.page2)
        self.stacked_widget.addWidget(self.page3)
        self.stacked_widget.addWidget(self.page4)

        # 버튼 연결
        self.page1.login_button.clicked.connect(lambda: self.switch_page(1))
        self.page2.Camera_Button.clicked.connect(lambda: self.switch_page(2))
        self.page2.Conveyor_Button.clicked.connect(lambda: self.switch_page(3))
        self.page3.pushButton.clicked.connect(lambda: self.switch_page(1))
        self.page4.back_button.clicked.connect(lambda: self.switch_page(1))

    def switch_page(self, index):
        """페이지를 전환"""
        self.stacked_widget.setCurrentIndex(index)


# 실행 코드
if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
