from PySide2.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QCheckBox,
    QWidget,
)
from PySide2.QtCore import Qt
from PySide2.QtGui import QFont
import sys
from login import *


class LoginWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 모니터 해상도 가져오기
        screen = QApplication.primaryScreen()
        screen_geometry = screen.geometry()
        screen_width = screen_geometry.width()
        screen_height = screen_geometry.height()

        self.setWindowTitle("Login")
        self.setGeometry(
            int(screen_width * 0.1),
            int(screen_height * 0.1),
            int(screen_width * 0.8),
            int(screen_height * 0.8),
        )
        # 중앙 위젯 및 레이아웃 설정
        central_widget = QWidget()
        layout = QVBoxLayout()

        # 아이디 라벨과 입력 칸
        self.label_id = QLabel("ID : ")
        self.label_id.setFont(QFont("Arial", 20))
        self.input_id = QLineEdit()
        self.input_id.setPlaceholderText("아이디를 입력하세요")
        layout.addWidget(self.label_id)
        layout.addWidget(self.input_id)

        # 비밀번호 라벨과 입력 칸
        self.label_password = QLabel("Password:")
        self.label_password.setFont(QFont("Arial", 20))
        self.input_password = QLineEdit()
        self.input_password.setEchoMode(QLineEdit.Password)  # 비밀번호 마스킹 설정
        self.input_password.setPlaceholderText("비밀번호를 입력하세요")
        layout.addWidget(self.label_password)
        layout.addWidget(self.input_password)

        # 비밀번호 보기 체크박스
        self.checkbox_show_password = QCheckBox("비밀번호 보기")
        self.checkbox_show_password.setFont(QFont("Arial", 12))
        self.checkbox_show_password.stateChanged.connect(
            self.toggle_password_visibility
        )
        layout.addWidget(self.checkbox_show_password)

        # 로그인 버튼
        self.login_button = QPushButton("Login")
        self.login_button.setFont(QFont("Arial", 20))
        self.login_button.clicked.connect(self.handle_login)
        layout.addWidget(self.login_button)

        # 레이아웃을 중앙 위젯에 설정
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def toggle_password_visibility(self, state):
        """
        체크박스 상태에 따라 비밀번호 보이기/마스킹 설정
        """
        if state == Qt.Checked:
            self.input_password.setEchoMode(QLineEdit.Normal)  # 비밀번호 보이기
        else:
            self.input_password.setEchoMode(QLineEdit.Password)  # 비밀번호 마스킹

    def handle_login(self):
        """
        로그인 버튼 클릭 시 아이디와 비밀번호를 터미널에 출력
        """
        user_id = self.input_id.text()
        user_password = self.input_password.text()
        gmail = Login(user_id, user_password)
        msg = gmail.create_msg("a", "b")
        gmail.pub_email(msg, "ekqls6655@naver.com")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LoginWindow()
    window.show()
    sys.exit(app.exec_())
