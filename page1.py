import smtplib
import sys
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Login:
    def __init__(self, ID, Password):
        self.id = ID
        self.password = Password
        self.server = self.google_login()

    def google_login(self):
        try:
            server = smtplib.SMTP("smtp.gmail.com", 587)
            server.starttls()
            server.login(self.id, self.password)
            return server
        except Exception as e:
            print(e)

    def create_msg(self, subject, txt):
        try:
            msg = MIMEMultipart()
            msg["Subject"] = subject
            msg.attach(MIMEText(txt, "plain"))
            return msg
        except Exception as e:
            print(e)

    def pub_email(self, msg, sub):
        try:
            msg["From"] = self.id
            msg["To"] = sub
            self.server.sendmail(self.id, sub, msg.as_string())
            print("send")
        except Exception as e:
            print(e)


class Page1(QWidget):
    def __init__(self):
        super().__init__()
        self.user_id = None
        self.user_password = None

    def setupUi(self):
        layout = QVBoxLayout()

        self.label_id = QLabel("ID : ")
        self.input_id = QLineEdit()
        self.input_id.setPlaceholderText("아이디를 입력하세요")
        layout.addWidget(self.label_id)
        layout.addWidget(self.input_id)

        # 비밀번호 라벨과 입력 칸
        self.label_password = QLabel("Password:")
        self.input_password = QLineEdit()
        self.input_password.setEchoMode(QLineEdit.Password)  # 비밀번호 마스킹 설정
        self.input_password.setPlaceholderText("비밀번호를 입력하세요")
        layout.addWidget(self.label_password)
        layout.addWidget(self.input_password)

        # 비밀번호 보기 체크박스
        self.checkbox_show_password = QCheckBox("비밀번호 보기")
        self.checkbox_show_password.stateChanged.connect(
            self.toggle_password_visibility
        )
        layout.addWidget(self.checkbox_show_password)

        self.login_button = QPushButton("Login")
        self.login_button.clicked.connect(self.handle_login)
        layout.addWidget(self.login_button)

        self.setLayout(layout)

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
        # gmail = Login(user_id, user_password)
        # msg = gmail.create_msg("a", "b")
        # gmail.pub_email(msg, "ekqls6655@naver.com")
