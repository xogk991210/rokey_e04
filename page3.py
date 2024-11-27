from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
import sys


class Page3(QWidget):

    def __init__(self):
        super().__init__()

    def setupUi(self):
        if not self.objectName():
            self.setObjectName("self")

        # 창 크기 고정
        self.resize(1340, 618)  # 적당한 크기로 설정, 필요에 따라 조정

        # UI 요소들의 위치와 크기 설정
        sizePolicy = QSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sizePolicy().hasHeightForWidth())
        self.setSizePolicy(sizePolicy)

        # 첫 번째 이미지 영역
        self.verticalLayoutWidget = QWidget(self)
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayoutWidget.setGeometry(QRect(9, 9, 651, 511))  # 위치와 크기 설정
        self.verticalLayout = QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)

        self.CAM_1 = QLabel(self.verticalLayoutWidget)
        self.CAM_1.setObjectName("CAM_1")
        self.verticalLayout.addWidget(self.CAM_1)

        # 두 번째 이미지 영역
        self.verticalLayoutWidget_2 = QWidget(self)
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayoutWidget_2.setGeometry(
            QRect(679, 9, 651, 511)
        )  # 위치와 크기 설정
        self.verticalLayout_2 = QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)

        self.CAM_2 = QLabel(self.verticalLayoutWidget_2)
        self.CAM_2.setObjectName("CAM_2")
        self.verticalLayout_2.addWidget(self.CAM_2)

        # 텍스트 영역
        self.verticalLayoutWidget_4 = QWidget(self)
        self.verticalLayoutWidget_4.setObjectName("verticalLayoutWidget_4")
        self.verticalLayoutWidget_4.setGeometry(QRect(930, 530, 151, 21))  # 위치 설정
        self.verticalLayout_4 = QVBoxLayout(self.verticalLayoutWidget_4)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)

        self.label_2 = QLabel(self.verticalLayoutWidget_4)
        self.label_2.setObjectName("label_2")
        self.label_2.setAlignment(Qt.AlignCenter)
        self.label_2.setText("Worker View")  # 텍스트 설정

        self.verticalLayout_4.addWidget(self.label_2)

        # 또 다른 텍스트 영역
        self.verticalLayoutWidget_5 = QWidget(self)
        self.verticalLayoutWidget_5.setObjectName("verticalLayoutWidget_5")
        self.verticalLayoutWidget_5.setGeometry(QRect(270, 530, 151, 21))  # 위치 설정
        self.verticalLayout_5 = QVBoxLayout(self.verticalLayoutWidget_5)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)

        self.label = QLabel(self.verticalLayoutWidget_5)
        self.label.setObjectName("label")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setText("Global View")  # 텍스트 설정

        self.verticalLayout_5.addWidget(self.label)

        # 버튼 영역
        self.verticalLayoutWidget_3 = QWidget(self)
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.verticalLayoutWidget_3.setGeometry(QRect(590, 570, 160, 31))  # 위치 설정
        self.verticalLayout_3 = QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)

        self.pushButton = QPushButton(self.verticalLayoutWidget_3)
        self.pushButton.setObjectName("pushButton")
        self.pushButton.setText("Close")  # 버튼 텍스트 설정
        sizePolicy1 = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.pushButton.sizePolicy().hasHeightForWidth())
        self.pushButton.setSizePolicy(sizePolicy1)

        self.verticalLayout_3.addWidget(self.pushButton)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = Page3()
    ui.setupUi()
    ui.show()
    sys.exit(app.exec_())
