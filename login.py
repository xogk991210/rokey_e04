import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart


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
