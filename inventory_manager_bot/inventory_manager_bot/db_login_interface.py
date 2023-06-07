from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
import os

class DBLoginUI(QMainWindow):
    def __init__(self,app):
        #initialize UI
        super(DBLoginUI,self).__init__()
        #Load UI file
        uic.loadUi(os.getcwd() + "/src/inventory_manager_bot/inventory_manager_bot/UI Layouts/db_login.ui",self)
        #Show applicaiton
        self.show()
        self.main_app = app
        self.login_button.clicked.connect(lambda:self.on_login_clicked())
    
    def set_ui_components(self):
        pass

    def on_login_clicked(self):
        #Grab all text from each user input box
        user = self.user_edit.text()
        password = self.password_edit.text()
        host = self.host_edit.text()
        port = self.port_edit.text()
        db = self.db_edit.text()
        #Use information to connect to database
        self.main_app.login_to_database(user,password,host,port,db)

def main():
    app = QApplication([])
    window = DBLoginUI("hi")
    window.setWindowTitle('Database Login')
    app.exec()
    

if __name__ == "__main__":
    main()