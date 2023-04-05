from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
from inventory_manager_bot.db_login_interface import DBLoginUI
from inventory_manager_bot.inventory_db_connector import InventoryDBConnector
import rclpy
from rclpy.node import Node
from inventory_management_interfaces.srv import CallRobotToWaitingArea
from inventory_manager_bot.robot_caller_client import RobotCallerClientNode

class RobotCallerUI(QMainWindow):
    def __init__(self):
        #initialize UI
        super(RobotCallerUI,self).__init__()
        #Load UI file
        uic.loadUi("/home/petern25/ros2_ws/src/inventory_manager_bot/inventory_manager_bot/UI Layouts/robot_caller.ui",self)
        #Show applicaiton
        self.show()
        
        rclpy.init(args=None)
        self.node = Node("robot_caller_client")
        self.conn = None
        self.open_db_login_UI()

        self.set_ui_components()
        self.search_button.clicked.connect(lambda: self.search_for_keyword())
        self.add_button.clicked.connect(lambda: self.add_selected_item_to_list())
        self.remove_button.clicked.connect(lambda: self.remove_selected_item_from_list())
        self.call_button.clicked.connect(lambda: self.call_robot())

    def set_ui_components(self):
        self.fetched_table.setColumnCount(8)
        header = ["Product No.", "Product Name", "Brand","Weight","Quantity","Aisle","Check-In Date","Check-out Date"]
        self.fetched_table.setHorizontalHeaderLabels(header)
        

    def open_db_login_UI(self):
        self.login_window = DBLoginUI(self)
        self.login_window.setWindowTitle('Database Login')
        self.login_window.show()

    def login_to_database(self,user,password,host,port,db):
        self.conn = InventoryDBConnector()
        try:
            self.conn.connect_to_db(db,host,user,password,port)
            message = QMessageBox()
            message.setText("Database connection established!")
            message.setWindowTitle("Connection Message")
            message.exec_()
            self.login_window.hide()
        except Exception as e:
            self.conn = None
            message.setText("Could not connect to database!\n" + \
                            str(e))
            message.setWindowTitle("Connection Message")
            message.exec_()
        
    def search_for_keyword(self):
        keyword = self.keyword_edit.text()
        result=self.conn.get_all_records_from_word(keyword)
        if len(result) != 0:
            self.fetched_table.setRowCount(len(result))
            for i in range(len(result)):
                self.set_item_into_table(i,0,result[i][0])
                self.set_item_into_table(i,1,result[i][1])
                self.set_item_into_table(i,2,result[i][2])
                self.set_item_into_table(i,3,str(result[i][3]))
                self.set_item_into_table(i,4,str(result[i][4]))
                self.set_item_into_table(i,5,str(result[i][5]))
                self.set_item_into_table(i,6,str(result[i][6]))
                self.set_item_into_table(i,7,str(result[i][7]))
        else:
            msg = QMessageBox()
            msg.setText("No record was found with keyword")
            msg.setWindowTitle("Search Result")
            msg.exec_()
            
    def set_item_into_table(self,row,col,item):
        self.fetched_table.setItem(row,col,QTableWidgetItem(item))

    def add_selected_item_to_list(self):
        row = self.fetched_table.currentRow()
        product_no = self.fetched_table.item(row,0).text()
        product_name = self.fetched_table.item(row,1).text()
        item = product_name + "-" + product_no
        if self.list_widget.findItems(item,Qt.MatchFlag.MatchCaseSensitive):
            msg = QMessageBox()
            msg.setText("Item already added to the list")
            msg.setWindowTitle("Warning Message")
            msg.exec_()
            pass
        else:
            self.list_widget.addItem(item)
        
    def remove_selected_item_from_list(self):
        self.list_widget.takeItem(self.list_widget.currentRow())
        pass

    def call_robot(self):

        client = self.node.create_client(CallRobotToWaitingArea,"robot_caller")
        while not client.wait_for_service(1.0):
            self.node.get_logger().warn("Waiting for Server...")

        queue = []
        for row in range(self.list_widget.count()):
            queue.append(self.list_widget.item(row).text())

        request = CallRobotToWaitingArea.Request()
        request.user_number = self.user_number.text()
        request.queue = queue

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node,future)

        try:
            response = future.result()
            self.node.get_logger().info(str(response.selected_robot))
            self.node.get_logger().info(str(response.waiting_area_number))
            msg = QMessageBox()
            msg.setText("{} will come to meet you very soon at Waiting Area {}"\
                        .format(str(response.selected_robot),str(response.waiting_area_number)))
            msg.setWindowTitle("Call Message")
            msg.exec_()

        except Exception as e:
            self.node.get_logger.error("Service call failed %r" % (e,))

def main():
    app = QApplication([])
    window = RobotCallerUI()
    window.setWindowTitle('Robot Caller Interface')
    app.exec()
    

if __name__ == "__main__":
    main()