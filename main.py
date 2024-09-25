import sys
from view.view import QtWidgets, ViewQVisualiser, pg
from threading import Thread
from controller import Controller

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    view = ViewQVisualiser(MainWindow)
    controller = Controller(view)
    controller_thread = Thread(target=controller.run, args={})
    controller_thread.start()
    MainWindow.show()
    sys.exit(app.exec_())
