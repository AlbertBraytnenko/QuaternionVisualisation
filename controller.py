import time
from threading import Lock

import numpy as np

from models.data_model import DataModel
from models.serial_model import SerialModel
from submodule.QLogic.src.QLogic import Quaternion


class Controller:
    def __init__(self, view):
        self.serial_model = SerialModel("COM10", 115200)
        self.serial_model.connect()
        self.data_model = DataModel()
        self.q = Quaternion(np.array([1, 0, 0, 0], dtype=np.float64))
        self.qW = Quaternion(np.array([1, 0, 0, 0], dtype=np.float64))
        self.lock = Lock()
        
        self.gyr_error = np.array([0, 0, 0], dtype=np.float64)
        self.counter = 0

        self.compensated_gyr_data = np.array([0, 0, 0], dtype=np.float64)
        self.geo_accel_data = np.array([0, 0, 0], dtype=np.float64)
        self.accel_q = Quaternion()
        self.previous_time = time.time()
        self.calc_gyr_error()
        self.view = view

        self.view.addActionsToMenu(self.serial_model.get_serial_ports())
        self.view.setActionFunc(self.view.menuPort, self.connectToCom)

    def get_q(self):
        return self.q

    def calc_gyr_error(self):
        print("Start error calculating...")
        try:
            for i in range(500):
                self.serial_model.read_data()
                print(f"\r{i}", end="")
                self.serial_model.process_data()
                if self.data_model.parse(self.serial_model.get_processed_data()):
                    self.gyr_error += self.data_model.angular_velocity
                    self.counter += 1
        
            self.gyr_error /= self.counter
            print(f"\n\nБіаси кутових швидкостей по осях: {self.gyr_error}\nКількість ітерацій розрахунку: {self.counter}")
        except:
            print("Connect esp32 with mpu6050")

    def update_view(self, enable_charts=False, enable_q=True):
        if ((t := time.time()) - self.previous_time) > 0.05:
            if enable_charts:
                self.view.set_acc_gyr_data(self.data_model.acceleration, self.compensated_gyr_data, self.q.to_numpy())
                # self.view.set_acc_gyr_data(self.data_model.acceleration, self.compensated_gyr_data,
                #                            np.array([0, *self.geo_accel_data], dtype=np.float64))
            if enable_q:
                self.view.qw.setValue(self.q.w)
                self.view.qx.setValue(self.q.x)
                self.view.qy.setValue(self.q.y)
                self.view.qz.setValue(self.q.z)
            self.previous_time = t

    def run(self):
        try:
            while True:
                self.serial_model.read_data()
                self.serial_model.process_data()
                self.data_model.parse(self.serial_model.get_processed_data())

                self.calculate_orientation()
                self.update_view(enable_charts=True, enable_q=True)
        except:
            print("Connect some esp32 with mpu6050!")

    def calculate_orientation(self):
            self.compensated_gyr_data = self.data_model.angular_velocity - self.gyr_error
            self.qW.set_vector_as_q(self.compensated_gyr_data)
            q = self.q * self.qW / 2
            self.q += q * 0.012
            self.q.normalize()

            self.accel_q.set_vector_as_q(self.data_model.acceleration)
            gel_accel = self.q * self.accel_q * self.q.conjugate
            self.geo_accel_data = gel_accel.vector_to_numpy() - np.array([0, 9.81, 0])

    def connectToCom(self, com):
        self.serial_model.disconnect()
        self.serial_model._serial_port = com
        self.serial_model.connect()

            
