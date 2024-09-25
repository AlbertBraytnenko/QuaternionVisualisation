# -*- coding: utf-8 -*-
#
# Created 10.05.2023
# By Bohdan Korzhak
#

import sys

sys.path.append("../QuaternionVisualisation")

import serial
import struct
from serial.tools.list_ports import comports

import config
import statuses


class SerialModel:
    """
    Serial Model для роботи з пакетами передачі даних через UART
    з використанням заголовків та контрольних сум CRC16
    """

    def __init__(self, serial_port: str = "COM1", baud_rate: int = 921600):
        self._available_serial = []
        self._serial_port = serial_port
        self._baud_rate = baud_rate
        self._is_serial_connected = False
        self.ser = None

        self._lost_connection = False

        self._obtained_bytes = None
        self._unpacked_data = None

    @staticmethod
    def get_serial_ports() -> list:
        """
        Method which returns list of available Serial ports.
        :return: list of Serial ports
        """
        return comports()

    def get_obtained_bytes(self):
        """
        Ґеттер для чистого отриманого масиву байт
        :return: bytes рядок
        """
        return self._obtained_bytes

    def get_processed_data(self):
        """
        Ґеттер для розпакованих оброблених даних
        :return: list розпакованих даних
        """
        return self._unpacked_data

    def is_serial_connected(self) -> bool:
        """
        Getter for _is_com_connected
        :return: True if Serial connected, False if not
        """
        return self._is_serial_connected

    def is_connection_lost(self) -> bool:
        """
        Повертає значення прапорця втраченого з'єднання
        :return: True, якщо з'єднання втрачено, False, якщо все добре
        """
        return self._lost_connection

    def reset_lost_connection_variable(self) -> None:
        """
        Скидає прапорець втраченого з'єднання
        :return: None
        """
        self._lost_connection = False

    def get_baud_rate(self) -> int:
        """
        Method for getting baud rate
        :return: current baud rate
        """
        return self._baud_rate

    def set_serial_port(self, serial_port: str) -> int:
        """
        Setter of Serial port.
        :param serial_port: str name of Serial ports
        :return: 0 if everything is OK,
                 1 if not (FAILED)
        """
        if isinstance(serial_port, str):
            self._serial_port = serial_port
            return statuses.OK
        else:
            return statuses.FAILED

    def set_baud_rate(self, br: int = 115200) -> int:
        """
        Method for setting baud rate
        :param br: can be 9600 - 115200 (default value)
        :return: 0 if everything is OK,
                 1 if not (FAILED)
        """
        if isinstance(br, int):
            self._baud_rate = br
            return statuses.OK
        else:
            return statuses.FAILED

    def connect(self) -> int:
        """
        Method which is connecting to Serial device.
        :return: 0 if everything is OK,
                 1 if SerialException (FAILED)
        """
        try:
            self.ser = serial.Serial(self._serial_port, self._baud_rate)
        except serial.SerialException:
            return statuses.FAILED
        else:
            self._is_serial_connected = True
            return statuses.OK

    def disconnect(self) -> int:
        """
        Method which is disconnecting from Serial device.
        :return: 0 if everything is OK,
                 1 if SerialException (FAILED)
        """
        try:
            self.ser.close()
        except:
            return statuses.FAILED
        else:
            self._is_serial_connected = False
            return statuses.OK

    def read_data(self):
        """
        Читаємо по байту, чекаємо на хедер. Після знаходження хедера читаємо
        32 байти та повертаємо їх в конкатенації з хедером
        :return: None
        """
        try:
            self._obtained_bytes = self.ser.readline()
        except:
            self._lost_connection = True
            self.disconnect()

    def process_data(self):
        """
        Розпакування даних, розрахунок CRC. Якщо все добре - повертаємо розпакований масив з даними
        :return: якщо масив байтів пустий - FAILED. Якщо розра
        """
        if not self._obtained_bytes:
            return statuses.FAILED

        try:
            # Розпакування даних
            self._unpacked_data = self._obtained_bytes.decode('utf-8').strip().split(',')
            return statuses.OK
        except ValueError:
            return statuses.FAILED

    def calculate_crc(self, data: bytes) -> int:
        """
        Calculating CRC16
        :param data: data for calculating CRC16
        :return: value of CRC16 (2 bytes)
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(0, 8):
                crc = (crc << 1) ^ config.POLYNOMIAL if (crc & 0x8000) else crc << 1
        return crc & 0xFFFF


# FOR TESTING OF CLASS
if __name__ == "__main__":
    sm = SerialModel()
    for i in sm.get_serial_ports():
        print(i)
