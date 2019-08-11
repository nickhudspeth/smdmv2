import sys
import serial.tools.list_ports
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import time
from datetime import datetime
import lorem

serial_device = serial.Serial()


class SerialDeviceSelector(QWidget):
	def __init__(self, *args, **kwargs):
		super(SerialDeviceSelector, self).__init__(*args, **kwargs)
		self.ports = serial.tools.list_ports.comports()

		label = QLabel("Select Serial Port:")

		combobox = QComboBox()
		combobox.addItem("")
		for port in self.ports:
			combobox.addItem(port.description)
		combobox.currentIndexChanged.connect(self.connect_serial_device)

		layout = QVBoxLayout()
		layout.addWidget(label)
		layout.addWidget(combobox)

		self.setLayout(layout)
		self.setMinimumWidth(250)
		self.setMinimumHeight(self.sizeHint().height())
		self.setMaximumHeight(self.sizeHint().height())

	def connect_serial_device(self, device_index):
		if serial_device.open:
			serial_device.close()

		serial_device.port = self.ports[device_index - 1].device
		serial_device.baudrate = 115200
		try:
			serial_device.open()
		except:
			print(f"Error: Unable to open connection to device: {self.ports[device_index - 1]}")
			print(sys.exc_info()[0])
		else:
			print(f"Opened connection to device: {self.ports[device_index - 1]}")

class ActionItem(QWidget):
	def __init__(self, content=[], *args, **kwargs):
		super(ActionItem, self).__init__(*args, **kwargs)
		self.layout = QHBoxLayout()
		self.actionlabel = QLabel(content[0])
		self.layout.addWidget(self.actionlabel)

		
		self.setLayout(self.layout)

class LogWindow(QPlainTextEdit):
	def __init__(self, *args, **kwargs):
		super(LogWindow, self).__init__(*args, **kwargs)
		self.setReadOnly(True)
		self.verticalScrollBar().setValue(self.verticalScrollBar().maximum())
		self.setMinimumWidth(500)
		
		for i in range(0,100):
			self.logMessage(lorem.sentence())

	def logMessage(self, message):
		dt = datetime.now()
		self.appendPlainText(f"{str(dt.hour)}:{str(dt.minute)}:{str(dt.second)}:{str(round((dt.microsecond / 1000) % 1000))}\t{message}")

class MainWindow(QMainWindow):
	serial_device = serial.Serial()
	def __init__(self, *args, **kwargs):
		super(MainWindow, self).__init__(*args, **kwargs)
		self.setWindowTitle("SMDMv2 Tool")

		
		mainlayout = QHBoxLayout()

		leftlayout = QVBoxLayout()
		rightlayout = QVBoxLayout()
		rightlayout.setAlignment(Qt.AlignTop)

		widgets = [QCheckBox,
			QComboBox,
			QDateEdit,
			QDateTimeEdit,
			QDial,
			QDoubleSpinBox,
			QFontComboBox,
			QLCDNumber,
			QLabel,
			QLineEdit,
			QProgressBar,
			QPushButton,
			QRadioButton,
			QSlider,
			QSpinBox,
			QTimeEdit]

		sds = SerialDeviceSelector(self)
		logWindow = LogWindow(self)

		for w in widgets:
			leftlayout.addWidget(w())

		leftlayout.addWidget(ActionItem(["TestLabel"]))

		leftlayout.addWidget(logWindow)

		rightlayout.addWidget(sds)

		mainlayout.addLayout(leftlayout)
		mainlayout.addLayout(rightlayout)

		widget = QWidget()
		widget.setLayout(mainlayout)
		self.setCentralWidget(widget)
	def __del__(self):
		if serial_device.is_open == True:
			serial_device.close()
			print("Closed serial device.")

	def onWindowTitleChange(self, s):
		print(s)

	def my_custom_fn(self, a="HELLO", b=5):
		print(a,b)

	def contextMenuEvent(self, event):
		print("CME")
		super(MainWindow, self).contextMenuEvent(event)



def main():
	app = QApplication(sys.argv)

	window = MainWindow()
	window.show()

	app.exec_()

if __name__ == '__main__':
	main()
