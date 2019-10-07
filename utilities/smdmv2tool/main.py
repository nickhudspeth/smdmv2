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
		button = QPushButton("Connect")
		combobox.addItem("")
		for port in self.ports:
			combobox.addItem(port.description)
		combobox.currentIndexChanged.connect(self.connect_serial_device)

		layout = QVBoxLayout()
		layout.addWidget(label)
		layout.addWidget(combobox)
		layout.addWidget(button)

		self.setLayout(layout)
		self.setFixedWidth(250)
		self.setFixedHeight(self.sizeHint().height())

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
		self.command_char = content[0][1]
		self.layout = QHBoxLayout()
		self.actionlabel = QPushButton(content[0][0])
		self.actionlabel.setFixedWidth(130)
		self.layout.addWidget(self.actionlabel)
		for i in range(1,len(content)):
			label = QLabel(content[i][0])
			sb = QDoubleSpinBox()
			sb.setValue(content[i][1])
			sb.setFixedWidth(sb.sizeHint().width())
			self.layout.addWidget(label)		
			self.layout.addWidget(sb)
		self.setLayout(self.layout)

		self.actionlabel.clicked.connect(self.sendCommandString)

	def sendCommandString(self):
		msg = self.command_char
		children = self.findChildren(QDoubleSpinBox)
		for child in children:
			msg += (f",{child.value()}")
		MainWindow.logMessage(message=msg)
		print(msg)


class LogWindow(QPlainTextEdit):
	def __init__(self, *args, **kwargs):
		super(LogWindow, self).__init__(*args, **kwargs)
		self.setReadOnly(True)
		self.verticalScrollBar().setValue(self.verticalScrollBar().maximum())
		self.setMinimumWidth(500)
		self.setMinimumHeight(500 / (2*1.618))
		
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

		device_selector = SerialDeviceSelector(self)
		self.logWindow = LogWindow(self)

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



		# for w in widgets:
		# 	leftlayout.addWidget(w())

		actionitems = [	
			ActionItem([["Home", "H"], ["Velocity", 25.0], ["Offset", 0.0]]),
			ActionItem([["Move Absolute", "J"], ["Position", 0.0], ["Velocity", 25.0]]),
			ActionItem([["Move Continuous", "C"], ["Velocity", 25.0]]),
			ActionItem([["Move Relative", "K"], ["Position", 0.0], ["Velocity", 25.0]]),
			ActionItem([["Set Accelerations", "A"], ["Accel", 100.0], ["Decel", 100.0], ["Bow1", 1000.0], ["Bow2", 1000.0], ["Bow3", 1000.0], ["Bow4", 1000.0]]),
			ActionItem([["Set Current", "B"], ["Drive Current", 400]]),
			ActionItem([["Set Microstep Resolution", "E"], ["Resolution", 256]]),
			ActionItem([["Set Motion Parameters", "X"], ["Steps Per Revolution", 200], ["Lead", 10.0]]),
			ActionItem([["Set Velocity", "V"], ["Velocity", 25.0]])
		]
		for a in actionitems:
			leftlayout.addWidget(a)


		# leftlayout.addWidget(ActionItem(["Home", "Velocity", "Offset"]))
		# leftlayout.addWidget(ActionItem(["Move Absolute", "Position", "Velocity"]))
		# leftlayout.addWidget(ActionItem(["Move Continuous", "Velocity"]))
		# leftlayout.addWidget(ActionItem(["Move Relative", "Position", "Velocity"]))
		# leftlayout.addWidget(ActionItem(["Set Accelerations", "Accel", "Decel", "Bow1", "Bow2", "Bow3", "Bow4"]))
		# leftlayout.addWidget(ActionItem(["Set Current", "Drive Current"]))
		# leftlayout.addWidget(ActionItem(["Set Microstep Resolution", "Resolution"]))
		# leftlayout.addWidget(ActionItem(["Set Motion Parameters", "Steps Per Revolution", "Lead"]))
		# leftlayout.addWidget(ActionItem(["Set Velocity", "Velocity"]))


		leftlayout.addWidget(self.logWindow)

		rightlayout.addWidget(device_selector)
		#Add connect button 
		#Add firmware version display, populate on connect
		#Add current position display
		#Add device temperature display
		#Add system reset button
		#Add pause button
		#Add resume button
		#Add stop button
		#Add Emergency Stop button

		mainlayout.addLayout(leftlayout)
		mainlayout.addLayout(rightlayout)

		widget = QWidget()
		widget.setLayout(mainlayout)
		self.setCentralWidget(widget)

	def __del__(self):
		if serial_device.is_open == True:
			serial_device.close()
			print("Closed serial device.")

	def logMessage(self, message):
		self.logWindow.logMessage(message=message)



def main():
	app = QApplication(sys.argv)

	window = MainWindow()
	window.show()

	app.exec_()

if __name__ == '__main__':
	main()
