import RPi.GPIO as GPIO
import time

class AlphaBot(object):
	
	def __init__(self,in1=13,in2=12,ena=6,in3=21,in4=20,enb=26):
		self.IN1 = in1
		self.IN2 = in2
		self.IN3 = in3
		self.IN4 = in4
		self.ENA = ena
		self.ENB = enb
		self.PA  = 20
		self.PB  = 20
		self.DR = 16
		self.DL = 19
		self.BUZ = 4

		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.IN1,GPIO.OUT)
		GPIO.setup(self.IN2,GPIO.OUT)
		GPIO.setup(self.IN3,GPIO.OUT)
		GPIO.setup(self.IN4,GPIO.OUT)
		GPIO.setup(self.ENA,GPIO.OUT)
		GPIO.setup(self.ENB,GPIO.OUT)
		GPIO.setup(self.DR,GPIO.IN,GPIO.PUD_UP)
		GPIO.setup(self.DL,GPIO.IN,GPIO.PUD_UP)
		GPIO.setup(self.BUZ,GPIO.OUT)
		self.PWMA = GPIO.PWM(self.ENA,500)
		self.PWMB = GPIO.PWM(self.ENB,500)
		self.PWMA.start(self.PA)
		self.PWMB.start(self.PB)
		self.stop()

	def forward(self):
		self.PWMA.ChangeDutyCycle(self.PA)
		self.PWMB.ChangeDutyCycle(self.PB)
		GPIO.output(self.IN1,GPIO.HIGH)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.HIGH)
		GPIO.output(self.IN4,GPIO.LOW)

	def stop(self):
		self.PWMA.ChangeDutyCycle(0)
		self.PWMB.ChangeDutyCycle(0)
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.LOW)

	def backward(self):
		self.PWMA.ChangeDutyCycle(self.PA)
		self.PWMB.ChangeDutyCycle(self.PB)
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.HIGH)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH)

	def left(self):
		self.PWMA.ChangeDutyCycle(30)
		self.PWMB.ChangeDutyCycle(30)
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.HIGH)
		GPIO.output(self.IN3,GPIO.HIGH)
		GPIO.output(self.IN4,GPIO.LOW)

	def right(self):
		self.PWMA.ChangeDutyCycle(30)
		self.PWMB.ChangeDutyCycle(30)
		GPIO.output(self.IN1,GPIO.HIGH)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH)
		
	def setPWMA(self,value):
		self.PA = value
		self.PWMA.ChangeDutyCycle(self.PA)

	def setPWMB(self,value):
		self.PB = value
		self.PWMB.ChangeDutyCycle(self.PB)	
		
	def setMotor(self, left, right):
		if((left >= 0) and (left <= 100)):
			GPIO.output(self.IN1,GPIO.HIGH)
			GPIO.output(self.IN2,GPIO.LOW)
			self.PWMA.ChangeDutyCycle(left)
		elif((left < 0) and (left >= -100)):
			GPIO.output(self.IN1,GPIO.LOW)
			GPIO.output(self.IN2,GPIO.HIGH)
			self.PWMA.ChangeDutyCycle(0 - left)
        
		if((right >= 0) and (right <= 100)):
			GPIO.output(self.IN3,GPIO.HIGH)
			GPIO.output(self.IN4,GPIO.LOW)
			self.PWMB.ChangeDutyCycle(right)
		elif((right < 0) and (right >= -100)):
			GPIO.output(self.IN3,GPIO.LOW)
			GPIO.output(self.IN4,GPIO.HIGH)
			self.PWMB.ChangeDutyCycle(0 - right)
	def checkIR(self):
		DR_status = GPIO.input(self.DR)
		DL_status = GPIO.input(self.DL)
		print(DR_status,DL_status)
		return DR_status, DL_status

	def beep_on(self):
		GPIO.output(self.BUZ,GPIO.HIGH)
		
	def beep_off(self):
		GPIO.output(self.BUZ,GPIO.LOW)

if __name__=='__main__':

	Ab = AlphaBot()
	Ab.forward()
	try:
		while True:
			time.sleep(1)
	except KeyboardInterrupt:
		GPIO.cleanup()
