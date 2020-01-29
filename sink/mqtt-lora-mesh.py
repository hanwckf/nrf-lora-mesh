#!/usr/bin/python3
import time,sys,os,re
import serial,json
import paho.mqtt.client as mqtt
from threading import Thread
from queue import Queue

Tlink_SN = ""
USBserial = "/dev/ttyUSB0"
BaudRate = 115200

def on_connect(client, userdata, flags, rc):
	print("mqtt connection return: " + str(rc))

def serial_read(s, q):
	pattern = re.compile(r'__upd__\[(\d+)\]\[(\-?\d+\.\d+)\]\[(\d+)\]')
	while (serial_alive):
		b = s.readline()
		if b:
			b = b.decode('utf-8')
			r = pattern.findall(b)
			if r:
				q.put(r[0])
	print("serial thread exit")

def data2json(r):
	temp = { "flag": "temp_" + r[0], "value": float(r[1]) }
	volt = { "flag": "volt_" + r[0], "value": int(r[2]) }
	upload_data = { "sensorDatas": [ temp, volt ] }
	return json.dumps(upload_data)

def mqtt_success_hook():
	led_path = "/sys/class/leds/orangepi:red:status"
	cmd = """
	if [ \"$(cat {0}/brightness)\" = \"0\" ]; then
		echo 1 > {0}/brightness;
	else
		echo 0 > {0}/brightness;
	fi """.format(led_path)
	os.system(cmd)

if __name__ == '__main__':
	argc = len(sys.argv)
	if argc < 2:
		print ("please specify TlinkSN")
		sys.exit(1)
	Tlink_SN = sys.argv[1]
	if argc >= 3:
		USBserial = sys.argv[2]
		if argc >= 4:
			BaudRate = int(sys.argv[3])
	print("Serial port: " + USBserial)
	print("BaudRate: " + str(BaudRate))
	try:
		tty = serial.Serial(USBserial, BaudRate, timeout=3)
	except:
		print("Serial port open failed!")
		sys.exit(1)

	client = mqtt.Client(client_id=Tlink_SN)
	client.on_connect = on_connect
	client.username_pw_set("MQTT", password="MQTTPW")

	client.connect("mq.tlink.io", 1883, 60)
	client.loop_start()

	q = Queue()
	serial_alive = True
	t = Thread(target=serial_read, args=(tty,q,))
	t.start()

	try:
		while True:
			data = q.get(block=True, timeout=None)
			j = data2json(data)
			ret = client.publish(Tlink_SN, payload=j, qos=0)
			if ret.rc == mqtt.MQTT_ERR_NO_CONN:
				print("mqtt connection fail")
			elif ret.rc != mqtt.MQTT_ERR_SUCCESS:
				print("mqtt publish fail")
			else:
				mqtt_success_hook()

	except KeyboardInterrupt:
		client.loop_stop()
		serial_alive = False
		t.join()
