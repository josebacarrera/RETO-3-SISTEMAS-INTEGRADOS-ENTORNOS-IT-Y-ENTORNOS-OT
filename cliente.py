import asyncio
from pymodbus.server import StartAsyncTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from threading import Thread
import sys
import os
import Adafruit_DHT
import RPi.GPIO as GPIO


try:

    store = ModbusSlaveContext(
        di = ModbusSequentialDataBlock(0, [1]*100),
        co = ModbusSequentialDataBlock(0, [2]*100),
        hr = ModbusSequentialDataBlock(0, [3]*100),
        ir = ModbusSequentialDataBlock(0, [4]*100))
    context = ModbusServerContext(slaves=store, single=True)

    identity = ModbusDeviceIdentification()
    identity.VendorName  = 'pymodbus'
    identity.ProductCode = 'PM'
    identity.VendorUrl   = 'http://github.com/bashwork/pymodbus/'
    identity.ProductName = 'pymodbus Server'
    identity.ModelName   = 'pymodbus Server'
    identity.MajorMinorRevision = '1.0'

    def program(x):

        SENSOR_DHT = Adafruit_DHT.DHT11
        dhtGpio = 21
        ledPin = 2

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ledPin, GPIO.OUT)
        heartbeat = 0
        while True:
                humedad, temperatura = Adafruit_DHT.read(SENSOR_DHT, dhtGpio)
                if humedad is not None and temperatura is not None:
                        heartbeat = heartbeat +1
                        os.system('clear')
                        print("Temp={0:0.1f}C Hum={1:0.1f}%".format(temperatura, humedad))
                        context  = x[1]
                        register = 3
                        slave_id = 0x00
                        address  = 0x00
                        values = context.getValues(register, address, count=3)
                        values[0] = temperatura
                        values[1] = humedad
                        context.setValues(register, 0, [heartbeat, int(temperatura), int(humedad)])
                        print(context.getValues(register, address, count=3))
                        if temperatura >= 19:
                                GPIO.output(ledPin,GPIO.HIGH)
                        else:
                                GPIO.output(ledPin, GPIO.LOW)


    thread_DHT11 = Thread(target=program, args=(context))
    thread_DHT11.daemon = True
    thread_DHT11.start()

    async def main():
        print("Iniciando Programa")
        await StartAsyncTcpServer(context, identity=identity, address=("192.168.1.155", 502))
        print("Servidor inicializado")
    asyncio.run(main())

except KeyboardInterrupt:
        GPIO.cleanup()