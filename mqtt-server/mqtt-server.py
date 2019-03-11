import time
import ttn
import json

app_id = "<your TTN application id>"
access_key = "<your TTN application access key>"


def uplink_callback(msg, client):
    print('wind_analog: ', msg.payload_fields.analog_in_5)
    print('temp: ', msg.payload_fields.temperature_1)
    print('press: ', msg.payload_fields.barometric_pressure_3)
    print('humid: ', msg.payload_fields.relative_humidity_2)


handler = ttn.HandlerClient(app_id, access_key)

# using mqtt client
mqtt_client = handler.data()
mqtt_client.set_uplink_callback(uplink_callback)

while True:
    mqtt_client.connect()
    time.sleep(60)
# mqtt_client.close()

# using application manager client
app_client = handler.application()
my_app = app_client.get()
# print(my_app)
my_devices = app_client.devices()
# print(my_devices)
