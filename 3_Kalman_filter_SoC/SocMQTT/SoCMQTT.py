import paho.mqtt.client as mqtt
from datetime import datetime
import os,json
import pandas as pd


datetime_object = datetime.now()
date = datetime.strftime(datetime_object, "%d-%m")
headers = ['SOC','Current','Voltage','Temperature','Voltage Error']

def on_message(client, userdata, message):
    topic = message.topic
    decoded = str(message.payload.decode("utf-8"))
    m_json = json.loads(decoded)
    print("received message =",decoded)
    df = pd.DataFrame([m_json])
    if os.path.isfile(f"./logs/{date}.csv"):
        df.to_csv(f'./logs/{date}.csv', mode='a', header=False, index=False)
    else:
        df.to_csv(f'./logs/{date}.csv', mode='w', header=headers, index=False)

broker = 'broker.hivemq.com'
mqttUsername = 'ocbshoyv'
mqttPwd = 'u7RF9Xts1g1r'
mqttPort = 1883
mqttClient = mqtt.Client('Laptop', True)

# global mqttClient
mqttClient.on_message = on_message
# mqttClient.on_connect = on_connect
print("Connecting to", broker)
mqttClient.username_pw_set(username=mqttUsername, password=mqttPwd)
mqttClient.connect(broker, mqttPort)
print("Connected")
mqttClient.subscribe("Magnes/SOC",1)
print("Subscribed")

print("on_message function connected")
mqttClient.loop_forever()
print("After loop")