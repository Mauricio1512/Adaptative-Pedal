from pylsl import resolve_streams
from time import sleep, time
import numpy as np
import matplotlib.pyplot as plt
from modules.muse_stream import MuseStream
from modules.constants import LSL_EEG_CHUNK, SETTINGS_TOPIC, ACTIONS_TOPIC, TOGGLE_CELESTINO, TOGGLE_SOSTENUTO, DEFAULT_THRESHOLD
from modules.lsl_viewer import LSLViewer
import paho.mqtt.client as mqtt
import json

threshold = DEFAULT_THRESHOLD
calibrate = False

class MQTTConection:
    def __init__(self, adj_topic, host, port, keepalive):
        self.adj_topic = adj_topic

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_message = self.on_message
        self.client.on_connect = self.on_connect
        self.client.connect(host, port, keepalive)

    def start(self):
        print("MQTT Started")
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"Connected with result code {reason_code}")
        self.client.subscribe(self.adj_topic)

    def on_message(self, client, userdata, msg):
        global calibrate
        global threshold
        payload = json.loads(msg.payload.decode())

        if msg.topic == self.adj_topic and payload["action"] == "CHANGE_THRESHOLD":
            print(f"✅ Threshold changed to {payload['value']}")
            threshold = int(payload["value"])
        elif msg.topic == self.adj_topic and payload["action"] == "DEFAULT_THRESHOLD":
            print(f"✅ Threshold changed to default value {DEFAULT_THRESHOLD}")
        elif msg.topic == self.adj_topic and payload["action"] == "CALIBRATE":
            print(f"⚙️  Calibrating")
            calibrate = True

    def publish_action(self, action):
        self.client.publish(ACTIONS_TOPIC, action)

class Gyro:
    def __init__(self, gyro_stream: MuseStream, mqtt_conn: MQTTConection, ax):
        self.gyro_stream = gyro_stream
        self.gyro_stream.setup_ax(ax)
        self.startef = False    
        self.mqtt_conn = mqtt_conn

    def calibrate(self):
        print("Calibrating...")
        time_diff = 0
        data = np.zeros((self.gyro_stream.n_samples, self.gyro_stream.n_chan))
        while (time_diff < 5):
            samples, timestamps = self.gyro_stream.inlet.pull_chunk(timeout=0.5, max_samples=LSL_EEG_CHUNK)
            data = np.vstack([data, samples])
            time_diff += 0.5
        self.mean = np.mean(data, axis=0)
        print("Calibration offset:", self.mean)

    def detect(self):
        global threshold
        global calibrate
        self.started = True
        last_down = 0
        last_up = 0
        try:
            while self.started:
                if calibrate:
                    sleep(1)
                    self.calibrate()
                    calibrate = False
                    
                muse_stream = self.gyro_stream
                samples, timestamps = self.gyro_stream.inlet.pull_chunk(
                    timeout=0.1, max_samples=LSL_EEG_CHUNK
                )
                if timestamps:
                    timestamps = np.float64(np.arange(len(timestamps)))
                    timestamps /= muse_stream.sfreq
                    timestamps += muse_stream.times[-1] + 1.0 / muse_stream.sfreq

                    muse_stream.n_samples = int(muse_stream.sfreq * muse_stream.window)
                    muse_stream.times = muse_stream.times[-muse_stream.n_samples :]

                    muse_stream.data = np.vstack([muse_stream.data, samples])
                    muse_stream.data = muse_stream.data[-muse_stream.n_samples :]
                    plot_data = muse_stream.data - self.mean

                    for ii in range(muse_stream.n_chan):
                        if muse_stream.type == "GYRO":
                            # print(
                            #     f"{muse_stream.ch_names[ii]}:",
                            #     plot_data[-1, ii],
                            #     end=" ",
                            # )
                            if ii == 2:
                                # print("")
                                # * Select channel Y (1)
                                if (
                                    plot_data[-1, 1] > threshold
                                    and time() - last_up > 1
                                ):
                                    self.mqtt_conn.publish_action(TOGGLE_CELESTINO)
                                    print("☁️  Toggled celestino pedal")
                                    last_down = time()
                                    sleep(0.3)
                                elif (
                                    plot_data[-1, 1] < -threshold
                                    and time() - last_down > 1
                                ):
                                    self.mqtt_conn.publish_action(TOGGLE_SOSTENUTO)
                                    last_up = time()
                                    print("〰️ Toggled sostenuto pedal")
                                    sleep(0.3)

                        # muse_stream.lines[ii].set_xdata(
                        #     muse_stream.times[:: muse_stream.subsample]
                        #     - muse_stream.times[-1]
                        # )
                        # muse_stream.lines[ii].set_ydata(
                        #     plot_data[:: muse_stream.subsample, ii]
                        #     / muse_stream.scale
                        #     - 0
                        # )
                        # impedances = np.std(plot_data, axis=0)
                        # ticks_labels = [
                        #     "%s - %.2f" % (muse_stream.ch_names[ii], impedances[ii])
                        #     for ii in range(muse_stream.n_chan)
                        # ]
                        # muse_stream.axes.set_yticklabels(ticks_labels)
                        # muse_stream.axes.set_xlim(-muse_stream.window, 0)
                        # self.fig.canvas.draw()
            print("End")
        except RuntimeError as e:
            raise

def main():
    # * Setup MQTT connection
    mqtt_conection = MQTTConection(
        SETTINGS_TOPIC, "test.mosquitto.org", 1883, 60
    )
    mqtt_conection.start()
    # * Then search for the data that is being streamed
    streams = resolve_streams()

    if (len(streams) == 0):
        raise (RuntimeError("No streams found"))

    gyro_stream: MuseStream = None
    for stream in streams:
        if stream.type() == "GYRO":
            filter = False
            scale = 1
            window = 10
            gyro_stream = MuseStream(stream, filter, scale, window)
            print(gyro_stream)
            print("")

    # * Creating tbe canvas with all the plots we found in the stream
    figure = "15x6"
    figsize = np.int16(figure.split('x'))
    fig, axes = plt.subplots(1, figsize=figsize)
    gyro = Gyro(gyro_stream, mqtt_conection, axes)
    gyro.calibrate()
    gyro.detect()
    # lsl_viewer = LSLViewer(gyro_stream, fig, axes, gyro.mean)
    # lsl_viewer.start()

if __name__ == "__main__":
    main()
