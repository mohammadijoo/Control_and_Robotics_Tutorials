# pip install pyzmq
import zmq, time, json, threading

CTX = zmq.Context()

def publisher(rate_hz=50):
    pub = CTX.socket(zmq.PUB)
    pub.bind("tcp://*:5556")
    k = 0
    T = 1.0 / rate_hz
    while True:
        msg = {"seq": k, "t_pub": time.time()}
        pub.send_string("imu " + json.dumps(msg))
        k += 1
        time.sleep(T)

def subscriber(mu_hz=40):
    sub = CTX.socket(zmq.SUB)
    sub.connect("tcp://localhost:5556")
    sub.setsockopt_string(zmq.SUBSCRIBE, "imu")
    T = 1.0 / mu_hz
    while True:
        topic, payload = sub.recv_string().split(" ", 1)
        msg = json.loads(payload)
        age = time.time() - msg["t_pub"]
        print(f"recv seq={msg['seq']} age={age:.4f}s")
        time.sleep(T)

threading.Thread(target=publisher, daemon=True).start()
subscriber()
      
