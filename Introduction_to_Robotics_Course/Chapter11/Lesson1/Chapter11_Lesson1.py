# L0: driver abstraction
class EncoderDriver:
    def __init__(self, counts_per_rev=4096):
        self.cpr = counts_per_rev
        self.counts = 0

    def read_counts(self):
        return self.counts

    def write_motor_pwm(self, pwm):
        # placeholder for real hardware output
        self.counts += int(0.1 * pwm)

# L1: middleware-like bus
class MessageBus:
    def __init__(self):
        self.subs = {}

    def publish(self, topic, msg):
        for cb in self.subs.get(topic, []):
            cb(msg)

    def subscribe(self, topic, cb):
        self.subs.setdefault(topic, []).append(cb)

# L2: algorithm (P controller)
class PositionController:
    def __init__(self, bus, kp=0.2):
        self.bus = bus
        self.kp = kp
        self.ref = 0.0
        bus.subscribe("ref", self.set_ref)
        bus.subscribe("pos", self.on_pos)

    def set_ref(self, r):
        self.ref = r

    def on_pos(self, pos):
        u = self.kp * (self.ref - pos)
        self.bus.publish("cmd_pwm", u)

# L3: application (sets reference)
class App:
    def __init__(self, bus):
        self.bus = bus

    def step(self, t):
        r = 10.0 if t > 1.0 else 0.0
        self.bus.publish("ref", r)

# wire stack
drv = EncoderDriver()
bus = MessageBus()
ctrl = PositionController(bus)
app = App(bus)

# connect L0 <-> L1
bus.subscribe("cmd_pwm", drv.write_motor_pwm)

def driver_tick():
    counts = drv.read_counts()
    pos = counts / drv.cpr * 2.0 * 3.14159
    bus.publish("pos", pos)

# simulate
dt = 0.01
t = 0.0
for k in range(400):
    app.step(t)
    driver_tick()
    t += dt
      
