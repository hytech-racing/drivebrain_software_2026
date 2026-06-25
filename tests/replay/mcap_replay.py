from mcap.reader import make_reader
import zmq, time

WHITELIST = None
BLACKLIST = ("estimator_Outports")

ctx = zmq.Context()
sock = ctx.socket(zmq.PUSH)
sock.connect("ipc:///tmp/drivebrain_sim_6767")

input("drivebrain up? press enter to start replay...")

with open("mcap/277-recovered.mcap", "rb") as f:
    reader = make_reader(f)
    wall_start = time.monotonic()
    mcap_start = None
    for schema, channel, message in reader.iter_messages():
        print("Sending message: " + channel.topic)
        if channel.message_encoding != "protobuf":
            continue
        if channel.topic in BLACKLIST:
            continue
        if mcap_start is None:
            mcap_start = message.log_time
        target = wall_start + (message.log_time - mcap_start) / 1e9
        dt = target - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        sock.send_multipart([schema.name.encode(), message.data])

sock.close()
ctx.term()