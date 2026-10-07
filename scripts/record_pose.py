import argparse
import json
from scservo_sdk import PortHandler, PacketHandler

parser = argparse.ArgumentParser()
parser.add_argument(
    "--output",
    help="Output JSON file",
    required=True
)
args = parser.parse_args()

out = {"left": {}, "right": {}}

for side, port in [("left", "/dev/ttyACM0"), ("right", "/dev/ttyACM1")]:
    ph = PortHandler(port)
    ph.openPort()
    ph.setBaudRate(1_000_000)

    pkt = PacketHandler(0)

    for i in range(1, 7):
        value, _, _ = pkt.read2ByteTxRx(ph, i, 56)
        out[side][str(i)] = int(value)

    ph.closePort()

with open(args.output, "w") as f:
    json.dump(out, f, indent=2)

print(f"Saved pose to {args.output}")
