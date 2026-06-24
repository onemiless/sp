#!/usr/bin/env python3
import argparse
import json
import socket
import time

from openpilot.common.params import Params


VALID_DIRECTIONS = {"left", "right", "cancel", "off"}


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Receive iOS Tesla blinker test commands over UDP.")
  parser.add_argument("--host", default="0.0.0.0", help="UDP bind host")
  parser.add_argument("--port", type=int, default=7788, help="UDP bind port")
  parser.add_argument("--token", default="", help="Optional shared token required in incoming JSON")
  parser.add_argument("--enable-control", action="store_true",
                      help="Set TeslaNavBlinkerControl=1 before listening. Restart comma/openpilot first if CarParamsSP was already built.")
  return parser.parse_args()


def clamp_duration(value: object) -> float:
  try:
    duration = float(value)
  except (TypeError, ValueError):
    duration = 1.5
  return max(0.1, min(duration, 5.0))


def main() -> None:
  args = parse_args()
  params = Params()

  if args.enable_control:
    params.put_bool("TeslaNavBlinkerControl", True)

  if not params.get_bool("TeslaNavBlinkerControl"):
    print("TeslaNavBlinkerControl is OFF.")
    print("Enable it first, then restart comma/openpilot so Tesla CarParamsSP includes the nav blinker flag.")
    print("Example: ./tools/tesla_nav_blinker_remote/remote_blinker_receiver.py --enable-control")
    return

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.bind((args.host, args.port))
  print(f"Listening for Tesla blinker test UDP commands on {args.host}:{args.port}")
  if args.token:
    print("Token check: enabled")
  else:
    print("Token check: disabled; only run this on a trusted local network")

  while True:
    data, addr = sock.recvfrom(2048)
    try:
      message = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
      print(f"ignored invalid packet from {addr}")
      continue

    if args.token and message.get("token") != args.token:
      print(f"ignored packet with bad token from {addr}")
      continue

    direction = str(message.get("direction", "")).lower()
    if direction not in VALID_DIRECTIONS:
      print(f"ignored unknown direction from {addr}: {direction!r}")
      continue

    duration = clamp_duration(message.get("duration"))
    command = {
      "direction": direction,
      "expires_at": time.monotonic() + duration,
      "source": "ios",
      "received_at": time.time(),
    }
    params.put("TeslaNavBlinkerTestCommand", json.dumps(command))
    print(f"{addr[0]}:{addr[1]} -> {direction} for {duration:.1f}s")


if __name__ == "__main__":
  main()
