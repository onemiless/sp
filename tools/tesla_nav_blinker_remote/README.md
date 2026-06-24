# Tesla nav blinker remote test

This is a bench/road-test helper for the Tesla navigation blinker control path.

It does not implement navigation. It lets an iPhone send short-lived test commands to the comma device:

- left
- right
- cancel
- off

The command expires on the comma device, so a lost phone connection does not leave a stuck request.

## comma side

The receiver is managed by comma/openpilot and starts automatically when `TeslaNavBlinkerRemoteEnabled=1`.

It is enabled by default on this test branch.

After installing this branch, restart comma once:

```bash
cd /data/sp
sudo systemctl restart comma
```

To confirm it is running:

```bash
cd /data/sp
python3 - <<'PY'
from openpilot.common.params import Params
p = Params()
print("TeslaNavBlinkerRemoteEnabled:", p.get_bool("TeslaNavBlinkerRemoteEnabled"))
print("TeslaNavBlinkerControl:", p.get_bool("TeslaNavBlinkerControl"))
PY
```

If you want to run it manually for debugging, stop comma first so the managed process is not already using port `7788`:

```bash
cd /data/sp
sudo systemctl stop comma
./tools/tesla_nav_blinker_remote/remote_blinker_receiver.py
```

If you want a simple shared secret:

```bash
./tools/tesla_nav_blinker_remote/remote_blinker_receiver.py --token 123456
```

Use the same token in the iOS app.

## iPhone side

Open this project in Xcode:

```bash
open tools/tesla_nav_blinker_remote/ios/TeslaBlinkerRemote.xcodeproj
```

Select your iPhone as the run target. If Xcode asks for signing, set your Apple ID team under `Signing & Capabilities`.

Set:

- Host: the comma device Wi-Fi IP address
- Port: `7788`
- Token: empty, unless you started the receiver with `--token`

Tap `Left`, `Right`, or `Cancel`.

## Safety notes

- The managed receiver sets `TeslaNavBlinkerControl=1` automatically.
- To disable the receiver:

  ```bash
  cd /data/sp
  python3 common/params.py TeslaNavBlinkerRemoteEnabled 0
  python3 common/params.py TeslaNavBlinkerControl 0
  sudo systemctl restart comma
  ```

- The command is ignored unless the current car brand is Tesla.
- The actual CAN transmit path still requires the Tesla vehicle bus and a valid stock `DAS_bodyControls` frame.
