# Tesla nav blinker remote test

This is a bench/road-test helper for the Tesla navigation blinker control path.

It does not implement navigation. It lets an iPhone send short-lived test commands to the comma device:

- left
- right
- cancel
- off

The command expires on the comma device, so a lost phone connection does not leave a stuck request.

## comma side

Run this on the comma device from `/data/openpilot` or your repo path:

```bash
cd /data/sp
sudo systemctl stop comma
./tools/tesla_nav_blinker_remote/remote_blinker_receiver.py --enable-control
```

Then restart comma/openpilot once so `CarParamsSP` is rebuilt with `TeslaNavBlinkerControl=1`:

```bash
sudo systemctl restart comma
```

When you are ready to test, start the receiver again:

```bash
cd /data/sp
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

- This requires `TeslaNavBlinkerControl=1`.
- The command is ignored unless the current car brand is Tesla.
- The actual CAN transmit path still requires the Tesla vehicle bus and a valid stock `DAS_bodyControls` frame.
- Stop the receiver when you are done testing.
