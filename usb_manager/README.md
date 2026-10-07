# USB Manager — Home Assistant App

<a href="https://buymeacoffee.com/leon_bell"><img src="https://img.shields.io/badge/Buy_Me_a_Beer-FFDD00?style=for-the-badge&logo=buymeacoffee&logoColor=black" alt="Buy me a beer"></a>
<a href="https://ko-fi.com/leonbell"><img src="https://img.shields.io/badge/Ko--fi-FF5E5B?style=for-the-badge&logo=kofi&logoColor=white" alt="Support on Ko-fi"></a>
<a href="https://paypal.me/leonbell95"><img src="https://img.shields.io/badge/PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white" alt="Donate with PayPal"></a>

[Buy me a beer](https://buymeacoffee.com/leon_bell) · [Ko-fi](https://ko-fi.com/leonbell) · [PayPal](https://paypal.me/leonbell95)

Automatically identify USB serial devices by their responses, assign them to
named virtual ports, and expose a stable TCP endpoint for each device.
Manage everything in the Web UI instead of editing YAML.

![USB Manager overview](https://raw.githubusercontent.com/thaihoang987/app-usb-manager/main/images/overview.png)

## Why this app exists

Home Assistant hosts often have several USB serial adapters plugged in at the
same time: RS-485 dongles for energy meters and relay boards, a solar inverter
link, an Arduino or ESP board on UART. Talking to them reliably is harder than
it looks:

- **`/dev/ttyUSB0` is not stable.** The kernel numbers adapters in the order it
  finds them, so after a reboot, an update or replugging a cable, the meter that
  was `ttyUSB0` can become `ttyUSB2` and your flows talk to the wrong device.
- **`by-id` and `by-path` do not always help.** Cheap CH340/CP210x/FTDI clones
  often share the same name and have no unique serial number, so two identical
  adapters look the same. `by-path` changes as soon as you move a cable to another
  USB port or hub.
- **One serial port, one owner.** Only one program can open a serial port
  safely. Node-RED, another container and a Home Assistant integration cannot all
  use the same adapter directly.

USB Manager was written to solve this on a real installation where several
identical RS-485 adapters kept swapping places.

## What it solves

- Each device gets a **name and a fixed TCP port** (6001–6030) that never changes,
  whatever tty number or USB socket it ends up on.
- Devices are recognised by **what they answer**, not by where they are plugged
  in, so identical adapters are told apart by the device behind them.
- Clients connect over TCP (Node-RED, Modbus tools, other containers, another
  computer) instead of fighting over `/dev/tty*`. Several clients can share one
  device.
- Unplug and replug, reboot or move a cable: the app finds the device again and
  the TCP port keeps working.

## How it works

1. **Scan.** The app looks at every USB serial device that is not excluded and
   not already claimed by another virtual port.
2. **Listen.** It first listens without sending anything, so a device that
   talks on its own, or a bus that already has a master, is not disturbed.
3. **Identify.** If the signature was not heard, it runs the port's
   identification steps in order. Each step sends one command (Raw text/hex, or a
   Modbus RTU read built with the right CRC), waits for that step's reply and
   compares it with that step's expected response. Optionally, Modbus replies must
   also have a valid CRC, unit ID and function code. The device is accepted only
   when **every** step matches.
4. **Bridge.** The matched device is claimed and a TCP server is opened on the
   port's fixed number. Bytes are passed through unchanged, or translated between
   Modbus TCP and Modbus RTU when that option is on.
5. **Recover.** If the device disappears, clients are disconnected, the port goes
   back to scanning and reconnects automatically when the device returns.

The logs show, for every USB device tried, which identification step matched and
which failed, with the bytes received.

## What you can use it for

- **RS-485 Modbus devices:** energy meters (for example PZEM-016), relay boards,
  sensors and inverters, read from Node-RED or any Modbus TCP client.
- **Several identical USB-RS485 adapters** that would otherwise swap tty numbers.
- **Arduino / ESP / custom UART devices** that answer an ID command such as
  `GET_ID$`, exposed as raw TCP.
- **Sharing one serial device** between Node-RED and other tools at the same time.
- **Exploring unknown devices:** capture replies, scan Modbus unit IDs and
  registers, and test matching rules before saving.

## Features

- Response-based identification with ordered **identification steps**: each step
  pairs one command with its own expected reply and wait time.
- Raw UART over TCP, and Modbus TCP ↔ Modbus RTU conversion.
- Web UI for ports, matching rules, timeouts, USB exclusions and optional MQTT
  discovery; Vietnamese and English; light and dark themes.
- Capture a real reply from a plugged-in device and test the whole identification
  sequence before saving.
- Automatic reconnect and rescan, traffic counters, activity log and Modbus tools.
- Persistent settings, configuration import/export, and live changes to
  individual ports.

## Install on Home Assistant (Hass.io)

Requires **Home Assistant OS with Supervisor**. Existing Supervised installations
also provide an Add-on Store. Container/Core cannot install this package as an app.

[![Add repository to Home Assistant](https://my.home-assistant.io/badges/supervisor_add_addon_repository.svg)](https://my.home-assistant.io/redirect/supervisor_add_addon_repository/?repository_url=https%3A%2F%2Fgithub.com%2Fthaihoang987%2Fapp-usb-manager)

1. Click **Add repository**, or open **Settings → Apps/Add-ons → Store → ⋮ → Repositories**.
2. Add `https://github.com/thaihoang987/app-usb-manager`.
3. Find **USB Manager**, click **Install**, then **Start** and **Open Web UI**.
4. Enable **Start on boot** and **Show in sidebar** if desired.
5. Review **Network** to expose the TCP ports your clients need (6001–6030).

Releases use prebuilt AMD64/ARM64 images.

## Quick start

1. Open **Configuration** → **Add port**. Fields marked with a red * are required.
2. Set a unique ID, display name, baud rate, Raw/Modbus protocol and TCP port.
3. In **Step 1**, enter a probe command and a distinctive expected response. Use
   `text:` for text or `hex:` for bytes. For Modbus, enter the unit ID and read
   parameters. Click **+ Add step** to require more command/response pairs; the
   device must match every step.
4. Under **Test with a connected USB device**, pick a plugged-in USB device, click
   **Send & capture response** on a step, then **Use as expected response**
   (Modbus: **Keep the fixed part only** if register values change). **Test all
   steps** checks the whole sequence.
5. Click **Keep port changes**, then **Save and apply**.
6. Connect Node-RED or another client to `HOME_ASSISTANT_IP:TCP_PORT`.

Choose **Modbus TCP ↔ RTU** only for a standard Modbus TCP client; leave it off
for clients that send raw RTU bytes. Exclude USB devices owned by other services
(for example a Zigbee coordinator). Use a distinctive response: a shared prefix
can match the wrong device.

See [Usage and migration](DOCS.md) for matching, backups and updates.
Switch the interface language under **Settings → Language**.

This is a personal project. Feature requests are considered when they fit the
project and time permits.

## Automatic releases

Push fixes to `dev`. GitHub Actions automatically increments the patch version,
builds AMD64/ARM64 images, verifies startup and public downloads, then updates
`main`, creates a version tag and publishes a GitHub Release. Failed builds do
not replace the installable version.
