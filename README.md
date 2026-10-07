# USB Manager — Home Assistant Add-on

<a href="https://buymeacoffee.com/leon_bell"><img src="https://img.shields.io/badge/Buy_Me_a_Beer-FFDD00?style=for-the-badge&logo=buymeacoffee&logoColor=black" alt="Buy me a beer"></a>
<a href="https://ko-fi.com/leonbell"><img src="https://img.shields.io/badge/Ko--fi-FF5E5B?style=for-the-badge&logo=kofi&logoColor=white" alt="Support on Ko-fi"></a>
<a href="https://paypal.me/leonbell95"><img src="https://img.shields.io/badge/PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white" alt="Donate with PayPal"></a>

[Buy me a beer](https://buymeacoffee.com/leon_bell) · [Ko-fi](https://ko-fi.com/leonbell) · [PayPal](https://paypal.me/leonbell95)

Automatically identify USB serial devices by their responses, assign them to
named virtual ports, and expose a stable TCP endpoint for each device.
Manage configuration in the Web UI instead of editing YAML.

## Features

- Response-based identification: distinguish similar USB adapters even when tty numbers change.
- Raw UART over TCP and Modbus TCP ↔ Modbus RTU conversion.
- Web UI for ports, matching rules, timeouts, USB exclusions and optional MQTT discovery.
- Automatic reconnect and rescan, traffic counters, logs and communication tools.
- Persistent settings, configuration import/export, and live changes to individual ports.

## Install on Home Assistant (Hass.io)

Requires **Home Assistant OS with Supervisor**. Existing Supervised installations
also provide an Add-on Store. Container/Core cannot install this package as an add-on.

[![Add repository to Home Assistant](https://my.home-assistant.io/badges/supervisor_add_addon_repository.svg)](https://my.home-assistant.io/redirect/supervisor_add_addon_repository/?repository_url=https%3A%2F%2Fgithub.com%2Fthaihoang987%2Faddon-usb-manager)

1. Click **Add repository**, or open **Settings → Apps/Add-ons → Store → ⋮ → Repositories**.
2. Add `https://github.com/thaihoang987/addon-usb-manager`.
3. Find **USB Manager**, click **Install**, then **Start** and **Open Web UI**.
4. Enable **Start on boot** and **Show in sidebar** if desired.
5. Review **Network** to expose the TCP ports your clients need (6001–6030).

The initial release can build locally during installation. Later releases use
prebuilt AMD64/ARM64 images when the release workflow has finished.

## Quick start

1. Open **Configuration (Cấu hình)** → **Add port (Thêm port)**.
2. Set a unique ID, display name, baud rate, Raw/Modbus protocol and TCP port.
3. Enter a probe command and a distinctive expected response. Use `text:` for
   text or `hex:` for bytes. For Modbus, enter the unit ID and read parameters.
4. Use **Get Response** to inspect real replies, then check your matching rule.
5. Click **Keep port changes (Giữ thay đổi port)**, then **Save and apply (Lưu và áp dụng)**.
6. Connect Node-RED or another client to `HOME_ASSISTANT_IP:TCP_PORT`.

Choose **Modbus TCP ↔ RTU** only for a standard Modbus TCP client; leave it off
for clients that send raw RTU bytes. Exclude USB devices owned by other services.
Use a distinctive response: a shared prefix can match the wrong device.

See [Usage and migration](usb_manager/DOCS.md) for matching, backups and updates.
The current Web UI uses Vietnamese labels.

This is a personal project. Feature requests are considered when they fit the
project and time permits.

## Automatic releases

Push fixes to `dev`. GitHub Actions automatically increments the patch version,
builds AMD64/ARM64 images, verifies startup and public downloads, then updates
`main`, creates a version tag and publishes a GitHub Release. Failed builds do
not replace the installable version. The first release keeps the initial version.
