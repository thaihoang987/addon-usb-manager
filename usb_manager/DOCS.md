# USB Manager — Usage

<a href="https://buymeacoffee.com/leon_bell"><img src="https://img.shields.io/badge/Buy_Me_a_Beer-FFDD00?style=for-the-badge&logo=buymeacoffee&logoColor=black" alt="Buy me a beer"></a>
<a href="https://ko-fi.com/leonbell"><img src="https://img.shields.io/badge/Ko--fi-FF5E5B?style=for-the-badge&logo=kofi&logoColor=white" alt="Support on Ko-fi"></a>
<a href="https://paypal.me/leonbell95"><img src="https://img.shields.io/badge/PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white" alt="Donate with PayPal"></a>

[Buy me a beer](https://buymeacoffee.com/leon_bell) · [Ko-fi](https://ko-fi.com/leonbell) · [PayPal](https://paypal.me/leonbell95)

## Configure a port

Open **Configuration (Cấu hình)** and add a port. The ID is permanent and uses
letters, numbers, `_` or `-`; change the display name to rename an existing port.
Choose its baud rate, device protocol and TCP port (6001–6010).

- **Raw:** enter `text:GET_ID$` or another device-specific command and a matching
  response such as `text:DEVICE_A`. Leave the command empty for passive listening.
- **Modbus RTU:** enter unit ID, function code, start address and quantity.
  Read functions FC01–04 are useful probes. CRC is generated automatically.
  A write function changes the device every time the port is probed.
- **Expected response:** use `text:` or `hex:`. The two response fields are
  alternatives (OR). With both empty, Modbus identification checks CRC, unit ID
  and function code. Raw ports need an expected response before enabling.
- **Matching:** Contains works for a reply inside a larger capture; Exact checks
  the entire buffer; Starts with checks the beginning; Fuzzy uses a similarity threshold.
- **Output:** Raw TCP preserves bytes. Enable Modbus TCP ↔ RTU only when clients
  send standard Modbus TCP frames. Advanced PTY output needs access to the generated
  serial path; use TCP for other containers.

Use **Get Response** to capture a real device reply. Paste it into **Check rule
against a captured response** in the port editor; that check sends nothing to USB.
Test each physical device to ensure the rule uniquely identifies it.

**Keep port changes** updates the draft. **Save and apply** validates and saves
the configuration. Only changed ports restart; their TCP clients must reconnect.
Changing scan patterns, exclusions or scan defaults restarts the port workers.
Changing log or MQTT settings does not restart the USB bridges.

## General settings

Expand **General settings** for scan patterns, excluded USB devices, passive
listening duration, probe/rescan defaults, startup delay, logging and MQTT.
Select an unrelated USB device to exclude it using a stable by-path/by-id path.
Device identification itself remains response-based; it is not bound to by-path.

MQTT discovery is optional. Supply broker settings in the UI, or leave the host
empty to use the environment if available. A blank new password keeps the saved
password; **Clear password** removes it. Configuration exports omit the password.
The startup delay takes effect on the next add-on start.

## Storage and migration

Settings live in `/data/usb-manager-config.json` and survive restarts/updates.
Existing Supervisor options are imported once when this file does not exist.
Later Supervisor option edits do not replace the configuration saved by the UI.
New installs start with no ports and keep the UI running so you can add the first.

**Export backup** exports the current form without the MQTT password.
**Import backup** loads a draft; review it and click **Save and apply**.
Concurrent edits are protected by a configuration revision.

Installing this repository creates a separate add-on from a local installation.
Export from the old UI, stop the old add-on, import into the new one, re-enter
MQTT credentials if needed, and save. Confirm TCP connections before removing
the old installation. Two services must not control the same USB at the same time.

Network mappings and hardware permissions are managed by Supervisor.
The add-on requires access to the serial devices; if Home Assistant requests
it, disable Protection mode for this add-on. USB ports are available to the
Home Assistant host, not the computer displaying the UI.
