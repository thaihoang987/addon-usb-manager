# USB Manager — Usage

<a href="https://buymeacoffee.com/leon_bell"><img src="https://img.shields.io/badge/Buy_Me_a_Beer-FFDD00?style=for-the-badge&logo=buymeacoffee&logoColor=black" alt="Buy me a beer"></a>
<a href="https://ko-fi.com/leonbell"><img src="https://img.shields.io/badge/Ko--fi-FF5E5B?style=for-the-badge&logo=kofi&logoColor=white" alt="Support on Ko-fi"></a>
<a href="https://paypal.me/leonbell95"><img src="https://img.shields.io/badge/PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white" alt="Donate with PayPal"></a>

[Buy me a beer](https://buymeacoffee.com/leon_bell) · [Ko-fi](https://ko-fi.com/leonbell) · [PayPal](https://paypal.me/leonbell95)

## Configure a port

Open **Configuration (Cấu hình)** and add a port. The ID is permanent and uses
letters, numbers, `_` or `-`; change the display name to rename an existing port.
Choose its baud rate, device protocol and TCP port (6001–6030).

- **Identification steps:** a port starts with one step; click **+ Add step**
  for more (up to 10). Each step sends its own command, waits its own time and
  is matched only against its own expected response. Steps run in order and the
  device is accepted only when **every** step matches; the first failing step
  stops the probe and the remaining steps are skipped.
- **Raw:** each step has a command such as `text:GET_ID$` and a matching response
  such as `text:DEVICE_A`. Leave the command empty to listen for data the device
  sends by itself.
- **Modbus RTU:** each step has its own unit ID, function code, start address and
  quantity. Read functions FC01–04 are useful probes. CRC is generated
  automatically. A write function changes the device every time the port is probed.
- **Expected response:** use `text:` or `hex:`. Put alternatives on separate
  lines; matching any line passes the step. Raw steps need an expected response.
- **Wait for reply:** seconds to wait for the step's reply; empty uses the
  probe timeout.
- **Check Modbus CRC + unit ID + function code:** Modbus option. On its own it
  accepts any valid reply from the step's unit/function; together with an
  expected response, the reply must match both. Each Modbus step needs at least one.
  Ports saved before this option keep CRC checking when no response was entered.
- **Matching:** Contains works for a reply inside a larger capture; Exact checks
  the entire buffer; Starts with checks the beginning; Fuzzy uses a similarity threshold.
- **Output:** Raw TCP preserves bytes. Enable Modbus TCP ↔ RTU only when clients
  send standard Modbus TCP frames. Advanced PTY output needs access to the generated
  serial path; use TCP for other containers.

In the port editor, pick a device under **Test with a connected USB device**
(devices held by other ports are disabled; the port's own device is shared
safely). **Send & capture response** on a step sends that step's command and
shows the reply with an immediate match check; **Use as expected response** fills
it in. **Test all steps** runs every step in order and reports which steps match.
You can also paste a reply into **Check rule against a captured response**; that
check sends nothing to USB and reports each step. Each port card shows the USB
device it currently matches. Test each physical device to ensure the rule
uniquely identifies it.

The add-on log and the **Log** tab report, per USB device, which step matched
and which failed (with the bytes received). A repeated identical result is
logged only once.

Ports configured before identification steps are converted automatically to one
step: the command, response 1 and response 2 (as an alternative line). The former
fallback command 2 is dropped, because every step must now match.

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
Supervisor has no runtime options form. Manage ports and USB exclusions in the
Web UI; network mappings remain in Supervisor. Before upgrading a legacy
installation, start its UI once to persist the imported options, or export a backup.
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

## Templates, language and time

Choose **Add port → Start from a template** for UART, listen-only UART, Modbus TCP/RTU, RTU over TCP or PTY. Review the device ID, baud, response signature and destination before saving.

Use **Settings → Language** to select Vietnamese or English. The choice is saved in your browser. Device names, commands and captured bytes stay unchanged.

The app uses the Home Assistant host clock and reads Home Assistant's timezone through the Supervisor proxy. When Core is temporarily unavailable, it keeps the last confirmed timezone or uses the timezone supplied by Supervisor.
