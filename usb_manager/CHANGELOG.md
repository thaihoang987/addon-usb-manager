# Changelog

<a href="https://buymeacoffee.com/leon_bell"><img src="https://img.shields.io/badge/Buy_Me_a_Beer-FFDD00?style=for-the-badge&logo=buymeacoffee&logoColor=black" alt="Buy me a beer"></a>
<a href="https://ko-fi.com/leonbell"><img src="https://img.shields.io/badge/Ko--fi-FF5E5B?style=for-the-badge&logo=kofi&logoColor=white" alt="Support on Ko-fi"></a>
<a href="https://paypal.me/leonbell95"><img src="https://img.shields.io/badge/PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white" alt="Donate with PayPal"></a>

[Buy me a beer](https://buymeacoffee.com/leon_bell) · [Ko-fi](https://ko-fi.com/leonbell) · [PayPal](https://paypal.me/leonbell95)

## 1.12.9

- README: screenshot, why the app exists, how it works and what it is for.
- Repository links point to `app-usb-manager`.

## 1.12.8

- Baud is now a list of common rates (300–921600) in the port editor, Get Response
  and the Modbus tool; the port editor also has **Other…** for a custom value.

## 1.12.7

- Light/dark theme button next to Settings (remembered per browser; first visit
  follows the system setting).
- Higher contrast and larger small text across all tabs; all text meets WCAG AA
  contrast in both themes.

## 1.12.6

- Port editor: larger, brighter labels and notes; still fits phone screens.

## 1.12.5

- Port editor: required fields are marked with a red * (the expected response
  is required for Raw, and for Modbus unless the CRC check is on).

## 1.12.4

- Identification steps: each step pairs one command with its own expected
  response and wait time. Start with one step and add more with **+ Add step**
  (up to 10); a device matches only when every step matches, in order.
- Modbus steps carry their own unit ID, function code, address and quantity; the
  CRC check applies to each step.
- Per-step **Send & capture response** and **Test all steps** in the port editor.
- Logs report which step matched and which failed for each USB device, once per
  change of result.
- Existing ports convert automatically to one step (response 2 becomes an
  alternative line). The fallback command 2 is removed.

## 1.12.3

- Port editor: pick a connected USB device and **Send & capture response** to fill
  the response signature from the real reply, with an immediate match check.
  Devices held by other ports are disabled; the edited port's own device is shared.
- Modbus: new **Check CRC + unit ID + function code** option. A Modbus port now
  needs a response signature, the CRC check, or both (both must then match).
  Existing Modbus ports without a response keep CRC identification.
- Port cards show the USB device each port currently matches.
- Checking a rule no longer requires the port ID to be filled in first.
- Raise the TCP port range from 6001–6010 to 6001–6030 (up to 30 TCP ports).

## 1.12.2

- Add ready-to-edit UART, Modbus and PTY templates, Vietnamese/English settings and Home Assistant time synchronization.
- Shorten interface instructions.

- Add a shared str() translation layer for static and dynamic UI text, accessible labels and confirmation dialogs; keep device data unchanged.

- Redesign the dashboard with live connection counters, responsive navigation, searchable port cards and a clearer configuration editor.

- Remove the duplicate Supervisor options form; configure ports and USB exclusions in the Web UI.
- Start correctly when Supervisor does not provide an options file.

## 1.12.0

- First public Home Assistant add-on release with AMD64 and ARM64 support.
- Manage all runtime settings through the Web UI, including ports, response
  matching, TCP/PTY output, Modbus conversion, USB exclusions and MQTT.
- Import existing options once; preserve virtual port IDs and matching rules.
- Apply changes to individual workers and keep the UI alive with no ports.
- Validate configuration before saving and support password-free import/export.
- Keep USB identification response-based and serialize device detection/claiming.
