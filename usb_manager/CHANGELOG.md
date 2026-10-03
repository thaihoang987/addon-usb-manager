# Changelog

<a href="https://buymeacoffee.com/leon_bell"><img src="https://img.shields.io/badge/Buy_Me_a_Beer-FFDD00?style=for-the-badge&logo=buymeacoffee&logoColor=black" alt="Buy me a beer"></a>
<a href="https://ko-fi.com/leonbell"><img src="https://img.shields.io/badge/Ko--fi-FF5E5B?style=for-the-badge&logo=kofi&logoColor=white" alt="Support on Ko-fi"></a>
<a href="https://paypal.me/leonbell95"><img src="https://img.shields.io/badge/PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white" alt="Donate with PayPal"></a>

[Buy me a beer](https://buymeacoffee.com/leon_bell) · [Ko-fi](https://ko-fi.com/leonbell) · [PayPal](https://paypal.me/leonbell95)

## 1.12.1

- Maintenance update.
- Source: [b7e026f](https://github.com/thaihoang987/addon-usb-manager/commit/b7e026f04ca231d3fbeba70cf57defaf47de5c7a).

## Unreleased

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
