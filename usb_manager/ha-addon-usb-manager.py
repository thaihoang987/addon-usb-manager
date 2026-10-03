#!/usr/bin/env python3
"""
USB Manager Distribution
=========================
Tu dong do cac thiet bi serial that (/dev/ttyUSB*, /dev/ttyACM*) khop voi tung
"port ao" khai bao trong config, roi mo 1 TCP server rieng cho moi port ao de
Node-RED / add-on khac (vd ModbusSpy) ket noi toi - giong het kieu ModbusSpy
dang lam voi Modbus TCP :5020.

Khong app nao khac duoc phep mo thang /dev/ttyUSBx - tat ca di qua TCP bridge
cua add-on nay, nen khong bao gio bi xung dot "port dang bi chiem".

Vi 2 thiet bi CH340 giong het nhau co the tao ra cung 1 ten /dev/serial/by-id/
va /dev/ttyUSBx co the doi so thu tu sau moi lan reboot, add-on nay KHONG dua
vao by-id/by-path de nhan dien - no gui thu 1 lenh test that (do nguoi dung
cau hinh) toi tung /dev/ttyUSB*/ttyACM* con trong, roi so khop phan hoi de xac
dinh dung thiet bi cho tung port ao.
"""

import glob
import http.server
import json
import logging
import os
import pty as pty_module
import select
import tempfile
import re
import socket
import struct
import threading
import time
from collections import deque
from difflib import SequenceMatcher

import serial

OPTIONS_PATH = "/data/options.json"
NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")
SPY_HTTP_PORT = 8099

# LAST_MATCHED_PATH: ghi nho port da khop thanh cong lan gan nhat cho tung
# port ao (theo ten), luu trong /data (song song qua restart/update add-on).
# Lan do lai sau (ke ca sau khi restart container) se UU TIEN thu lai dung
# port nay TRUOC TIEN - neu van con khop thi xong ngay, khong can quet het
# moi ung vien tu dau; neu khong con khop (vd thiet bi da doi cho) thi tu
# dong quay lai quet binh thuong nhu cu.
LAST_MATCHED_PATH = "/data/last_matched_ports.json"
_last_matched_lock = threading.Lock()


def load_options() -> dict:
    with open(OPTIONS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def get_addon_version() -> str:
    """Doc truong 'version:' tu config.yaml nam CANH file .py nay - khong
    dung PyYAML (khong co trong requirements.txt, chi de doc 1 dong don
    gian thi khong dang them dependency) - tu parse bang tay dong dau tien
    khop 'version:'. Tra ve 'unknown' neu khong doc duoc (KHONG duoc lam
    sap add-on chi vi khong hien thi duoc so version)."""
    try:
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
        with open(config_path, "r", encoding="utf-8") as f:
            for line in f:
                stripped = line.strip()
                if stripped.startswith("version:"):
                    return stripped.split(":", 1)[1].strip().strip('"').strip("'")
    except OSError:
        pass
    return "unknown"


def load_last_matched() -> dict:
    try:
        with open(LAST_MATCHED_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def save_last_matched(vport_name: str, path: str) -> None:
    with _last_matched_lock:
        cache = load_last_matched()
        cache[vport_name] = path
        try:
            with open(LAST_MATCHED_PATH, "w", encoding="utf-8") as f:
                json.dump(cache, f, ensure_ascii=False)
        except OSError as exc:
            logging.warning("Khong ghi duoc %s: %s", LAST_MATCHED_PATH, exc)


# ---------------------------------------------------------------------------
# Modbus RTU helpers (protocol: modbus_rtu) - tu dong build frame + kiem CRC,
# nguoi dung chi can dien unit_id/function_code/start_address/quantity, khong
# phai tu tinh CRC bang tay.
# ---------------------------------------------------------------------------

def modbus_crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack("<H", crc)


# FC ghi (write) dung "value" thay vi "quantity" - khung khac han FC doc.
_WRITE_SINGLE_COIL = 5      # FC 0x05 - Write Single Coil (value: 1=ON/0=OFF)
_WRITE_SINGLE_REGISTER = 6  # FC 0x06 - Write Single Register (value: so nguyen 16-bit)


def build_modbus_request(unit_id: int, function_code: int, start_address: int,
                          quantity: int = 1, value: int | None = None) -> bytes:
    if function_code == _WRITE_SINGLE_COIL:
        coil_value = 0xFF00 if value else 0x0000
        body = struct.pack(">BBHH", unit_id, function_code, start_address, coil_value)
    elif function_code == _WRITE_SINGLE_REGISTER:
        body = struct.pack(">BBHH", unit_id, function_code, start_address, value or 0)
    else:
        # FC doc (0x01/02/03/04): dung quantity nhu cu
        body = struct.pack(">BBHH", unit_id, function_code, start_address, quantity)
    return body + modbus_crc16(body)


def check_modbus_response(unit_id: int, function_code: int, response: bytes) -> bool:
    if len(response) < 5:
        return False
    payload, recv_crc = response[:-2], response[-2:]
    if modbus_crc16(payload) != recv_crc:
        return False
    if response[0] != unit_id:
        return False
    # cho phep ca ma loi (function_code | 0x80) - van la bang chung co thiet bi
    # Modbus that tra loi dung khuon dang, chi la bao loi thanh ghi/dia chi.
    if response[1] not in (function_code, function_code | 0x80):
        return False
    return True


def passive_scan_modbus(buffer: bytes, unit_id: int, function_code: int,
                         min_len: int = 4, max_len: int = 64) -> bool:
    """
    Do tim 1 khung Modbus RTU HOP LE (CRC dung, DUNG unit_id VA DUNG
    function_code o byte thu 2) o bat ky vi tri nao trong 'buffer' da nghe
    len duoc thu dong - KHONG tu gui gi len bus ca. Dung cho bus dang co
    master that (vd datalogger Growatt) hoat dong that su, tranh dung
    collision tren RS485 half-duplex.

    Kiem tra ca function_code (khong chi unit_id) de giam rui ro nham thiet
    bi giua 2 bus RS-485 KHAC NHAU nhung tinh co cung dung unit_id mac dinh
    (vd unit_id=1) - neu chi kiem unit_id, mot thiet bi khac tren 1 bus khac
    (vd relay board dung FC3) van co the bi nham la khop.
    """
    n = len(buffer)
    for start in range(n):
        if buffer[start] != unit_id:
            continue
        max_frame_len = min(max_len, n - start)
        for length in range(min_len, max_frame_len + 1):
            frame = buffer[start:start + length]
            payload, crc = frame[:-2], frame[-2:]
            if modbus_crc16(payload) == crc and frame[1] == function_code:
                return True
    return False


# ---------------------------------------------------------------------------
# MBAP <-> RTU bridge (option "mbap_rtu_bridge": true tren port ao, MAC DINH
# false = passthrough byte nguyen si nhu cu, KHONG doi gi ca). Chi bat khi
# client TCP (vd node-red-contrib-modbus, clienttype "tcp") noi chuyen theo
# khung Modbus-TCP (header MBAP 6 byte: transaction_id+protocol_id+length,
# KHONG CRC) nhung thiet bi RTU that tren serial lai can khung RTU (CO
# CRC16, KHONG header MBAP) - 2 khung nay KHAC NHAU hoan toan, gui thang
# khung MBAP xuong serial thi thiet bi RTU se im lang (sai khung, khong
# phai loi bus/day). Bat option nay de add-on tu dich 2 chieu; port nao
# khong bat (mac dinh) van passthrough 100% nhu truoc, dung cho protocol
# "raw" (vd Arduino) hoac cac thiet bi tu lo CRC/khung rieng.
# ---------------------------------------------------------------------------

_MBAP_HEADER_LEN = 6  # transaction_id(2) + protocol_id(2) + length(2)


def mbap_frame_to_rtu(mbap_data: bytes) -> tuple[bytes, bytes, int] | None:
    """Boc 1 khung MBAP HOAN CHINH o dau 'mbap_data', tra ve (transaction_id,
    khung RTU tuong ung DA TINH CRC, so byte MBAP da "an" - de ben goi tu cat
    khoi buffer). Tra None neu chua du byte (can doc/nhan them, KHONG phai
    loi) - goi lai voi buffer day hon sau."""
    if len(mbap_data) < _MBAP_HEADER_LEN:
        return None
    if mbap_data[2:4] != b"\x00\x00":
        raise ValueError("Invalid Modbus TCP protocol ID")
    transaction_id = mbap_data[0:2]
    length = struct.unpack(">H", mbap_data[4:6])[0]
    if not 2 <= length <= 254:
        raise ValueError("Invalid Modbus TCP length")
    consumed = _MBAP_HEADER_LEN + length
    if len(mbap_data) < consumed:
        return None
    pdu = mbap_data[_MBAP_HEADER_LEN:consumed]  # unit_id+fc+data
    return transaction_id, pdu + modbus_crc16(pdu), consumed


def rtu_response_expected_len(buf: bytes) -> int | None:
    """Doan tong so byte 1 khung RTU response CAN CO (ke ca 2 byte CRC cuoi),
    dua vao function_code o byte thu 2. Tra None neu CHUA DU du lieu de doan
    (can doc them, KHONG phai loi), hoac function_code la khong doan duoc
    (goi lai se fallback ve passthrough nguyen ban cho lan do do)."""
    if len(buf) < 2:
        return None
    function_code = buf[1]
    if function_code & 0x80:
        return 5  # exception: unit_id+fc+exception_code+crc(2)
    if function_code in (1, 2, 3, 4):
        if len(buf) < 3:
            return None
        return 3 + buf[2] + 2  # unit_id+fc+byte_count+data+crc(2)
    if function_code in (5, 6, 15, 16):
        return 8  # unit_id+fc+addr(2)+value_or_qty(2)+crc(2)
    return None


def rtu_frame_to_mbap(rtu_frame: bytes, transaction_id: bytes) -> bytes:
    """Ghep 1 khung RTU HOP LE (da kiem CRC truoc do) thanh khung MBAP de tra
    ve cho client TCP dang doi theo dinh dang Modbus-TCP."""
    pdu = rtu_frame[:-2]  # bo 2 byte CRC
    return transaction_id + b"\x00\x00" + struct.pack(">H", len(pdu)) + pdu


# ---------------------------------------------------------------------------
# Raw protocol helpers (protocol: raw) - nguoi dung tu dien lenh gui/nhan,
# co the la "hex:01 04 00 0A" hoac "text:PING".
# ---------------------------------------------------------------------------

def decode_command(value: str) -> bytes:
    if value is None:
        return b""
    if value.startswith("hex:"):
        hexstr = value[len("hex:"):].strip().replace(" ", "")
        return bytes.fromhex(hexstr)
    if value.startswith("text:"):
        text = value[len("text:"):]
        # Cho phep go tay "\n"/"\r"/"\t" (2 ky tu: backslash + chu) qua giao
        # dien Configuration cua HA (form JSON khong tu dich escape sequence
        # nhu YAML lam) - tu dich thanh ky tu xuong dong/tab THAT truoc khi
        # encode. Neu file config.yaml da dung chuoi YAML co dau ngoac kep
        # (vd "text:\n") thi \n o do da la ky tu that san roi, .replace() voi
        # chuoi 2-ky-tu "\\n" se khong dung phai, khong anh huong gi ca.
        text = text.replace("\\r\\n", "\r\n").replace("\\n", "\n").replace("\\t", "\t")
        return text.encode("utf-8")
    return value.encode("utf-8")


def describe_bytes(data: bytes) -> str:
    """Hien ca dang text (de doc) lan hex (chinh xac tung byte) cho log,
    thay vi chi in hex kho doc. Ky tu khong in duoc thay bang '.'."""
    text = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
    return f"text: {text!r} | hex: {data.hex()}"


def response_matches(expected_raw: str, actual: bytes, mode: str, threshold: int) -> bool:
    expected_bytes = decode_command(expected_raw)
    if mode == "exact":
        return actual == expected_bytes
    if mode == "contains":
        return expected_bytes in actual
    if mode == "startswith":
        # Chi khop dung VI TRI DAU (vd 2 byte unit_id+function_code cua
        # Modbus RTU o dau khung) - khac "contains" la tim CHUOI o BAT KY
        # dau trong toan bo response, ke ca lot vao trong phan data. Vd 1
        # thiet bi Modbus khac (fc that = 3) co the tinh co co data chua
        # dung 2 byte "01 04" o giua khung - "contains" se khop nham, con
        # "startswith" thi khong vi 2 byte dau that su cua no la "01 03".
        return actual[:len(expected_bytes)] == expected_bytes
    if mode == "fuzzy":
        a = actual.hex()
        b = expected_bytes.hex()
        ratio = SequenceMatcher(None, a, b).ratio() * 100
        return ratio >= threshold
    return False


# ---------------------------------------------------------------------------
# Virtual port
# ---------------------------------------------------------------------------

class VirtualPort:
    def __init__(self, cfg: dict, defaults: dict):
        self.name = cfg["name"]
        if not NAME_RE.match(self.name):
            raise ValueError(
                f"Ten port ao khong hop le: {self.name!r} "
                "(chi cho phep chu khong dau, so, '-' hoac '_', khong dau cach)"
            )
        # friendly_name: CHI dung cho ten hien thi tren entity HA (MQTT
        # Discovery) - "name" o tren la ID CO DINH, dung cho unique_id/topic
        # MQTT + cache last_matched_ports.json + claimed dict, KHONG duoc doi
        # theo friendly_name (xem TROUBLESHOOTING_NOTES.md muc 17b).
        self.friendly_name = cfg.get("friendly_name") or self.name
        self.enabled = cfg.get("enabled", True)
        self.protocol = cfg.get("protocol", "raw")
        self.baud = int(cfg.get("baud", 9600))
        if self.baud <= 0:
            raise ValueError(f"[{self.name}] baud must be positive")
        self.output_mode = cfg.get("output_mode", "tcp")
        self.tcp_port = cfg.get("tcp_port")
        self.pty_symlink = cfg.get("pty_symlink")
        # on_connect_send: TUY CHON, chi dung cho output_mode=tcp. Gui dung 1 lan
        # ngay khi client TCP vua ket noi xong (truoc khi co du lieu that tu serial),
        # KHONG gui xuong serial that. Muc dich: mot so client (vd Node-RED "tcp in"
        # dang "Connect to") chi phat sinh msg (kem _session de dung "Reply to" gui
        # nguoc) khi NHAN duoc byte dau tien - neu thiet bi that (vd Arduino) chi
        # tra loi khi duoc hoi thi se bi "ga va qua trung" (khong co _session thi
        # khong gui duoc lenh hoi dau tien). De trong (mac dinh) thi KHONG gui gi ca -
        # an toan cho cac port dang bridge nguyen si 1 giao thuc byte-nhay-cam nhu
        # Modbus RTU (vd port modbusspy), vi gui them byte la se lam hong khung.
        self.on_connect_send = cfg.get("on_connect_send")
        # mbap_rtu_bridge: TUY CHON (mac dinh false = passthrough nguyen si
        # nhu cu). Bat = true khi client TCP noi vao port nay dung khung
        # Modbus-TCP (MBAP, vd node-red-contrib-modbus clienttype "tcp") con
        # thiet bi that tren serial la RTU (co CRC16) - add-on tu dich 2
        # chieu (xem ham mbap_frame_to_rtu/rtu_frame_to_mbap). Chi co y nghia
        # khi protocol=modbus_rtu VA output_mode=tcp; bo qua (khong dung) voi
        # protocol=raw hoac output_mode=pty.
        self.mbap_rtu_bridge = bool(cfg.get("mbap_rtu_bridge", False))
        self.modbus_response_timeout_s = float(cfg.get("modbus_response_timeout_s", 1.0))
        if not 0.1 <= self.modbus_response_timeout_s <= 30:
            raise ValueError("modbus_response_timeout_s must be between 0.1 and 30")
        if self.mbap_rtu_bridge and (self.protocol != "modbus_rtu" or self.output_mode != "tcp"):
            raise ValueError(
                f"[{self.name}] mbap_rtu_bridge=true chi dung duoc voi "
                "protocol=modbus_rtu VA output_mode=tcp"
            )
        if self.output_mode == "tcp" and not self.tcp_port:
            raise ValueError(f"[{self.name}] output_mode=tcp can co tcp_port")
        if self.output_mode == "pty" and not self.pty_symlink:
            raise ValueError(f"[{self.name}] output_mode=pty can co pty_symlink")
        self.match_mode = cfg.get("match_mode", "exact")
        self.fuzzy_threshold = cfg.get("fuzzy_threshold", 80)
        self.probe_timeout_s = cfg.get("probe_timeout_s", defaults["probe_timeout_s"])
        self.rescan_interval_s = cfg.get("rescan_interval_s", defaults["rescan_interval_s"])

        if self.protocol == "modbus_rtu":
            self.unit_id = cfg.get("unit_id")
            self.function_code = cfg.get("function_code")
            self.start_address = cfg.get("start_address", 0)
            self.quantity = cfg.get("quantity", 1)
            self.value = cfg.get("value")  # chi dung cho FC ghi (0x05/0x06)

        # send_command / send_command_2, expected_response / expected_response_2:
        # AP DUNG CHUNG cho ca 2 protocol (raw va modbus_rtu), TUY CHON - o
        # nao de trong thi bo qua, khong xet toi.
        #
        # Neu co it nhat 1 trong 2 expected_response(_2) duoc dien: chuyen
        # sang so khop LITERAL BYTE (giong het "raw") cho CA modbus_rtu -
        # KHONG con dung CRC/unit_id/function_code de xac dinh khop nua. Ly
        # do: (1) response cua cung 1 thiet bi co the KHAC NHAU giua cac thoi
        # diem (vd ban ngay Growatt hoi dia chi khac ban dem add-on tu hoi) -
        # dien 2 chu ky da quan sat duoc (1 cho ban ngay, 1 cho ban dem) an
        # toan hon suy doan CRC chung chung; (2) tranh nham thiet bi giua 2
        # bus RS-485 KHAC NHAU nhung tinh co cung unit_id (rat de xay ra vi
        # unit_id=1 la mac dinh pho bien) - so khop chu ky byte that cu the
        # hon nhieu so voi chi kiem CRC+unit_id.
        #
        # Neu CA HAI deu de trong VA protocol=modbus_rtu: dung lai logic CRC/
        # unit_id/function_code cu (backward-compat cho port da cau hinh tu
        # truoc, vd "modbusspy").
        #
        # Luong do (ca active lan passive) thu theo thu tu:
        #   1. Nghe passive (khong gui gi) -> so khop voi response 1 HOAC 2.
        #   2. Khong khop -> gui command 1 (rieng modbus_rtu: command 1 LUON
        #      la khung tu dong build tu unit_id/function_code/address/
        #      quantity/value nhu cu, "send_command" chi ap dung cho raw) ->
        #      so khop voi response 1 HOAC 2.
        #   3. Van khong khop VA co command_2 duoc dien -> gui command_2 ->
        #      so khop voi response 1 HOAC 2.
        self.send_command = cfg.get("send_command")
        self.send_command_2 = cfg.get("send_command_2")
        self.expected_response = cfg.get("expected_response")
        self.expected_response_2 = cfg.get("expected_response_2")

        self.device_path = None
        self.stop_event = threading.Event()
        # rescan_event: nguoi dung bam nut "Rescan ngay" tren Spy/Test UI ->
        # set event nay. Moi cho doi (Event.wait(timeout=...)) trong vong doi
        # port ao (ca luc dang bridge lan luc dang cho retry sau khi khong
        # tim thay thiet bi) deu dung event nay thay vi time.sleep() thuong -
        # Event.wait() tra ve NGAY khi event duoc set, khong can doi het
        # rescan_interval_s/ket noi hien tai bi ngat. Tu clear() lai sau moi
        # lan cho, tranh kich hoat rescan thua o vong lap ke tiep.
        self.rescan_event = threading.Event()
        # connected_clients: so client TCP dang ket noi vao port ao nay -
        # cap nhat boi serial_to_tcp_bridge(), doc boi Spy/Test web UI de
        # hien trang thai "dang ket noi" / "da ghim nhung chua co client".
        self.connected_clients = 0
        # Thong ke traffic (chi de xem cho vui tren Spy/Test web UI, khong
        # dung de tinh toan gi quan trong - khong can lock rieng, += tren
        # int la an toan du xai nho GIL cua Python).
        # "read" = huong serial -> client (doc tu thiet bi that, gui cho
        # client), "written" = huong client -> serial (client gui, ghi
        # xuong thiet bi that).
        self.bytes_read = 0
        self.bytes_written = 0
        self.messages_read = 0
        self.messages_written = 0
        # last_seen_state: "connected" | "missing" | None (chua ro) - dung de
        # CHI ghi 1 event log() khi TRANG THAI THAY DOI (vd tu "missing" sang
        # "connected"), tranh spam tab Log moi chu ky rescan_interval_s neu
        # thiet bi cu van chua tim thay lai.
        self.last_seen_state = None
        # taps: danh sach bytearray cua cac phien "nghe len" (Spy/Test UI)
        # dang mo tren port ao nay - reader cua bridge COPY du lieu doc duoc
        # vao tung tap, KHONG ai khac mo them fd toi /dev/ttyUSBx dang chay
        # (mo them fd = doi baud/termios chung + cuop byte cua bridge, xem
        # TROUBLESHOOTING_NOTES.md muc 25).
        self.taps: list = []
        self.taps_lock = threading.Lock()
        # serial_write_lock: nhieu client TCP cung ghi xuong 1 serial - moi
        # chunk recv() duoc ghi tron ven, khong bi xen byte cua client khac.
        self.serial_write_lock = threading.Lock()
        # active_serial: serial.Serial ma bridge (tcp/pty) dang mo - None khi
        # chua ghim/dang do. Dung cho che do "Dung chung" cua Get Response/
        # Communication: gui lenh test qua CHINH fd nay (khong mo fd thu 2).
        self.active_serial = None
        # test_hold: nguoi dung bam "Lay quyen port ao de test" - bridge dong,
        # nha cong, vong doi port ao DUNG CHO (khong do lai) toi khi "Tra
        # quyen" hoac qua TEST_HOLD_MAX_S. test_hold_device = cong dang nhuong
        # (port ao khac cung KHONG duoc do vao cong nay trong luc giu).
        self.test_hold = threading.Event()
        self.test_hold_device = None
        self.test_hold_since = 0.0

        if self.protocol == "modbus_rtu" and self.function_code in (5, 6, 15, 16):
            logging.warning(
                "[%s] function_code=%s la lenh GHI - MOI lan do port (khoi dong, "
                "mat ket noi, Rescan) add-on se GHI xuong thiet bi that (vd dong/"
                "cat relay)! Nen dung lenh DOC (FC 1/2/3/4) de do.",
                self.name, self.function_code,
            )

        # Kiem tra som ngay luc khoi tao: build thu tat ca lenh test + decode
        # thu tat ca response mong doi 1 lan - bat loi cau hinh (hex sai,
        # tham so modbus sai) ngay tu dau, kem ten port ro rang, thay vi de
        # loi am tham moi lan detect_port() chay. Bo qua neu port dang bi
        # disable - khong bat loi cau hinh dang lam do de dang tam thoi.
        if self.enabled:
            try:
                self.build_probe_frames()
                if self.expected_response:
                    decode_command(self.expected_response)
                if self.expected_response_2:
                    decode_command(self.expected_response_2)
            except Exception as exc:  # noqa: BLE001 - muon bien no thanh ValueError ro rang
                raise ValueError(
                    f"[{self.name}] cau hinh lenh test khong hop le: {exc}"
                ) from exc

    def feed_taps(self, data: bytes) -> None:
        if not self.taps:
            return
        with self.taps_lock:
            for buf in self.taps:
                buf.extend(data)

    def has_literal_response(self) -> bool:
        return bool(self.expected_response) or bool(self.expected_response_2)

    def build_probe_frames(self) -> list:
        """Tra ve danh sach 0, 1 hoac 2 khung lenh se thu gui theo thu tu."""
        frames = []
        if self.protocol == "modbus_rtu":
            frames.append(build_modbus_request(
                self.unit_id, self.function_code, self.start_address,
                self.quantity, self.value,
            ))
        elif self.send_command:
            frames.append(decode_command(self.send_command))
        if self.send_command_2:
            frames.append(decode_command(self.send_command_2))
        return frames

    def _check_literal(self, data: bytes) -> bool:
        if self.expected_response and response_matches(
            self.expected_response, data, self.match_mode, self.fuzzy_threshold
        ):
            return True
        if self.expected_response_2 and response_matches(
            self.expected_response_2, data, self.match_mode, self.fuzzy_threshold
        ):
            return True
        return False

    def check_probe_response(self, response: bytes) -> bool:
        if self.has_literal_response():
            return self._check_literal(response)
        if self.protocol == "modbus_rtu":
            return check_modbus_response(self.unit_id, self.function_code, response)
        return False

    def check_passive_buffer(self, buffer: bytes) -> bool:
        """Kiem tra traffic da nghe len duoc (khong tu gui gi) co dung khung
        cua thiet bi nay khong."""
        if self.has_literal_response():
            return self._check_literal(buffer)
        if self.protocol == "modbus_rtu":
            return passive_scan_modbus(buffer, self.unit_id, self.function_code)
        return False

    def describe_probe_frame(self, index: int) -> str:
        """Mo ta lenh test thu index (0-based) dang de doc de ghi log debug."""
        if self.protocol == "modbus_rtu" and index == 0:
            info = {
                "unitid": self.unit_id,
                "fc": self.function_code,
                "address": self.start_address,
                "quantity": self.quantity,
                "value": self.value,
            }
            return json.dumps(info, ensure_ascii=False)
        if index == 0:
            return repr(self.send_command)
        return repr(self.send_command_2)


def resolve_exclude_usb(exclude_raw: list) -> set:
    """Resolve moi entry trong 'exclude_usb' (str) thanh realpath - by-id/
    by-path la symlink, phai resolve moi lan goi (KHONG cache 1 lan luc
    khoi dong) vi symlink co the xuat hien tre hon add-on start, hoac
    /dev/ttyUSBx dang tro toi co the doi sau reboot/rut cam lai thiet bi.
    Giu ca gia tri raw (phong khi nguoi dung dien thang /dev/ttyUSBx) lan
    realpath da resolve trong cung 1 set de so khop chac chan."""
    resolved = set()
    for entry in exclude_raw or []:
        entry = (entry or "").strip()
        if not entry:
            continue
        resolved.add(entry)
        try:
            resolved.add(os.path.realpath(entry))
        except OSError:
            pass
    return resolved


def find_candidates(scan_glob: str, exclude_raw: list | None = None) -> list:
    patterns = [p.strip() for p in scan_glob.split(",") if p.strip()]
    found = []
    for pattern in patterns:
        found.extend(glob.glob(pattern))
    candidates = sorted(set(found))
    if not exclude_raw:
        return candidates
    excluded = resolve_exclude_usb(exclude_raw)
    if not excluded:
        return candidates
    return [
        c for c in candidates
        if c not in excluded and os.path.realpath(c) not in excluded
    ]


# errno thuong gap khi 1 tien trinh KHAC (hoac 1 port ao khac dang mo cung
# luc, dung truoc khi kip "giu" trong 'claimed') da giu doc quyen port do:
# EACCES=13 (permission/khong mo duoc), EBUSY=16, EAGAIN=11.
_BUSY_ERRNOS = {11, 13, 16}


def is_port_busy_error(exc: Exception) -> bool:
    errno_val = getattr(exc, "errno", None)
    if errno_val in _BUSY_ERRNOS:
        return True
    msg = str(exc).lower()
    return "could not exclusively lock" in msg or "busy" in msg or "permission denied" in msg


def _open_serial(path: str, baud: int, timeout_s: float, exclusive: bool) -> "serial.Serial":
    """
    Mo serial GIONG HET Node-RED (dtr/rts/cts/dsr: none - khong dung vao cac
    chan dieu khien). Ly do quan trong: pyserial mac dinh CHU DONG bat DTR
    luc mo port - day chinh la tin hieu kich hoat mach tu RESET Arduino (rat
    pho bien tren board Uno/Nano qua tu noi DTR->RESET, y het hieu ung mo
    Serial Monitor lam board tu khoi dong lai). Node-RED giu ket noi lien tuc
    tu luc khoi dong nen chi bi reset 1 lan roi chay on dinh; add-on nay dong
    /mo lai port moi chu ky do (~15s) nen se RESET ARDUINO MOI LAN neu khong
    chan dtr/rts truoc khi mo - Arduino khong kip boot xong da bi hoi tiep.
    Cach nay (tao Serial() roi set dtr/rts=False TRUOC khi goi .open()) la
    ky thuat pyserial chuan de tranh kich hoat auto-reset luc mo port.
    """
    ser = serial.Serial()
    ser.port = path
    ser.baudrate = baud
    ser.bytesize = 8
    ser.parity = "N"
    ser.stopbits = 1
    ser.timeout = timeout_s
    ser.exclusive = exclusive
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def _listen(ser: "serial.Serial", listen_s: float, stop_event=None) -> bytes:
    """Doc lien tuc trong listen_s giay, KHONG ghi/gui gi ca."""
    end_time = time.time() + listen_s
    buf = bytearray()
    while time.time() < end_time and not (stop_event and stop_event.is_set()):
        chunk = ser.read(512)
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def _detect_candidate(vport: VirtualPort, path: str, frames: list,
                       passive_listen_s: float) -> bool:
    """
    Luong thong nhat cho 1 candidate, ap dung chung cho ca modbus_rtu va raw:
      1. Thu mo doc quyen. Neu bi tien trinh khac giu (busy) -> log ro, thu mo
         KHONG doc quyen (chi de nghe len, khong bao gio gui/ghi gi len bus -
         tranh collision voi ai dang dung that).
      2. Neu mo doc quyen duoc -> nghe passive passive_listen_s giay truoc.
         Nghe thay khop -> xong, khong can gui gi ca.
      3. Khong nghe thay gi (vd bus dang im lang) -> lan luot gui TUNG lenh
         trong 'frames' (0, 1 hoac 2 lenh - "command 1" roi "command 2" neu
         co) tren CHINH ket noi doc quyen do, kiem phan hoi sau moi lenh -
         khop 1 trong 2 la xong ngay, khong can thu lenh con lai.
    Tra ve True neu candidate nay dung la thiet bi cua vport.
    """
    try:
        ser = _open_serial(path, vport.baud, 0.2, exclusive=True)
    except Exception as exc:  # noqa: BLE001
        if not is_port_busy_error(exc):
            logging.debug("[%s] khong mo duoc %s: %s", vport.name, path, exc)
            return False

        # KHONG "nghe len" port dang ban nua (bo tu 1.10.1): mo them 1 fd toi
        # cung /dev/ttyUSBx KHONG he thu dong - open() ghi de termios chung
        # (baud/parity cua port ao dang bridge, vd 115200 de len 9600 cua
        # relay), reset_input_buffer() xoa luon buffer cua ben kia, va read()
        # CUOP byte phan hoi cua tien trinh dang dung -> relay RS485 nhay/
        # mat phan hoi moi chu ky rescan cua port ao KHAC. Xem
        # TROUBLESHOOTING_NOTES.md muc 25.
        logging.info(
            "[%s] %s dang duoc dung boi tien trinh khac - bo qua (khong mo chung de tranh pha ket noi dang chay)",
            vport.name, path,
        )
        return False

    # Mo doc quyen thanh cong -> nghe passive truoc, chu dong gui sau neu can.
    try:
        ser.reset_input_buffer()
        buf = _listen(ser, passive_listen_s, vport.stop_event)
        logging.debug(
            "[%s] %s -> passive %d byte | %s", vport.name, path, len(buf), describe_bytes(buf)
        )

        try:
            matched = vport.check_passive_buffer(buf)
        except Exception as exc:  # noqa: BLE001
            logging.error(
                "[%s] LOI cau hinh khi quet passive tu %s: %s", vport.name, path, exc
            )
            raise

        if vport.stop_event.is_set():
            return False
        if matched:
            logging.info(
                "[%s] KHOP voi %s (passive, %d byte, khong can gui gi)",
                vport.name, path, len(buf),
            )
            return True

        # Khong nghe thay gi -> co the bus dang im lang -> lan luot gui tung
        # lenh trong 'frames' tren CHINH ket noi doc quyen dang co san nay.
        # Doi lai timeout doc theo dung probe_timeout_s cua port ao (luc mo
        # ket noi dang de tam 0.2s chi de phuc vu vong doc passive o tren).
        ser.timeout = vport.probe_timeout_s
        for idx, frame in enumerate(frames):
            if vport.stop_event.is_set():
                return False
            command_desc = vport.describe_probe_frame(idx)
            ser.reset_input_buffer()
            logging.debug(
                "[%s] %s <- GUI (lenh %d/%d): %s | %s (%d byte)",
                vport.name, path, idx + 1, len(frames), command_desc,
                describe_bytes(frame), len(frame),
            )
            ser.write(frame)
            time.sleep(0.05)
            resp = ser.read(512)
            logging.debug(
                "[%s] %s -> active response (lenh %d/%d): %s (%d byte)",
                vport.name, path, idx + 1, len(frames), describe_bytes(resp), len(resp),
            )

            try:
                matched = vport.check_probe_response(resp)
            except Exception as exc:  # noqa: BLE001
                logging.error(
                    "[%s] LOI cau hinh khi so khop response tu %s: %s", vport.name, path, exc
                )
                raise

            if matched:
                logging.info(
                    "[%s] KHOP voi %s (active, lenh %d/%d) | send_command: %s | response: %s",
                    vport.name, path, idx + 1, len(frames), command_desc, describe_bytes(resp),
                )
                return True

        logging.debug("[%s] %s khong khop (da thu het %d lenh)", vport.name, path, len(frames))
        return False
    finally:
        ser.close()


def detect_port(vport: VirtualPort, candidates: list, passive_listen_s: float) -> str | None:
    try:
        frames = vport.build_probe_frames()
    except Exception as exc:  # noqa: BLE001 - co y bat rong, khong duoc crash
        logging.error("[%s] LOI cau hinh khi build lenh test: %s", vport.name, exc)
        log_event("error", vport.name, f"Lỗi cấu hình khi build lệnh test: {exc}")
        return None

    for path in candidates:
        if vport.stop_event.is_set():
            return None
        try:
            if _detect_candidate(vport, path, frames, passive_listen_s):
                return path
        except Exception:  # noqa: BLE001 - loi cau hinh khi so khop -> dung han vong quet nay
            return None
    return None


# ---------------------------------------------------------------------------
# TCP <-> serial bridge - day la thu Node-RED/ModbusSpy se ket noi vao,
# khong bao gio ket noi thang vao /dev/ttyUSBx.
# ---------------------------------------------------------------------------

def serial_to_tcp_bridge(vport: VirtualPort) -> None:
    """Bridge TCP/serial. Raw mode broadcasts unchanged bytes to clients.

    Modbus MBAP mode serializes complete request/response transactions and
    sends each response only to the originating client with its own TID.
    A timeout closes the bridge to discard queued work and late replies.
    """
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", vport.tcp_port))
        server.listen(5)
    except OSError as exc:
        # vd "Address already in use" (2 port ao trung tcp_port, hoac socket
        # cu chua kip giai phong) - dong socket ngay, de vong ngoai rescan,
        # KHONG duoc de loi nay thoat ra ngoai ham.
        logging.error(
            "[%s] khong the mo TCP port %d: %s", vport.name, vport.tcp_port, exc
        )
        log_event("error", vport.name, f"Không mở được TCP port {vport.tcp_port}: {exc}")
        server.close()
        return

    try:
        ser = _open_serial(vport.device_path, vport.baud, 0.2, exclusive=True)
    except (serial.SerialException, OSError) as exc:
        logging.error(
            "[%s] mat ket noi toi %s: %s", vport.name, vport.device_path, exc
        )
        log_event("error", vport.name, f"Không mở được {vport.device_path}: {exc}")
        server.close()
        return

    vport.active_serial = ser
    logging.info(
        "[%s] TCP server dang lang nghe port %d (ho tro nhieu client), cau noi toi %s",
        vport.name, vport.tcp_port, vport.device_path,
    )

    clients_lock = threading.Lock()
    clients: set = set()
    serial_error = threading.Event()

    # One RTU transaction at a time, including the response wait. A TCP
    # transaction ID belongs to its requesting connection, never to the bus.
    rtu_read_buffer = bytearray()
    mbap_state_lock = threading.Lock()
    pending = None
    silent_interval = max(0.004, 38.5 / vport.baud) if vport.mbap_rtu_bridge else 0.0

    def transact_rtu(frame: bytes, client_addr, transaction_id: bytes) -> bytes | None:
        nonlocal pending
        with vport.serial_write_lock:
            if serial_error.is_set() or vport.stop_event.is_set() or vport.rescan_event.is_set():
                raise OSError("Bridge is closing")
            transaction = {"request": frame, "response": None, "done": threading.Event()}
            with mbap_state_lock:
                rtu_read_buffer.clear()
                pending = transaction
            try:
                logging.debug("[%s] RS485 TX MBAP client=%s tid=%s %s",
                              vport.name, client_addr, transaction_id.hex(), describe_bytes(frame))
                ser.write(frame)
                ser.flush()
                if frame[0] == 0:  # RTU broadcast has no response.
                    time.sleep(silent_interval)
                    return None
                deadline = time.monotonic() + vport.modbus_response_timeout_s
                while not transaction["done"].wait(0.02):
                    if serial_error.is_set() or vport.stop_event.is_set() or vport.rescan_event.is_set():
                        raise OSError("Bridge interrupted")
                    if time.monotonic() >= deadline:
                        # RTU carries no transaction ID. After a timeout we
                        # cannot safely attribute a late reply to a new request.
                        # Close this bridge; never retry a control command here.
                        serial_error.set()
                        log_event("error", vport.name, "Modbus timeout: đóng TCP để tránh ghép phản hồi trễ vào lệnh mới")
                        raise OSError("Modbus response timeout; reconnect required")
                time.sleep(silent_interval)
                return transaction["response"]
            finally:
                with mbap_state_lock:
                    pending = None
                    rtu_read_buffer.clear()

    def broadcast(data: bytes) -> None:
        with clients_lock:
            dead = []
            for c in clients:
                try:
                    c.sendall(data)
                except OSError:
                    dead.append(c)
            for c in dead:
                clients.discard(c)
                try:
                    c.close()
                except OSError:
                    pass

    def reader_loop() -> None:
        while not vport.stop_event.is_set() and not serial_error.is_set() and not vport.rescan_event.is_set():
            try:
                # Modbus is binary: forward available bytes immediately.
                # read(512) waits until the 200 ms timeout for short replies.
                # Keep raw/text batching for existing Arduino consumers.
                if vport.protocol == "modbus_rtu":
                    data = ser.read(1)
                    if data:
                        available = min(511, ser.in_waiting)
                        if available:
                            data += ser.read(available)
                else:
                    data = ser.read(512)
                if not data:
                    continue
                vport.bytes_read += len(data)
                vport.messages_read += 1
                vport.feed_taps(data)
                if not vport.mbap_rtu_bridge:
                    broadcast(data)
                    continue
                with mbap_state_lock:
                    if pending is None or pending["done"].is_set():
                        continue  # Unsolicited/late RTU bytes are not MBAP.
                    rtu_read_buffer.extend(data)
                    request = pending["request"]
                    while len(rtu_read_buffer) >= 2:
                        # Resynchronize on the expected unit/function. Keep
                        # partial headers; never leak raw RTU to TCP clients.
                        if (rtu_read_buffer[0] != request[0] or
                                rtu_read_buffer[1] not in (request[1], request[1] | 0x80)):
                            del rtu_read_buffer[0]
                            continue
                        need = rtu_response_expected_len(rtu_read_buffer)
                        if need is None or len(rtu_read_buffer) < need:
                            break
                        frame = bytes(rtu_read_buffer[:need])
                        if modbus_crc16(frame[:-2]) != frame[-2:]:
                            del rtu_read_buffer[0]
                            continue
                        del rtu_read_buffer[:need]
                        fc = frame[1]
                        if fc in (5, 6, 15, 16) and frame[:6] != request[:6]:
                            continue  # Acknowledgement for another write.
                        if fc in (1, 2, 3, 4):
                            quantity = int.from_bytes(request[4:6], "big")
                            expected = (quantity + 7) // 8 if fc in (1, 2) else quantity * 2
                            if frame[2] != expected:
                                continue
                        pending["response"] = frame
                        pending["done"].set()
                        break
            except (serial.SerialException, OSError) as exc:
                logging.error(
                    "[%s] mat ket noi toi %s: %s", vport.name, vport.device_path, exc
                )
                log_event("disconnect", vport.name, f"Mất kết nối USB tới {vport.device_path}: {exc}")
                serial_error.set()
                break

    reader_thread = threading.Thread(target=reader_loop, daemon=True)
    reader_thread.start()

    def handle_client(client: socket.socket, addr) -> None:
        logging.info("[%s] client %s da ket noi", vport.name, addr)
        log_event("client", vport.name, f"Client TCP {addr} đã kết nối")

        if vport.on_connect_send:
            try:
                greeting = decode_command(vport.on_connect_send)
                client.sendall(greeting)
                logging.debug(
                    "[%s] da gui on_connect_send toi client %s: %s",
                    vport.name, addr, describe_bytes(greeting),
                )
            except OSError as exc:
                logging.warning(
                    "[%s] khong gui duoc on_connect_send toi client %s: %s",
                    vport.name, addr, exc,
                )

        with clients_lock:
            clients.add(client)
            vport.connected_clients = len(clients)

        # mbap_write_buffer: RIENG cho client nay (moi client 1 thread/1
        # buffer) - tich luy byte MBAP nhan tu client toi khi ghep duoc it
        # nhat 1 khung hoan chinh (xem mbap_frame_to_rtu), chi dung khi
        # vport.mbap_rtu_bridge=True.
        mbap_write_buffer = bytearray()

        try:
            client.settimeout(1.0)
            while not vport.stop_event.is_set() and not serial_error.is_set() and not vport.rescan_event.is_set():
                try:
                    data = client.recv(512)
                except socket.timeout:
                    continue
                if not data:
                    break
                vport.bytes_written += len(data)
                vport.messages_written += 1
                try:
                    if not vport.mbap_rtu_bridge:
                        with vport.serial_write_lock:
                            if vport.protocol == "modbus_rtu":
                                logging.debug("[%s] RS485 TX passthrough client=%s %s",
                                              vport.name, addr, describe_bytes(data))
                            ser.write(data)
                        continue
                    mbap_write_buffer.extend(data)
                    while True:
                        parsed = mbap_frame_to_rtu(bytes(mbap_write_buffer))
                        if parsed is None:
                            if len(mbap_write_buffer) > 4096:
                                # khong the nao 1 lenh Modbus dai toi muc nay -
                                # chac chan da lech khung, xoa sach de tu hoi
                                # phuc tu khung MBAP tiep theo, tranh ket dinh
                                # vinh vien.
                                logging.warning(
                                    "[%s] mbap_rtu_bridge: buffer client %s "
                                    "qua dai, khong ghep duoc khung MBAP - "
                                    "xoa lam sach",
                                    vport.name, addr,
                                )
                                mbap_write_buffer.clear()
                            break
                        transaction_id, rtu_frame, consumed = parsed
                        del mbap_write_buffer[:consumed]
                        if len(rtu_frame) < 8 or rtu_frame[1] not in (1, 2, 3, 4, 5, 6, 15, 16):
                            # Supported response parsers only; reject locally.
                            body = bytes((rtu_frame[0], rtu_frame[1] | 0x80, 1))
                            client.sendall(rtu_frame_to_mbap(body + modbus_crc16(body), transaction_id))
                            continue
                        response = transact_rtu(rtu_frame, addr, transaction_id)
                        if response is not None:
                            try:
                                client.sendall(rtu_frame_to_mbap(response, transaction_id))
                            except OSError:
                                return  # Disconnected client must not stop other clients.
                except (serial.SerialException, OSError) as exc:
                    logging.error(
                        "[%s] mat ket noi toi %s: %s", vport.name, vport.device_path, exc
                    )
                    log_event("disconnect", vport.name, f"Mất kết nối USB (khi ghi) tới {vport.device_path}: {exc}")
                    serial_error.set()
                    break
        except (OSError, ValueError) as exc:
            logging.debug("[%s] TCP client closed: %s", vport.name, exc)
        finally:
            with clients_lock:
                clients.discard(client)
                vport.connected_clients = len(clients)
            client.close()
            logging.info("[%s] client %s ngat ket noi", vport.name, addr)
            log_event("client", vport.name, f"Client TCP {addr} đã ngắt kết nối")

    try:
        while not vport.stop_event.is_set() and not serial_error.is_set() and not vport.rescan_event.is_set():
            server.settimeout(1.0)
            try:
                client, addr = server.accept()
            except socket.timeout:
                continue
            threading.Thread(
                target=handle_client, args=(client, addr), daemon=True
            ).start()
    finally:
        serial_error.set()
        with clients_lock:
            for c in list(clients):
                try:
                    c.close()
                except OSError:
                    pass
            clients.clear()
            vport.connected_clients = 0
        vport.active_serial = None
        try:
            ser.close()
        except OSError:
            pass
        server.close()

    if vport.rescan_event.is_set():
        logging.info(
            "[%s] Rescan thu cong duoc yeu cau, dong TCP server de do lai ngay",
            vport.name,
        )
    elif serial_error.is_set():
        logging.warning(
            "[%s] mat ket noi serial that, dong TCP server de rescan lai tu dau",
            vport.name,
        )
        # bao hieu vong ngoai (run_virtual_port_lifecycle) can rescan lai tu dau -
        # CHI khi thiet bi that su co loi/mat, khong phai khi 1 client thuong
        # ngat ket noi (client ngat ket noi duoc xu ly rieng trong handle_client,
        # khong lam sap ca bridge).


def serial_to_pty_bridge(vport: VirtualPort) -> None:
    """
    Tao 1 PTY (pseudo-terminal) va symlink no vao vport.pty_symlink, cau noi 2
    chieu voi serial that. CANH BAO: PTY tao ra o day chi chac chan nhin thay
    duoc tu BEN TRONG cung container cua add-on nay - viec add-on/container
    KHAC co thay duoc symlink nay hay khong phu thuoc vao Supervisor co chia
    se chung /dev/pts giua cac container hay khong (chua kiem chung). Neu
    khong thay duoc tu container can dung, dung output_mode: tcp + chay socat
    ben phia container do de tu tao PTY tai cho no can (xem README).
    """
    master_fd, slave_fd = pty_module.openpty()
    slave_path = os.ttyname(slave_fd)

    symlink_path = vport.pty_symlink
    os.makedirs(os.path.dirname(symlink_path), exist_ok=True)
    try:
        if os.path.islink(symlink_path):
            os.remove(symlink_path)
        elif os.path.exists(symlink_path):
            raise ValueError("PTY path already exists and is not a symlink")
    except FileNotFoundError:
        pass
    os.symlink(slave_path, symlink_path)
    logging.info(
        "[%s] PTY %s (symlink %s) da tao, cau noi toi %s",
        vport.name, slave_path, symlink_path, vport.device_path,
    )

    try:
        ser = _open_serial(vport.device_path, vport.baud, 0.1, exclusive=True)
    except (serial.SerialException, OSError) as exc:
        logging.error("[%s] mat ket noi toi %s: %s", vport.name, vport.device_path, exc)
        log_event("error", vport.name, f"Không mở được {vport.device_path}: {exc}")
        os.close(master_fd)
        os.close(slave_fd)
        return

    vport.active_serial = ser
    stop_pump = threading.Event()

    def pump_serial_to_pty():
        while not stop_pump.is_set() and not vport.stop_event.is_set() and not vport.rescan_event.is_set():
            try:
                data = ser.read(512)
                if data:
                    vport.bytes_read += len(data)
                    vport.messages_read += 1
                    vport.feed_taps(data)
                    os.write(master_fd, data)
            except (serial.SerialException, OSError) as exc:
                log_event("disconnect", vport.name, f"Mất kết nối USB (pty) tới {vport.device_path}: {exc}")
                break
        stop_pump.set()

    reader_thread = threading.Thread(target=pump_serial_to_pty, daemon=True)
    reader_thread.start()

    # Bounded select allows an idle PTY to release its UART on UI edits.
    try:
        while not vport.stop_event.is_set() and not stop_pump.is_set() and not vport.rescan_event.is_set():
            try:
                ready, _, _ = select.select([master_fd], [], [], 0.2)
                if not ready:
                    continue
                data = os.read(master_fd, 512)
            except OSError:
                break
            if not data:
                break
            vport.bytes_written += len(data)
            vport.messages_written += 1
            with vport.serial_write_lock:
                ser.write(data)
    finally:
        stop_pump.set()
        vport.active_serial = None
        ser.close()
        os.close(master_fd)
        os.close(slave_fd)
        logging.info("[%s] PTY bridge dong lai", vport.name)


# Tu tra quyen neu nguoi dung "Lay quyen port ao de test" roi quen bam "Tra
# quyen" - tranh relay/thiet bi that bi bo roi vinh vien.
TEST_HOLD_MAX_S = 30 * 60


def _held_devices() -> dict:
    """{device_path: ten port ao} cua cac cong dang nhuong cho test."""
    with _vport_registry_lock:
        entries = list(_vport_registry.values())
    result = {}
    for entry in entries:
        vport = entry.get("vport")
        if vport is not None and vport.test_hold.is_set() and vport.test_hold_device:
            result[vport.test_hold_device] = vport.name
    return result


def _wait_test_hold(vport: VirtualPort) -> None:
    """Port ao dang nhuong cong cho test: dung cho (khong do, khong mo cong)
    toi khi nguoi dung Tra quyen hoac qua TEST_HOLD_MAX_S."""
    logging.info("[%s] tam nhuong %s cho test - dung do lai", vport.name, vport.test_hold_device)
    while vport.test_hold.is_set() and not vport.stop_event.is_set():
        if time.time() - vport.test_hold_since > TEST_HOLD_MAX_S:
            logging.warning("[%s] giu quyen test qua %ds - tu tra quyen", vport.name, TEST_HOLD_MAX_S)
            log_event("warning", vport.name, f"Giữ quyền test quá {TEST_HOLD_MAX_S // 60} phút - tự trả quyền cho port ảo")
            _release_test_hold(vport)
            break
        vport.rescan_event.wait(timeout=1.0)
        vport.rescan_event.clear()
    logging.info("[%s] da tra quyen test - do lai thiet bi", vport.name)


def _release_test_hold(vport: VirtualPort) -> None:
    vport.test_hold.clear()
    vport.test_hold_device = None
    vport.rescan_event.set()


def _take_test_hold(name: str) -> dict:
    """"Lay quyen port ao de test": dong bridge + nha cong that de tab Get
    Response/Communication mo doc quyen binh thuong. Khong bao gio raise."""
    with _vport_registry_lock:
        entry = _vport_registry.get(name)
    vport = entry.get("vport") if entry else None
    if vport is None:
        return {"ok": False, "error": f"Không tìm thấy port ảo đang bật tên {name!r}"}
    if vport.test_hold.is_set():
        return {"ok": True, "device": vport.test_hold_device}
    device = vport.device_path
    if not device:
        return {"ok": False, "error": "Port ảo chưa ghim thiết bị nào - không có gì để lấy quyền"}
    vport.test_hold_device = device
    vport.test_hold_since = time.time()
    vport.test_hold.set()
    vport.rescan_event.set()  # bridge thoat vong lap, dong serial
    deadline = time.time() + 5.0
    while vport.device_path is not None and time.time() < deadline:
        time.sleep(0.1)
    log_event("info", name, f"Lấy quyền test: tạm ngắt port ảo, nhường {device} cho Get Response/Communication")
    result = {"ok": True, "device": device}
    if vport.device_path is not None:
        result["warning"] = "Bridge chưa kịp đóng (pty chờ byte mới) - đợi vài giây rồi thử lại lệnh test."
    return result


def _give_back_test_hold(name: str) -> dict:
    with _vport_registry_lock:
        entry = _vport_registry.get(name)
    vport = entry.get("vport") if entry else None
    if vport is None:
        return {"ok": False, "error": f"Không tìm thấy port ảo đang bật tên {name!r}"}
    if vport.test_hold.is_set():
        _release_test_hold(vport)
        log_event("info", name, "Trả quyền test: port ảo dò/ghim lại thiết bị")
    return {"ok": True}


def run_virtual_port_lifecycle(vport: VirtualPort, scan_glob: str, passive_listen_s: float,
                                claimed_lock: threading.Lock, claimed: dict,
                                exclude_usb: list | None = None) -> None:
    """
    Vong doi cua 1 port ao, chay rieng 1 thread. QUAN TRONG: ham nay KHONG
    BAO GIO duoc phep de loi thoat ra ngoai (thread chet vinh vien = port ao
    do "chet luon", khong bao gio dò lai nua, cac port khac van chay nhung
    port nay thi mat han) - moi loi bat ngo deu phai bi bat, log lai, roi cho
    ROI DO LAI, dung y cau "port nao mat ket noi thi cho de do lai, loi gi
    thi xuat ra logs, khong duoc crash ngang".
    """
    path = None
    while not vport.stop_event.is_set():
        try:
            if vport.test_hold.is_set():
                _wait_test_hold(vport)
            if vport.stop_event.is_set():
                break

            candidates = find_candidates(scan_glob, exclude_usb)
            held = _held_devices()
            with claimed_lock:
                available = [c for c in candidates if c not in claimed and c not in held]

            # Uu tien thu lai DUNG port da khop lan gan nhat (ke ca sau khi
            # restart container) truoc khi quet het cac ung vien con lai tu dau.
            # Port ma port ao KHAC da ghim lan truoc -> day xuong CUOI danh
            # sach: luc khoi dong moi port ao deu thu port cu cua MINH truoc,
            # tranh gui lenh do cua port ao nay (vd text Arduino 115200) vao
            # bus RS485 relay truoc khi port ao relay kip ghim lai.
            cache = load_last_matched()
            others = {p for n, p in cache.items() if n != vport.name}
            available = [c for c in available if c not in others] +                 [c for c in available if c in others]
            last_matched = cache.get(vport.name)
            if last_matched and last_matched in available:
                available = [last_matched] + [c for c in available if c != last_matched]
                logging.debug(
                    "[%s] uu tien thu lai port da ghim lan truoc: %s",
                    vport.name, last_matched,
                )

            # Serialize detection + claim so two matching rules cannot claim
            # the same UART between detect returning and registry assignment.
            while not vport.stop_event.is_set():
                if _detection_lock.acquire(timeout=0.2):
                    break
            else:
                break
            try:
                held = _held_devices()
                with claimed_lock:
                    available = [c for c in available if c not in claimed and c not in held]
                path = detect_port(vport, available, passive_listen_s)
                if vport.stop_event.is_set():
                    path = None
                    break
                if path is not None:
                    with claimed_lock:
                        claimed[path] = vport.name
            finally:
                _detection_lock.release()
            if path is None:
                logging.warning(
                    "[%s] khong tim thay thiet bi khop trong %s, thu lai sau %ds",
                    vport.name, available, vport.rescan_interval_s,
                )
                # Chi ghi event khi TRANG THAI THAY DOI (vua tu "connected"
                # sang "missing") - tranh spam tab Log moi chu ky rescan neu
                # thiet bi van chua tim thay lai.
                if vport.last_seen_state != "missing":
                    log_event("warning", vport.name, f"Chưa tìm thấy thiết bị khớp trong {available}")
                    vport.last_seen_state = "missing"
                # rescan_event.wait(timeout=...) giong het time.sleep() neu
                # KHONG co ai set event - nhung tra ve NGAY neu nguoi dung
                # bam "Rescan ngay" tren Spy/Test UI trong luc dang cho.
                vport.rescan_event.wait(timeout=vport.rescan_interval_s)
                vport.rescan_event.clear()
                continue

            vport.device_path = path
            save_last_matched(vport.name, path)
            log_event("connect", vport.name, f"Đã ghim thiết bị {path}")
            vport.last_seen_state = "connected"

            if vport.output_mode == "pty":
                serial_to_pty_bridge(vport)  # block cho toi khi mat ket noi serial that HOAC rescan_event
            else:
                serial_to_tcp_bridge(vport)  # block cho toi khi mat ket noi serial that HOAC rescan_event

            if vport.rescan_event.is_set():
                logging.info(
                    "[%s] Rescan thu cong duoc yeu cau (Spy/Test UI) - dong ket noi %s, do lai ngay",
                    vport.name, path,
                )
                log_event("info", vport.name, f"Rescan thủ công yêu cầu - đóng kết nối {path}, dò lại ngay")
            else:
                logging.warning(
                    "[%s] mat ket noi toi %s, rescan lai sau %ds",
                    vport.name, path, vport.rescan_interval_s,
                )
        except Exception as exc:  # noqa: BLE001 - bat rong CO Y: khong duoc de thread nay chet
            logging.exception(
                "[%s] LOI KHONG LUONG TRUOC, se cho %ds roi dò lai (port khac khong bi anh huong)",
                vport.name, vport.rescan_interval_s,
            )
            log_event("error", vport.name, f"Lỗi không lường trước: {exc}")
        finally:
            if path is not None:
                with claimed_lock:
                    if claimed.get(path) == vport.name:
                        claimed.pop(path, None)
            path = None
            vport.device_path = None

        vport.rescan_event.wait(timeout=vport.rescan_interval_s)
        vport.rescan_event.clear()


# ---------------------------------------------------------------------------
# MQTT Discovery - tao entity THAT tren Home Assistant cho tung port ao (1
# sensor "thiet bi da ghim" + 1 binary_sensor "dang co client ket noi"),
# thay vi chi xem duoc qua Spy/Test web UI. Dung "services: mqtt:want" trong
# config.yaml de Supervisor TU CAP thong tin broker qua bien moi truong
# (MQTT_HOST/MQTT_PORT/MQTT_USERNAME/MQTT_PASSWORD) - khong can nguoi dung tu
# nhap broker. Neu khong co broker MQTT (Supervisor chua cai Mosquitto, hoac
# addon chua duoc cap quyen): CHI log canh bao 1 lan, add-on van chay binh
# thuong (TCP/PTY bridge khong phu thuoc gi vao MQTT).
#
# unique_id LUON dung vport.name (KHONG dung friendly_name) - xem
# TROUBLESHOOTING_NOTES.md muc 17b: name la ID CO DINH, doi ten hien thi thi
# sua friendly_name, KHONG duoc doi name sau khi da chay that (doi name =
# tao entity MQTT hoan toan moi, entity cu thanh "mo coi" tren HA).
# ---------------------------------------------------------------------------

MQTT_DISCOVERY_PREFIX = "homeassistant"
MQTT_BASE_TOPIC = "usbmgr"
MQTT_DEVICE_ID = "usbmgr_addon"
MQTT_AVAILABILITY_TOPIC = f"{MQTT_BASE_TOPIC}/status"

# _mqtt_state: doc boi Spy/Test web UI (GET /api/mqtt_status) de hien badge
# trang thai MQTT ngay tren trang, khong can vao tab Log moi biet. "host" la
# None khi CHUA TUNG thu ket noi (khac voi da thu nhung dut) - dung phan
# biet 2 truong hop "chua cau hinh" vs "co cau hinh nhung mat ket noi".
_mqtt_state = {"client": None, "host": None, "port": None}


def mqtt_status_snapshot() -> dict:
    client = _mqtt_state.get("client")
    # client.is_connected() phan anh dung trang thai socket THUC TE ngay luc
    # goi (khong phai chi nho lai luc connect() thanh cong 1 lan) - phat hien
    # duoc ca truong hop broker rot ket noi sau nay.
    connected = bool(client is not None and client.is_connected())
    return {
        "connected": connected,
        "configured": _mqtt_state.get("host") is not None,
        "host": _mqtt_state.get("host"),
        "port": _mqtt_state.get("port"),
    }


def _mqtt_device_info() -> dict:
    """1 'device' HA duy nhat dung chung cho MOI entity cua add-on nay - tat
    ca port ao gom vao 1 thiet bi 'USB Manager Distribution' tren HA, giong
    cach Zigbee2MQTT gom nhieu entity duoi 1 device (bridge)."""
    return {
        "identifiers": [MQTT_DEVICE_ID],
        "name": "USB Manager Distribution",
        "manufacturer": "Local Add-on",
        "model": "usb-manager-distribution",
    }


def mqtt_setup(options: dict | None = None):
    """Ket noi MQTT broker. Uu tien field cau hinh THU CONG trong options
    (mqtt_host/mqtt_port/mqtt_username/mqtt_password) - cach chac chan hoat
    dong voi MOI loai add-on. Neu khong dien, fallback sang bien moi truong
    Supervisor cap qua 'services: mqtt:want' (MQTT_HOST/...) - co the hoat
    dong hoac khong tuy add-on la local hay tu Store, khong dam bao chac
    chan. Tra ve client (da connect + loop_start) hoac None neu khong co
    broker/thieu thu vien - KHONG BAO GIO raise, goi tu main() phai an toan
    du MQTT khong san sang."""
    options = options or {}
    host = options.get("mqtt_host") or os.environ.get("MQTT_HOST")
    if not host:
        logging.warning(
            "Khong co MQTT broker (chua dien mqtt_host trong Configuration, "
            "VA Supervisor cung chua cap MQTT_HOST qua services: mqtt:want) "
            "- BO QUA tao entity HA qua MQTT, cac chuc nang khac cua add-on "
            "van chay binh thuong."
        )
        return None
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        logging.error(
            "Thieu thu vien paho-mqtt (kiem tra requirements.txt + rebuild "
            "image) - BO QUA tao entity HA qua MQTT."
        )
        return None

    port = int(options.get("mqtt_port") or os.environ.get("MQTT_PORT", 1883))
    username = options.get("mqtt_username") or os.environ.get("MQTT_USERNAME")
    password = options.get("mqtt_password") or os.environ.get("MQTT_PASSWORD")

    # paho-mqtt >=2.0 doi API: Client() khong truyen callback_api_version se
    # in DeprecationWarning (van chay dung, chi la canh bao). Add-on nay
    # khong dung callback nao (on_connect/on_message) nen VERSION2 la lua
    # chon an toan. Fallback ve cach goi cu (khong tham so nay) neu
    # requirements.txt lo cai ban paho-mqtt <2.0 (khong co CallbackAPIVersion).
    try:
        client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id="usb_manager_distribution",
        )
    except AttributeError:
        client = mqtt.Client(client_id="usb_manager_distribution")
    except Exception as exc:  # noqa: BLE001
        logging.error("Khong tao duoc MQTT client: %s", exc)
        return None
    if username:
        client.username_pw_set(username, password)
    client.will_set(MQTT_AVAILABILITY_TOPIC, payload="offline", retain=True)
    try:
        client.connect(host, port, keepalive=60)
    except (OSError, Exception) as exc:  # noqa: BLE001
        logging.error("Khong ket noi duoc MQTT broker %s:%d: %s", host, port, exc)
        log_event("error", None, f"Không kết nối được MQTT broker {host}:{port}: {exc}")
        _mqtt_state["host"] = host  # da CO CAU HINH, chi la lan nay ket noi that bai
        _mqtt_state["port"] = port
        return None
    client.loop_start()
    client.publish(MQTT_AVAILABILITY_TOPIC, payload="online", retain=True)
    logging.info(
        "Da ket noi MQTT broker %s:%d - se tao entity HA cho tung port ao", host, port
    )
    _mqtt_state["client"] = client
    _mqtt_state["host"] = host
    _mqtt_state["port"] = port
    return client


def mqtt_publish_discovery(client, name: str, friendly_name: str, enabled: bool) -> None:
    """Publish 2 config topic Discovery (sensor + binary_sensor) cho 1 port
    ao - goi 1 LAN luc khoi dong cho MOI port ao khai trong config, ke ca
    dang enabled=false (de nguoi dung thay ca port dang tat tren HA, giong
    bang trang thai cua Spy/Test web UI)."""
    if not NAME_RE.match(name):
        logging.warning(
            "[mqtt] bo qua publish discovery cho ten port khong hop le: %r", name
        )
        return
    device = _mqtt_device_info()

    # object_id: ep entity_id CO DINH (vd sensor.usbmgr_arduino_1_device_path_
    # usb_manager_distribution), KHONG de HA tu "slugify" tu "name" hien thi -
    # "name" hien thi dung friendly_name (co the tieng Viet co dau), tu
    # slugify se ra entity_id xau/kho doan (dau bi cat lung tung). object_id
    # luon dung vport.name (ASCII, khop NAME_RE) nen entity_id on dinh du doi
    # friendly_name. Hau to "_usb_manager_distribution" o CUOI CUNG de de
    # nhan ra ngay entity nao thuoc add-on nay khi tim trong danh sach entity
    # chung cua ca he thong HA (nhieu add-on khac cung co the dung tien to
    # "usbmgr" hoac ten port trung nhau).
    device_path_id = f"usbmgr_{name}_device_path_usb_manager_distribution"
    connected_id = f"usbmgr_{name}_connected_usb_manager_distribution"

    sensor_cfg = {
        "name": f"{friendly_name} - Thiết bị",
        "unique_id": device_path_id,
        "object_id": device_path_id,
        "state_topic": f"{MQTT_BASE_TOPIC}/{name}/device_path",
        "availability_topic": MQTT_AVAILABILITY_TOPIC,
        "icon": "mdi:usb-port",
        "device": device,
    }
    binary_cfg = {
        "name": f"{friendly_name} - Kết nối",
        "unique_id": connected_id,
        "object_id": connected_id,
        "state_topic": f"{MQTT_BASE_TOPIC}/{name}/connected",
        "payload_on": "ON",
        "payload_off": "OFF",
        "device_class": "connectivity",
        "availability_topic": MQTT_AVAILABILITY_TOPIC,
        "device": device,
    }
    client.publish(
        f"{MQTT_DISCOVERY_PREFIX}/sensor/{device_path_id}/config",
        json.dumps(sensor_cfg, ensure_ascii=False), retain=True,
    )
    client.publish(
        f"{MQTT_DISCOVERY_PREFIX}/binary_sensor/{connected_id}/config",
        json.dumps(binary_cfg, ensure_ascii=False), retain=True,
    )

    if not enabled:
        client.publish(f"{MQTT_BASE_TOPIC}/{name}/device_path", "Đã tắt (disabled)", retain=True)
        client.publish(f"{MQTT_BASE_TOPIC}/{name}/connected", "OFF", retain=True)


def mqtt_publish_state(client, name: str, device_path: str | None, connected: bool) -> None:
    client.publish(
        f"{MQTT_BASE_TOPIC}/{name}/device_path",
        device_path or "Chưa tìm thấy thiết bị", retain=True,
    )
    client.publish(f"{MQTT_BASE_TOPIC}/{name}/connected", "ON" if connected else "OFF", retain=True)


def mqtt_state_loop(client, poll_interval_s: float = 3.0, stop_event=None) -> None:
    """Vong lap nen: so sanh trang thai tung port ao (device_path + co client
    dang ket noi khong) voi lan truoc, CHI publish khi co thay doi thuc su -
    tranh spam MQTT/History cua HA vo ich. Bat rong CO Y (khong duoc de loi
    MQTT lam chet thread nay, cac chuc nang khac cua add-on khong lien quan)."""
    stop_event = stop_event or threading.Event()
    last_state: dict = {}
    while not stop_event.is_set():
        try:
            with _vport_registry_lock:
                items = list(_vport_registry.items())
            active_names = {name for name, entry in items if entry.get("vport") is not None}
            last_state = {name: state for name, state in last_state.items() if name in active_names}
            for name, entry in items:
                vport = entry.get("vport")
                if vport is None:  # port dang enabled=false, khong co gi doi
                    continue
                device_path = vport.device_path
                connected = getattr(vport, "connected_clients", 0) > 0
                key = (vport, device_path, connected)
                if last_state.get(name) != key:
                    mqtt_publish_state(client, name, device_path, connected)
                    last_state[name] = key
        except Exception:  # noqa: BLE001 - khong duoc de loi MQTT lam chet thread
            logging.exception("[mqtt] loi khong luong truoc trong vong lap publish state")
        stop_event.wait(poll_interval_s)


# ---------------------------------------------------------------------------
# USB device inventory - liet ke TOAN BO thiet bi serial dang cam (khong chi
# thiet bi da duoc port ao "nhan"), kem thong tin dinh danh phan cung that
# (DEVNAME/by-id/by-path/ID_SERIAL/ID_USB_DRIVER, doc truc tiep tu sysfs -
# khong can them thu vien/binary nao) va 1 o ghi chu tu do nguoi dung tu
# dien (vd "day la dongle Zigbee", "cong RS485 Growatt") - giup tra cuu
# dung gia tri can dien vao exclude_usb ma khong can tu SSH go
# `ls -la /dev/serial/by-id/` + `udevadm info` moi lan.
#
# Ghi chu luu o /data/ (giong last_matched_ports.json) - SONG SOT qua moi
# lan restart/rebuild/update add-on, vi /data la volume rieng cua add-on
# tren host, KHONG nam trong Docker image bi ghi de luc build lai.
# ---------------------------------------------------------------------------

USB_NOTES_PATH = "/data/usb_device_notes.json"
# PORT_NOTES_PATH: ghi chu RIENG cho tung PORT AO (khoa = ten port ao, on
# dinh/ASCII, khac han USB_NOTES_PATH dung cho THIET BI VAT LY, khoa = by-
# path/by-id). 2 file tach biet vi 2 khai niem khac nhau: 1 port ao co the
# doi thiet bi vat ly ben duoi (rescan sang /dev/ttyUSBx khac) ma ghi chu
# "day la port lam gi" cua nguoi dung van phai giu nguyen theo port ao, KHONG
# theo thiet bi vat ly cu the.
PORT_NOTES_PATH = "/data/port_ao_notes.json"
# USB_SEEN_PATH: thong tin dinh danh LAN CUOI THAY cua MOI thiet bi USB tung
# cam (khoa = by-id > by-path > devname, giong note_key) - rut ra thi bang
# "Tat ca cong USB" van hien lai day du DEVNAME/by-id/by-path/VID:PID... (lam
# mo) thay vi 1 dong trong "-", toi khi nguoi dung bam Xoa.
USB_SEEN_PATH = "/data/usb_device_seen.json"
_USB_SEEN_FIELDS = ("devname", "by_id", "by_path", "id_serial", "id_vendor", "id_product",
                    "manufacturer", "product", "id_usb_driver")
_USB_SEEN_REFRESH_S = 600  # chi ghi lai last_seen toi da 10 phut/lan (tranh ghi dia moi 3s)
_notes_lock = threading.Lock()


def _load_notes(path: str) -> dict:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def _save_note(path: str, key: str, note: str) -> None:
    with _notes_lock:
        notes = _load_notes(path)
        if note:
            notes[key] = note
        else:
            notes.pop(key, None)  # ghi chu rong -> xoa luon, khong luu rac
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(notes, f, ensure_ascii=False)
        except OSError as exc:
            logging.warning("Khong ghi duoc %s: %s", path, exc)


def _update_usb_seen(rows: list) -> dict:
    """Ghi thong tin cac thiet bi DANG CAM vao USB_SEEN_PATH (chi ghi khi co
    thay doi hoac last_seen cu hon _USB_SEEN_REFRESH_S). Tra ve cache moi."""
    now = time.time()
    with _notes_lock:
        seen = _load_notes(USB_SEEN_PATH)
        changed = False
        for row in rows:
            key = row["device_key"]
            meta = {f: row.get(f) for f in _USB_SEEN_FIELDS}
            old = seen.get(key) or {}
            if {f: old.get(f) for f in _USB_SEEN_FIELDS} != meta or \
                    now - old.get("last_seen", 0) > _USB_SEEN_REFRESH_S:
                meta["last_seen"] = now
                seen[key] = meta
                changed = True
        if changed:
            try:
                with open(USB_SEEN_PATH, "w", encoding="utf-8") as f:
                    json.dump(seen, f, ensure_ascii=False)
            except OSError as exc:
                logging.warning("Khong ghi duoc %s: %s", USB_SEEN_PATH, exc)
    return seen


def _forget_usb_device(key: str) -> None:
    """Nut Xoa o dong "Khong con cam": xoa ca ghi chu lan thong tin lan cuoi thay."""
    _save_note(USB_NOTES_PATH, key, "")
    with _notes_lock:
        seen = _load_notes(USB_SEEN_PATH)
        if seen.pop(key, None) is not None:
            try:
                with open(USB_SEEN_PATH, "w", encoding="utf-8") as f:
                    json.dump(seen, f, ensure_ascii=False)
            except OSError as exc:
                logging.warning("Khong ghi duoc %s: %s", USB_SEEN_PATH, exc)


def _find_usb_device_node(devname: str, max_up: int = 6) -> str | None:
    """Do nguoc len tu /sys/class/tty/<ten>/device (thuong la NODE INTERFACE
    USB, vd '1-2:1.0') toi dung THU MUC THIET BI USB CHA - nhan biet bang co
    file 'idVendor' (chi node thiet bi USB that su co file nay, node
    interface/hub/host-controller thi khong). Day la MOC CO DINH DUY NHAT
    dung de doc serial/manufacturer/product/idProduct - KHONG duoc di TIEP
    len cao hon node nay (USB hub, PCI host controller cha) vi da xac nhan
    thuc te: di qua luon se doc nham gia tri KHONG thuoc ve thiet bi nay (vd
    ID_SERIAL tung hien nham thanh dia chi PCI dang '0000:00:07.2' cua host
    controller, khong phai serial number that cua thiet bi USB). Tra ve
    None neu khong tim thay (container khong thay /sys, thiet bi da rut ra,
    hoac cau truc sysfs khac thuong) - KHONG BAO GIO raise."""
    base = os.path.basename(devname)
    try:
        path = os.path.realpath(f"/sys/class/tty/{base}/device")
    except OSError:
        return None
    if not os.path.exists(path):
        return None
    for _ in range(max_up):
        if os.path.isfile(os.path.join(path, "idVendor")):
            return path
        parent = os.path.dirname(path)
        if parent == path:
            return None
        path = parent
    return None


def _read_sysfs_file(node_path: str | None, filename: str) -> str | None:
    """Doc 1 file thuoc tinh (vd 'serial', 'manufacturer', 'product',
    'idVendor', 'idProduct') DUNG TAI node_path da xac dinh (khong tu do
    nguoc len them) - dung voi node tra ve tu _find_usb_device_node(). Tra
    ve None neu file khong ton tai/khong doc duoc/rong - KHONG BAO GIO raise."""
    if not node_path:
        return None
    candidate = os.path.join(node_path, filename)
    if not os.path.isfile(candidate):
        return None
    try:
        with open(candidate, "r", encoding="utf-8", errors="replace") as f:
            value = f.read().strip()
        return value or None
    except OSError:
        return None


def _read_sysfs_driver(devname: str) -> str | None:
    """Doc ten driver kernel dang dieu khien thiet bi - o dung cap
    /sys/class/tty/<ten>/device/driver (symlink) - driver bind dung tai day
    cho hau het chip USB-serial don gian (ch341-uart, cp210x, ftdi_sio,
    cdc_acm...), khong can do nguoc len. Tra ve None neu khong doc duoc."""
    base = os.path.basename(devname)
    try:
        target = os.readlink(f"/sys/class/tty/{base}/device/driver")
        return os.path.basename(target)
    except OSError:
        return None


def _build_dev_symlink_map(dir_path: str) -> dict:
    """Map path THAT (realpath) -> ten symlink trong 1 thu muc /dev/serial/
    (by-id hoac by-path). Tra ve {} neu thu muc khong ton tai (vd host khong
    co thiet bi nao co by-id)."""
    result = {}
    try:
        entries = os.listdir(dir_path)
    except OSError:
        return result
    for name in entries:
        full = os.path.join(dir_path, name)
        try:
            real = os.path.realpath(full)
        except OSError:
            continue
        result.setdefault(real, name)
    return result


def list_usb_device_inventory(scan_glob: str, claimed: dict, exclude_raw: list) -> list:
    """Liet ke TOAN BO thiet bi serial dang cam khop scan_glob (KHONG loai
    tru gi - khac find_candidates() dung cho vong quet tu dong, o day can
    thay CA thiet bi da bi loai tru de nguoi dung doi chieu) kem day du
    thong tin dinh danh + ghi chu da luu. KHONG BAO GIO raise."""
    candidates = find_candidates(scan_glob)
    by_id_map = _build_dev_symlink_map("/dev/serial/by-id")
    by_path_map = _build_dev_symlink_map("/dev/serial/by-path")
    excluded_set = resolve_exclude_usb(exclude_raw)
    usb_notes = _load_notes(USB_NOTES_PATH)
    port_notes = _load_notes(PORT_NOTES_PATH)

    result = []
    for path in candidates:
        try:
            real = os.path.realpath(path)
        except OSError:
            real = path
        by_id = by_id_map.get(real)
        by_path = by_path_map.get(real)
        claimed_by = claimed.get(path)

        if claimed_by:
            # Thiet bi dang duoc 1 port ao "nhan" (xac dinh qua active-
            # probing, khong phai suy doan tu thuoc tinh USB tinh) - day la
            # truong hop CHINH XAC 2 CH340 giong het nhau (cung/khong co
            # by-id, co the trung by-path neu tinh co cung dai bus) can phan
            # biet - danh tinh DUY NHAT dang tin duoc luc nay la VAI TRO do
            # port ao dam nhiem, nen dung THANG ten port ao lam khoa, va
            # DUNG CHUNG 1 nguon ghi chu voi bang "Danh sach port ao" (sua o
            # bang nao cung ra cung 1 gia tri, khong tach rieng 2 ban ghi).
            note_key = claimed_by
            note_store = "port"
            note = port_notes.get(claimed_by, "")
        else:
            # Chua/khong duoc port ao nao quan ly - uu tien by-id (thiet bi
            # DON LE, khong phai 1 trong cap CH340 clone giong het nhau danh
            # cho port ao, nen it kha nang trung), roi by-path (co the doi
            # neu cam sang cong khac, nhung van on dinh hon devname), cuoi
            # cung moi fallback ve devname (doi lien tuc sau moi reboot).
            note_key = by_id or by_path or path
            note_store = "usb"
            note = usb_notes.get(note_key, "")

        usb_node = _find_usb_device_node(path)
        result.append({
            "device_key": by_id or by_path or path,
            "devname": path,
            "by_id": by_id,
            "by_path": by_path,
            "id_serial": _read_sysfs_file(usb_node, "serial"),
            "id_vendor": _read_sysfs_file(usb_node, "idVendor"),
            "id_product": _read_sysfs_file(usb_node, "idProduct"),
            "manufacturer": _read_sysfs_file(usb_node, "manufacturer"),
            "product": _read_sysfs_file(usb_node, "product"),
            "id_usb_driver": _read_sysfs_driver(path),
            "claimed_by": claimed_by,
            "excluded": bool(excluded_set and (path in excluded_set or real in excluded_set)),
            "note_key": note_key,
            "note_store": note_store,
            "note": note,
            "unavailable": False,
        })

    # Thiet bi TUNG cam (USB_SEEN_PATH) hoac tung duoc ghi chu (usb_notes)
    # nhung lan quet nay khong con thay (rut ra, tam thoi khong nhan) - van
    # hien voi thong tin LAN CUOI THAY + nhan "khong con cam" (lam mo tren
    # UI) + nut Xoa rieng, khong mat du lieu am tham chi vi rut day 1 lan.
    # "present" xet MOI dinh danh cua thiet bi dang cam (by-id/by-path/
    # devname) - thiet bi dang do port ao giu (ghi chu chuyen sang theo ten
    # port ao) KHONG bi bao nham "khong con cam" chi vi ghi chu cu theo by-id.
    seen = _update_usb_seen(result)
    present = set()
    for row in result:
        present.update(k for k in (row["device_key"], row["by_id"], row["by_path"], row["devname"]) if k)
    for key in sorted(set(seen) | set(usb_notes)):
        if key in present:
            continue
        meta = seen.get(key) or {}
        result.append({
            "device_key": key,
            **{f: meta.get(f) for f in _USB_SEEN_FIELDS},
            "last_seen": meta.get("last_seen"),
            "claimed_by": None,
            "excluded": False,
            "note_key": key,
            "note_store": "usb",
            "note": usb_notes.get(key, ""),
            "unavailable": True,
        })
    return result


# ---------------------------------------------------------------------------
# Event log - luu tam TRONG BO NHO (KHONG ghi xuong /data, mat het sau khi
# restart container - chi de xem NHANH qua tab "Log" cua web UI; log day du,
# vinh vien van nam trong logging chuan cua add-on nhu tu truoc gio, xem qua
# tab Log cua Supervisor). Muc dich: nguoi dung khong can doc log dang text
# thuan de biet khi nao 1 port ao mat/co lai ket noi USB, client TCP nao vao/
# ra, hay loi cau hinh gi vua xay ra - deque(maxlen=...) tu dong bo dong cu
# nhat khi day, khong can don dep thu cong.
#
# "category" quyet dinh mau hien tren web UI:
#   connect    = xanh la    - port ao vua ghim duoc thiet bi (moi/co lai)
#   disconnect = do         - MAT ket noi USB that su (loi doc/ghi serial)
#   warning    = vang       - chua tim thay thiet bi khop, dang cho do lai
#   error      = do dam     - loi cau hinh/he thong can chu y (khong mo duoc
#                              port, sai tham so...)
#   client     = xanh duong - client TCP (Node-RED/ModbusSpy...) vao/ra, HOAC
#                              ket qua "Test ket noi nhanh" tren tab Log
#   info       = xam        - thong tin khac (rescan thu cong...)
# ---------------------------------------------------------------------------

_EVENT_LOG_MAXLEN = 500
_event_log_lock = threading.Lock()
_event_log: deque = deque(maxlen=_EVENT_LOG_MAXLEN)


def log_event(category: str, port: str | None, message: str) -> None:
    entry = {
        "ts": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "category": category,
        "port": port or "-",
        "message": message,
    }
    with _event_log_lock:
        _event_log.append(entry)


def event_log_snapshot(limit: int = 300) -> list:
    """Tra ve ban ghi MOI NHAT truoc (de web UI khong phai tu dao nguoc)."""
    with _event_log_lock:
        items = list(_event_log)
    items.reverse()
    return items[:limit]


# ---------------------------------------------------------------------------
# Spy/Test web UI - cong cu "mo mam" cho nguoi dung: chon 1 cong USB that,
# bam nghe (spy) hoac gui 1 lenh test, xem raw hex/text THAT nhan duoc ngay
# lap tuc - de biet chinh xac nen dien gi vao expected_response, thay vi
# doan mo CRC/unit_id/function_code. Hien qua Ingress (sidebar Home Assistant).
# ---------------------------------------------------------------------------

_spy_runtime = {
    "scan_glob": "/dev/ttyUSB*,/dev/ttyACM*",
    "claimed_lock": None,
    "claimed": None,
    "default_listen_s": 2.0,
    "exclude_usb": [],
}

# _vport_registry: TAT CA port ao khai bao trong config (KE CA dang tat) -
# de Spy/Test web UI liet ke duoc day du, khong chi cac port dang chay.
# key = ten port ao. value = {"enabled": bool, "vport": VirtualPort|None}.
# "vport" la None neu port dang bi enabled=false (khong co thread nao chay
# cho no, khong co gi de doc trang thai song).
_vport_registry_lock = threading.Lock()
_vport_registry: dict = {}


def _ports_status_snapshot() -> list:
    """Tra ve danh sach trang thai TAT CA port ao (bat/tat), dung cho
    Spy/Test web UI. Khong bao gio raise."""
    result = []
    port_notes = _load_notes(PORT_NOTES_PATH)
    with _vport_registry_lock:
        items = list(_vport_registry.items())
    for name, entry in items:
        enabled = entry.get("enabled", True)
        vport = entry.get("vport")
        if not enabled or vport is None:
            result.append({
                "name": name,
                "enabled": False,
                "state": "Đã tắt (disabled)",
                "protocol": entry.get("protocol"),
                "output_mode": entry.get("output_mode"),
                "tcp_port": entry.get("tcp_port"),
                "pty_symlink": entry.get("pty_symlink"),
                "device_path": None,
                "connected_clients": 0,
                "bytes_read": 0,
                "bytes_written": 0,
                "messages_read": 0,
                "messages_written": 0,
                "note": port_notes.get(name, ""),
            })
            continue
        device_path = vport.device_path
        clients = getattr(vport, "connected_clients", 0)
        if vport.test_hold.is_set():
            state = f"⏸ Đang nhường {vport.test_hold_device} cho test - chưa trả quyền"
        elif device_path is None:
            state = "Đang dò thiết bị..."
        elif vport.output_mode == "pty":
            state = f"Đã ghim {device_path} (pty)"
        elif clients > 0:
            state = f"Đang kết nối ({clients} client)"
        else:
            state = "Đã ghim, chưa có client nào kết nối"
        result.append({
            "name": name,
            "enabled": True,
            "state": state,
            "protocol": vport.protocol,
            "output_mode": vport.output_mode,
            "tcp_port": vport.tcp_port,
            "pty_symlink": vport.pty_symlink,
            "device_path": device_path,
            "connected_clients": clients,
            # Thong ke chi de xem cho vui (bytes/so lan doc-ghi CONG DON tu
            # luc port ao nay ghim duoc thiet bi lan gan nhat) - "read" =
            # serial -> client, "written" = client -> serial.
            "bytes_read": vport.bytes_read,
            "bytes_written": vport.bytes_written,
            "messages_read": vport.messages_read,
            "messages_written": vport.messages_written,
            "test_hold": vport.test_hold.is_set(),
            "test_hold_device": vport.test_hold_device,
            "note": port_notes.get(name, ""),
        })
    return result


def _test_port_connection(name: str) -> dict:
    """"Test ket noi nhanh" cho tab Log: add-on TU ket noi TCP toi CHINH
    port ao do (127.0.0.1:tcp_port) - chi xac nhan bridge dang mo va chap
    nhan ket noi (giong telnet/ping don gian), KHONG dung gi toi thiet bi
    serial that phia sau. Dung cho nguoi dung kiem tra nhanh "port ao con
    song khong" ma khong can mo Node-RED/tool ngoai."""
    with _vport_registry_lock:
        entry = _vport_registry.get(name)
    if not entry:
        return {"ok": False, "error": f"Không tìm thấy port ảo tên {name!r}"}
    if not entry.get("enabled") or entry.get("vport") is None:
        return {"ok": False, "error": "Port ảo này đang tắt (enabled=false)"}
    vport = entry["vport"]
    if vport.output_mode != "tcp" or not vport.tcp_port:
        return {"ok": False, "error": "Port này dùng output_mode=pty, không test qua TCP được"}
    start = time.time()
    try:
        with socket.create_connection(("127.0.0.1", vport.tcp_port), timeout=2.0):
            pass
        latency_ms = round((time.time() - start) * 1000, 1)
        log_event("client", name, f"Test kết nối TCP :{vport.tcp_port} thành công ({latency_ms}ms)")
        return {
            "ok": True, "latency_ms": latency_ms, "tcp_port": vport.tcp_port,
            "device_path": vport.device_path,
        }
    except OSError as exc:
        log_event("error", name, f"Test kết nối TCP :{vport.tcp_port} thất bại: {exc}")
        return {"ok": False, "error": str(exc), "tcp_port": vport.tcp_port}


def _find_bridged_vport(device: str):
    """Tra ve VirtualPort dang ghim 'device' (so theo realpath), hoac None."""
    try:
        real = os.path.realpath(device)
    except OSError:
        real = device
    with _vport_registry_lock:
        entries = list(_vport_registry.values())
    for entry in entries:
        vport = entry.get("vport")
        path = vport.device_path if vport else None
        if path and (path == device or os.path.realpath(path) == real):
            return vport
    return None


def _tap_vport(vport, listen_s: float) -> bytes:
    """Nghe len THAT SU thu dong: dang ky 1 tap vao bridge dang chay, doi
    listen_s giay roi go ra - bridge tu copy byte doc duoc vao day."""
    buf = bytearray()
    with vport.taps_lock:
        vport.taps.append(buf)
    try:
        time.sleep(max(0.0, min(listen_s, 60.0)))
    finally:
        with vport.taps_lock:
            vport.taps.remove(buf)
    return bytes(buf)


def _spy_build_frame(protocol: str, send_command: str | None, unit_id, function_code,
                      start_address, quantity, value) -> bytes | None:
    """Build khung se gui, tuy theo protocol chon tren Spy UI - y het cach
    build_probe_frames() cua VirtualPort lam voi 1 port ao that:
      - "modbus_rtu": tu dong build khung + tinh CRC tu unit_id/function_code/
        start_address/quantity/value (nguoi dung KHONG can biet CRC la gi).
      - "raw": dung nguyen van send_command (hex:... hoac text:...).
    Tra ve None neu khong co gi de gui (raw ma send_command rong)."""
    if protocol == "modbus_rtu":
        return build_modbus_request(
            int(unit_id), int(function_code), int(start_address or 0),
            int(quantity or 1), (int(value) if value not in (None, "") else None),
        )
    if send_command:
        return decode_command(send_command)
    return None


class _TestLinkError(Exception):
    def __init__(self, msg: str, busy: bool = False):
        super().__init__(msg)
        self.busy = busy


class _TestLink:
    """Ket noi tam cho Get Response/Communication (server), 4 che do:
      - "exclusive": cong ranh -> mo doc quyen nhu cu.
      - "vport": cong dang do port ao ghim + share=True -> ghi qua CHINH fd
        cua bridge (serial_write_lock), doc qua tap - KHONG mo fd thu 2, KHONG
        doi baud. Phan hoi van bi bridge phat cho client dang noi port ao.
      - "tap": cong dang do port ao ghim + share=False -> chi nghe (nhu cu).
      - "shared_fd": cong do tien trinh NGOAI add-on giu + share=True -> mo
        chung fd (exclusive=False). RUI RO: doi baud cua tien trinh kia +
        tranh byte (TROUBLESHOOTING_NOTES.md muc 25) - chi khi nguoi dung chon.
    Muon test doc quyen cong cua port ao -> "Lay quyen port ao" (_take_test_hold)
    roi mo lai, luc do se vao che do "exclusive"."""

    def __init__(self, device: str, baud: int, timeout_s: float, share: bool):
        self.ser = None
        self.vport = None
        self.note = None
        try:
            self.ser = _open_serial(device, baud, timeout_s, exclusive=True)
            self.mode = "exclusive"
            return
        except Exception as exc:  # noqa: BLE001
            if not is_port_busy_error(exc):
                raise _TestLinkError(f"Không mở được port: {exc}") from exc
        self.vport = _find_bridged_vport(device)
        if self.vport is not None:
            if share and self.vport.active_serial is not None:
                self.mode = "vport"
                self.note = (
                    f"Dùng chung: gửi qua kết nối của port ảo '{self.vport.name}' "
                    f"(baud {self.vport.baud}, bỏ qua baud chọn ở đây). Phản hồi cũng tới "
                    "client đang nối port ảo, và có thể lẫn byte của client đó."
                )
            else:
                self.mode = "tap"
                self.note = (
                    f"Port đang được port ảo '{self.vport.name}' ghim - chỉ nghe qua bridge "
                    f"(baud {self.vport.baud}). Muốn gửi: tick \"Dùng chung\" hoặc bấm "
                    "\"Lấy quyền port ảo\"."
                )
            return
        if not share:
            raise _TestLinkError(
                "Port đang bị 1 tiến trình ngoài add-on giữ. Tick \"Dùng chung\" để cố mở "
                "chung (rủi ro đổi baud/tranh byte của tiến trình đó).",
                busy=True,
            )
        try:
            self.ser = _open_serial(device, baud, timeout_s, exclusive=False)
        except Exception as exc:  # noqa: BLE001
            raise _TestLinkError(f"Không mở chung được port: {exc}", busy=True) from exc
        self.mode = "shared_fd"
        self.note = (
            "Dùng chung fd với tiến trình ngoài add-on - baud của tiến trình đó có thể bị "
            "đổi thành baud chọn ở đây, byte có thể bị 2 bên tranh nhau."
        )

    @property
    def can_send(self) -> bool:
        return self.mode != "tap"

    def listen(self, listen_s: float) -> bytes:
        if self.ser is None:
            return _tap_vport(self.vport, listen_s)
        if self.mode == "exclusive":
            self.ser.reset_input_buffer()
        return _listen(self.ser, listen_s)

    def transact(self, frame: bytes, wait_s: float) -> bytes:
        """Gui 1 khung, tra ve byte nhan duoc trong wait_s giay."""
        if self.mode == "tap":
            raise _TestLinkError("Cổng đang bận - chỉ nghe được, không gửi.", busy=True)
        if self.mode == "vport":
            if self.vport.mbap_rtu_bridge:
                raise _TestLinkError(
                    "Port đang dịch Modbus TCP/RTU. Bấm Lấy quyền port ảo để test; "
                    "không gửi xen lệnh test vào giao dịch điều khiển.", busy=True)
            ser = self.vport.active_serial
            if ser is None:
                raise _TestLinkError("Port ảo vừa đóng kết nối - thử lại.", busy=True)
            buf = bytearray()
            with self.vport.taps_lock:
                self.vport.taps.append(buf)
            try:
                with self.vport.serial_write_lock:
                    ser.write(frame)
                time.sleep(wait_s + 0.05)
            except (serial.SerialException, OSError) as exc:
                raise _TestLinkError(f"Ghi qua port ảo lỗi: {exc}", busy=True) from exc
            finally:
                with self.vport.taps_lock:
                    self.vport.taps.remove(buf)
            return bytes(buf)
        self.ser.timeout = wait_s
        if self.mode == "exclusive":
            self.ser.reset_input_buffer()
        self.ser.write(frame)
        time.sleep(0.05)
        return self.ser.read(512)

    def close(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except OSError:
                pass


def _find_modbus_response(buf: bytes, unit_id: int, function_code: int) -> bytes | None:
    """Tim 1 khung RTU phan hoi hop le (dung unit_id/FC, dung CRC) O BAT KY
    vi tri nao trong buf - che do "Dung chung" doc qua tap nen buffer co the
    lan byte cua client khac truoc/sau khung can tim."""
    for i in range(len(buf) - 4):
        if buf[i] != unit_id or buf[i + 1] not in (function_code, function_code | 0x80):
            continue
        need = rtu_response_expected_len(buf[i:])
        if need is None or i + need > len(buf):
            continue
        frame = buf[i:i + need]
        if check_modbus_response(unit_id, function_code, frame):
            return frame
    return None


def _spy_probe_port(device: str, baud: int, listen_s: float, spy_only: bool,
                     protocol: str = "raw", send_command: str | None = None,
                     unit_id=None, function_code=None,
                     start_address=0, quantity=1, value=None, share: bool = False) -> dict:
    """Mo thu 1 port (xem _TestLink), nghe passive listen_s giay, neu co lenh
    gui (raw hoac modbus_rtu, xem _spy_build_frame) thi gui va doc them
    response chu dong. KHONG BAO GIO raise - moi loi deu tra ve trong field
    'error' de web UI hien thi, khong lam sap http server."""
    result: dict = {"device": device, "baud": baud, "busy": False}
    try:
        link = _TestLink(device, baud, 0.2, share)
    except _TestLinkError as exc:
        result["busy"] = exc.busy
        result["error"] = str(exc)
        return result
    result["busy"] = link.mode != "exclusive"
    result["link_mode"] = link.mode
    if link.note:
        result["note"] = link.note

    try:
        buf = link.listen(listen_s)
        result["passive_hex"] = buf.hex()
        result["passive_display"] = describe_bytes(buf)

        if spy_only:
            return result
        if not link.can_send:
            result["error"] = "Chưa gửi lệnh - cổng đang bận (xem ghi chú)."
            return result

        try:
            frame = _spy_build_frame(
                protocol, send_command, unit_id, function_code, start_address, quantity, value
            )
        except Exception as exc:  # noqa: BLE001
            result["error"] = f"Lệnh gửi không hợp lệ: {exc}"
            return result

        if frame:
            result["sent_hex"] = frame.hex()
            result["sent_display"] = describe_bytes(frame)
            resp = link.transact(frame, 1.0)
            result["active_hex"] = resp.hex()
            result["active_display"] = describe_bytes(resp)
    except _TestLinkError as exc:
        result["error"] = str(exc)
    except (serial.SerialException, OSError) as exc:
        result["error"] = f"Lỗi đọc/ghi port: {exc}"
    finally:
        link.close()
    return result


# ---------------------------------------------------------------------------
# Communication tab - quet Modbus RTU khi khong co tai lieu thiet bi: quet
# unit_id (bus co bao nhieu thiet bi, dia chi bao nhieu) roi quet thanh ghi
# cua 1 dia chi cu the (thanh ghi nao tra loi hop le). Dung lai nguyen
# build_modbus_request/check_modbus_response/_open_serial da co, khong them
# logic Modbus moi - chi lap qua nhieu unit_id/address roi goi lai cung 1
# cach kiem tra CRC+function_code nhu luc dò port ao that.
# ---------------------------------------------------------------------------

def _open_scan_link(device: str, baud: int, timeout_s: float, share: bool):
    """Mo _TestLink cho quet Communication - tra (link, None) hoac (None, dict loi)."""
    try:
        link = _TestLink(device, baud, timeout_s, share)
    except _TestLinkError as exc:
        return None, {"error": str(exc)}
    if not link.can_send:
        link.close()
        return None, {"error": (
            f"Cổng đang được port ảo '{link.vport.name}' ghim - tick \"Dùng chung\" "
            "hoặc bấm \"Lấy quyền port ảo\" để quét."
        )}
    return link, None


def _modbus_scan_units(device: str, baud: int, unit_start: int, unit_end: int,
                        function_code: int, start_address: int, quantity: int,
                        timeout_s: float = 0.3, share: bool = False) -> dict:
    """Gui 1 lenh doc toi TUNG unit_id trong dai [unit_start, unit_end], ghi
    lai dia chi nao co phan hoi hop le (dung CRC + dung unit_id) - dung khi
    chua biet bus RS-485 nay co bao nhieu thiet bi, dia chi bao nhieu. CHAY
    CHAM (moi dia chi mat ~timeout_s+0.05 giay) - gioi han dai toi da 247
    dia chi/lan de tranh 1 request HTTP treo qua lau. KHONG BAO GIO raise -
    loi tra ve qua key 'error'."""
    if unit_end < unit_start:
        return {"error": "unit_id ket thuc phai >= bat dau"}
    if unit_end - unit_start > 247:
        return {"error": "dai qua rong (toi da 247 dia chi 1 lan quet)"}
    link, err = _open_scan_link(device, baud, timeout_s, share)
    if err:
        return err
    found = []
    result = {"found": found, "scanned": unit_end - unit_start + 1}
    if link.note:
        result["note"] = link.note
    try:
        for uid in range(unit_start, unit_end + 1):
            frame = build_modbus_request(uid, function_code, start_address, quantity)
            resp = _find_modbus_response(link.transact(frame, timeout_s), uid, function_code)
            if resp:
                found.append({"unit_id": uid, "response_hex": resp.hex()})
    except (_TestLinkError, serial.SerialException, OSError) as exc:
        result["error"] = f"Dừng quét giữa chừng: {exc}"
    finally:
        link.close()
    return result


def _modbus_scan_registers(device: str, baud: int, unit_id: int, function_codes: list,
                            addr_start: int, addr_end: int, quantity: int = 1,
                            timeout_s: float = 0.3, share: bool = False) -> dict:
    """Quet 1 dai dia chi thanh ghi cho DUNG 1 unit_id da biet, thu lan luot
    tung function code trong 'function_codes' - dung de 'mo' thiet bi khong
    co tai lieu: thanh ghi nao tra loi hop le (dung CRC) rat co the la thanh
    ghi that su thiet bi dang dung, dung de khoanh vung truoc khi doc thu gia
    tri that doi chieu. Gioi han dai toi da 500 dia chi/lan (nhan them so
    function_code) de tranh 1 request treo qua lau."""
    if addr_end < addr_start:
        return {"error": "address ket thuc phai >= bat dau"}
    if addr_end - addr_start > 500:
        return {"error": "dai qua rong (toi da 500 dia chi 1 lan quet)"}
    if not function_codes:
        return {"error": "can it nhat 1 function code de quet"}
    link, err = _open_scan_link(device, baud, timeout_s, share)
    if err:
        return err
    found = []
    result = {"found": found, "scanned": (addr_end - addr_start + 1) * len(function_codes)}
    if link.note:
        result["note"] = link.note
    try:
        for fc in function_codes:
            for addr in range(addr_start, addr_end + 1):
                frame = build_modbus_request(unit_id, fc, addr, quantity)
                resp = _find_modbus_response(link.transact(frame, timeout_s), unit_id, fc)
                if resp:
                    found.append({"function_code": fc, "address": addr, "response_hex": resp.hex()})
    except (_TestLinkError, serial.SerialException, OSError) as exc:
        result["error"] = f"Dừng quét giữa chừng: {exc}"
    finally:
        link.close()
    return result


_SPY_PAGE_HTML = """<!doctype html>
<html lang="vi">
<head>
<meta charset="utf-8">
<title>USB Manager - Spy/Test</title>
<style>
  body { font-family: -apple-system, Segoe UI, Roboto, sans-serif; margin: 16px; background:#0f1115; color:#e6e6e6; }
  h1 { font-size: 1.2rem; }
  label { display:block; margin-top:12px; font-size:0.85rem; color:#9aa0a6; }
  select, input, button { width:100%; box-sizing:border-box; padding:8px; margin-top:4px; font-size:1rem;
    background:#1c1f26; color:#e6e6e6; border:1px solid #333842; border-radius:6px; }
  input[type="number"], .narrow { max-width:160px; }
  button { background:#2f6feb; color:white; border:none; cursor:pointer; margin-top:14px; font-weight:600; }
  button:disabled { opacity:0.5; cursor:not-allowed; }
  .row { display:flex; gap:10px; flex-wrap:wrap; }
  .row > div { flex:0 0 auto; }
  table.log { width:100%; border-collapse:collapse; margin-top:8px; font-size:0.8rem; }
  table.log th, table.log td { text-align:left; padding:5px 8px; border-bottom:1px solid #333842; vertical-align:top; }
  table.log th { color:#9aa0a6; font-weight:600; position:sticky; top:0; background:#0f1115; }
  table.log td.mono { font-family:monospace; word-break:break-all; }
  table.log tr.err td { color:#ff6b6b; }
  .card { border:1px solid #333842; border-radius:8px; padding:16px; background:#161a20; }
  .card h2 { margin-top:0; font-size:0.95rem; }
  .card-grid { display:grid; grid-template-columns:repeat(auto-fit, minmax(280px, 1fr)); gap:16px; margin-top:16px; }
  pre { background:#1c1f26; padding:10px; border-radius:6px; white-space:pre-wrap; word-break:break-all; font-size:0.85rem; }
  .badge { display:inline-block; padding:2px 8px; border-radius:10px; font-size:0.75rem; margin-left:6px; }
  .badge.free { background:#1e4620; color:#7ee787; }
  .badge.claimed { background:#4a3410; color:#f0b429; }
  .copybox { display:flex; gap:6px; align-items:center; margin-top:4px; }
  .copybox input { flex:1; font-family:monospace; }
  .copybox button { width:auto; margin-top:0; padding:8px 12px; }
  .note { color:#f0b429; font-size:0.85rem; margin-top:8px; }
  .err { color:#ff6b6b; font-size:0.9rem; margin-top:8px; }
  .busy-box { margin-top:10px; padding:8px 10px; border:1px dashed #333842; border-radius:6px; font-size:0.85rem; }
  .hold-off, .hold-on { margin-top:8px; display:flex; flex-wrap:wrap; align-items:center; gap:8px; }
  .hold-on { color:#f0b429; }
  .hold-btn { width:auto; margin-top:0; padding:6px 12px; }
  .hold-btn.take { background:#8a4b08; }
  .hold-btn.release { background:#1f6f3a; }
  table.ports { width:100%; border-collapse:collapse; margin-top:8px; font-size:0.85rem; }
  table.ports th, table.ports td { text-align:left; padding:6px 8px; border-bottom:1px solid #333842; }
  table.ports th { color:#9aa0a6; font-weight:600; }
  .dot { display:inline-block; width:8px; height:8px; border-radius:50%; margin-right:6px; }
  .dot.off { background:#666; }
  .dot.waiting { background:#f0b429; }
  .dot.idle { background:#5b9bd5; }
  .dot.connected { background:#3fb950; }
  .tab-bar { display:flex; gap:4px; margin-bottom:20px; border-bottom:1px solid #333842; }
  .tab-btn { width:auto; margin-top:0; background:transparent; color:#9aa0a6; border:none;
    border-bottom:3px solid transparent; border-radius:0; padding:10px 16px; font-weight:600; }
  .tab-btn.active { color:#e6e6e6; border-bottom-color:#2f6feb; }
  .tab-content { display:none; }
  .tab-content.active { display:block; }
  tr.log-connect td { color:#7ee787; }
  tr.log-disconnect td { color:#ff6b6b; }
  tr.log-warning td { color:#f0b429; }
  tr.log-error td { color:#ff6b6b; font-weight:600; }
  tr.log-client td { color:#5b9bd5; }
  tr.log-info td { color:#9aa0a6; }
  .log-badge { display:inline-block; padding:2px 8px; border-radius:10px; font-size:0.72rem; font-weight:600; white-space:nowrap; }
  .log-badge.connect { background:#1e4620; color:#7ee787; }
  .log-badge.disconnect { background:#4a1f1f; color:#ff8080; }
  .log-badge.warning { background:#4a3410; color:#f0b429; }
  .log-badge.error { background:#4a1f1f; color:#ff6b6b; }
  .log-badge.client { background:#1c3a52; color:#5b9bd5; }
  .log-badge.info { background:#262b33; color:#9aa0a6; }
</style>
</head>
<body>
<div class="tab-bar">
  <button type="button" class="tab-btn" data-tab="tab_overview">🏠 USB Manager</button>
  <button type="button" class="tab-btn" data-tab="tab_config">⚙ Cấu hình</button>
  <button type="button" class="tab-btn" data-tab="tab_getresponse">Get Response</button>
  <button type="button" class="tab-btn" data-tab="tab_comm">Communication</button>
  <button type="button" class="tab-btn" data-tab="tab_log">📋 Log</button>
</div>

<button type="button" id="settings_gear_btn" title="Cài đặt trình duyệt (localStorage)"
  style="position:fixed; top:12px; right:12px; width:auto; margin-top:0; padding:8px 10px; z-index:100;
    background:#1c1f26; border:1px solid #333842; font-size:1.1rem; border-radius:6px;">⚙️</button>
<div id="addon_version_label" style="position:fixed; top:60px; right:14px; z-index:90; font-size:0.7rem; color:#666;"></div>

<div id="settings_panel" style="display:none; position:fixed; top:54px; right:12px; width:300px; z-index:100;
  background:#161a20; border:1px solid #333842; border-radius:8px; padding:16px; box-shadow:0 4px 16px rgba(0,0,0,0.4);">
  <h2 style="margin-top:0; font-size:0.95rem;">Cài đặt trình duyệt</h2>
  <p style="color:#9aa0a6; font-size:0.8rem;">
    Dữ liệu lưu TRONG TRÌNH DUYỆT NÀY (<code>localStorage</code>) — riêng
    biệt với dữ liệu trên server (ghi chú port ảo/USB, thiết bị "Không còn
    cắm") nên nút bên dưới KHÔNG đụng tới những thứ đó.
  </p>
  <div id="storage_usage_info" style="font-size:0.85rem; margin:10px 0; line-height:1.6;">Đang tính...</div>
  <button type="button" id="clear_comm_storage_btn" style="background:#4a1f1f; color:#ff8080; width:100%;">
    🗑 Xoá kết quả/log đã lưu
  </button>
  <p style="color:#9aa0a6; font-size:0.75rem; margin-top:8px;">
    Chỉ xoá kết quả quét + log "Gửi lặp lại" (tab Communication) đã lưu tạm
    trong trình duyệt. <b>KHÔNG</b> xoá ghi chú thiết bị Local PC, ghi chú
    port ảo/USB trên server, hay thiết bị "Không còn cắm".
  </p>
</div>

<div id="tab_overview" class="tab-content">
<div id="mqtt_status" style="margin-bottom:12px; font-size:0.85rem; color:#9aa0a6;">Đang kiểm tra MQTT...</div>

<h2 style="font-size:1rem; margin-top:0;">Danh sách port ảo</h2>
<div id="ports_list"><i>Đang tải...</i></div>

<hr style="border-color:#333842; margin:20px 0;">

<h2 style="font-size:1rem; margin-top:0;">Tất cả cổng USB đang cắm trên host</h2>
<p style="color:#9aa0a6; font-size:0.85rem;">
  Liệt kê MỌI thiết bị khớp <code>scan_glob</code> (kể cả thiết bị đã bị loại
  trừ qua <code>exclude_usb</code>, hoặc chưa được port ảo nào dùng tới) —
  dùng để tra cứu đúng <code>by-id</code>/<code>by-path</code> cần điền vào
  <code>exclude_usb</code>. Ô "Ghi chú" tự lưu khi rời khỏi ô nhập (không cần
  nút Save), lưu ở <code>/data/</code> nên <b>không mất</b> qua
  restart/rebuild/update add-on.
</p>
<div id="usb_devices_list"><i>Đang tải...</i></div>
</div>

<div id="tab_getresponse" class="tab-content">
<h1 style="font-size:1.1rem;">Get Response - Nghe lén / Gửi lệnh thử</h1>
<p style="color:#9aa0a6; font-size:0.85rem;">
  Chọn 1 cổng USB thật, để trống "Lệnh gửi" để CHỈ NGHE (spy), hoặc điền lệnh
  để gửi chủ động rồi xem phản hồi thật. Kết quả hiện ra dạng hex - copy thẳng
  vào <code>expected_response</code> (thêm tiền tố <code>hex:</code>).
</p>

<div class="card" style="margin-bottom:16px;">
  <h2>Nguồn kết nối</h2>
  <label style="display:flex; align-items:center; gap:8px; margin-top:8px;">
    <input type="radio" name="gr_source" id="gr_source_server" value="server" checked style="width:auto; margin-top:0;">
    <span>Server (add-on trên Unraid/HA) — cổng cắm ở máy chủ</span>
  </label>
  <label style="display:flex; align-items:center; gap:8px; margin-top:6px;">
    <input type="radio" name="gr_source" id="gr_source_local" value="local" style="width:auto; margin-top:0;">
    <span>Máy tính đang mở trang này (Local PC, qua trình duyệt)</span>
  </label>
  <p id="local_pc_support_note" style="color:#9aa0a6; font-size:0.8rem; margin-top:6px;">
    Cần Chrome/Edge/Opera (không hỗ trợ Firefox/Safari) VÀ truy cập qua
    HTTPS (Web Serial API bị trình duyệt tự chặn trên HTTP thường) — kiểm
    tra khi bạn chọn mục này.
  </p>
  <div id="local_pc_section" style="display:none; margin-top:8px;">
    <div class="row">
      <div><button type="button" id="local_connect_btn">🔌 Chọn cổng USB trên máy này</button></div>
      <div><button type="button" id="local_disconnect_btn" style="background:#333842;">Ngắt kết nối</button></div>
    </div>
    <div id="local_pc_status" style="margin-top:8px; font-size:0.85rem; color:#9aa0a6;">Chưa kết nối.</div>
    <label>Ghi chú riêng cho cổng này (chỉ lưu trên máy bạn, không gửi lên server)</label>
    <input id="local_pc_note" type="text" placeholder="vd: relay test trên bàn, chưa gắn tủ điện">
  </div>
</div>

<label id="device_label">Cổng USB (tự động quét lại mỗi 3s)</label>
<select id="device" class="device-select"></select>
<div id="gr_busy_box" class="busy-box">
  <label style="display:flex; align-items:center; gap:8px; margin-top:0;">
    <input type="checkbox" id="gr_share" style="width:auto; margin-top:0;">
    <span>Dùng chung nếu cổng đang bận — cố gửi lệnh xen vào kết nối đang chạy (qua port ảo đang ghim; tiến trình ngoài thì mở chung fd, có rủi ro)</span>
  </label>
  <div id="gr_hold_box"></div>
</div>

<div class="row">
  <div>
    <label>Baud</label>
    <select id="baud" class="narrow">
      <option>1200</option><option>2400</option><option>4800</option>
      <option selected>9600</option><option>19200</option><option>38400</option>
      <option>57600</option><option>115200</option>
    </select>
  </div>
  <div>
    <label>Thời gian nghe (giây)</label>
    <input id="listen_s" type="number" value="2" min="0.5" step="0.5">
  </div>
</div>

<label style="display:flex; align-items:center; gap:8px; margin-top:16px;">
  <input type="checkbox" id="spy_only" checked style="width:auto; margin-top:0;">
  <span>CHỈ NGHE (spy) - không gửi gì cả lên bus</span>
</label>

<div id="send_section" style="display:none;">
  <label>Kiểu lệnh gửi</label>
  <select id="protocol">
    <option value="raw" selected>raw (tự gõ hex/text)</option>
    <option value="modbus_rtu">modbus_rtu (tự động build khung + tính CRC)</option>
  </select>

  <div id="raw_fields">
    <label>Lệnh gửi</label>
    <input id="send_command" type="text" placeholder='vd: hex:01 04 00 00 00 02 71 CB   hoặc   text:GET_ID$'>
  </div>

  <div id="modbus_fields" style="display:none;">
    <div class="row">
      <div><label>Unit ID</label><input id="unit_id" type="number" value="1"></div>
      <div><label>Function code</label><input id="function_code" type="number" value="4"></div>
    </div>
    <div class="row">
      <div><label>Start address</label><input id="start_address" type="number" value="0"></div>
      <div><label>Quantity (FC đọc)</label><input id="quantity" type="number" value="2"></div>
    </div>
    <label>Value (chỉ dùng cho FC ghi 05/06, để trống nếu là FC đọc)</label>
    <input id="value" type="number" placeholder="để trống nếu dùng FC 01-04">
  </div>
</div>

<button id="btn">Nghe (Spy)</button>

<div id="result" style="margin-top:16px;"></div>
</div>

<div id="tab_comm" class="tab-content">
<h1 style="font-size:1.1rem;">Communication - Khám phá &amp; test thiết bị RTU</h1>
<p style="color:#9aa0a6; font-size:0.85rem;">
  Dùng khi thiết bị Modbus RTU không có tài liệu: quét xem bus có bao nhiêu
  thiết bị (unit_id), rồi quét thanh ghi của 1 địa chỉ cụ thể để mò chức
  năng. Cũng dùng để đổi địa chỉ mặc định lần đầu, hoặc gửi lặp lại 1 lệnh
  (raw/AT/UART hay modbus_rtu) kèm log gửi/nhận theo thời gian thực.
</p>

<div class="card" style="margin-top:12px;">
  <h2>Cổng kết nối</h2>
  <label style="display:flex; align-items:center; gap:8px; margin-top:8px;">
    <input type="radio" name="comm_source" id="comm_source_server" value="server" checked style="width:auto; margin-top:0;">
    <span>Server (add-on trên Unraid/HA)</span>
  </label>
  <label style="display:flex; align-items:center; gap:8px; margin-top:6px;">
    <input type="radio" name="comm_source" id="comm_source_local" value="local" style="width:auto; margin-top:0;">
    <span>Máy tính đang mở trang này (Local PC, qua trình duyệt)</span>
  </label>
  <div id="comm_local_pc_section" style="display:none; margin-top:8px;">
    <div class="row">
      <div><button type="button" id="comm_local_connect_btn">🔌 Chọn cổng USB trên máy này</button></div>
      <div><button type="button" id="comm_local_disconnect_btn" style="background:#333842;">Ngắt kết nối</button></div>
    </div>
    <div id="comm_local_pc_status" style="margin-top:8px; font-size:0.85rem; color:#9aa0a6;">Chưa kết nối.</div>
    <label>Ghi chú riêng cho cổng này (chỉ lưu trên máy bạn)</label>
    <input id="comm_local_pc_note" type="text" placeholder="vd: relay test trên bàn, chưa gắn tủ điện">
  </div>

  <div id="comm_device_row" class="row" style="margin-top:12px;">
    <div style="flex:2 1 260px;">
      <label id="comm_device_label">Cổng USB (tự động quét lại mỗi 3s)</label>
      <select id="comm_device" class="device-select"></select>
    </div>
    <div>
      <label>Baud</label>
      <select id="comm_baud" class="narrow">
        <option>1200</option><option>2400</option><option>4800</option>
        <option selected>9600</option><option>19200</option><option>38400</option>
        <option>57600</option><option>115200</option>
      </select>
    </div>
  </div>
<div id="comm_busy_box" class="busy-box">
  <label style="display:flex; align-items:center; gap:8px; margin-top:0;">
    <input type="checkbox" id="comm_share" style="width:auto; margin-top:0;">
    <span>Dùng chung nếu cổng đang bận — cố gửi lệnh xen vào kết nối đang chạy (qua port ảo đang ghim; tiến trình ngoài thì mở chung fd, có rủi ro)</span>
  </label>
  <div id="comm_hold_box"></div>
</div>
</div>

<div class="card-grid">
  <div class="card">
    <h2>1. Quét Unit ID</h2>
    <p style="color:#9aa0a6; font-size:0.8rem; margin-top:0;">Bus RTU có bao nhiêu thiết bị, địa chỉ bao nhiêu.</p>
    <div class="row">
      <div><label>Unit ID từ</label><input id="scan_unit_start" type="number" value="1"></div>
      <div><label>đến</label><input id="scan_unit_end" type="number" value="20"></div>
    </div>
    <div class="row">
      <div><label>Function code (3 hoặc 4)</label><input id="scan_unit_fc" type="number" value="3"></div>
      <div><label>Start address</label><input id="scan_unit_addr" type="number" value="0"></div>
    </div>
    <div class="row">
      <div><label>Quantity</label><input id="scan_unit_quantity" type="number" value="1" min="1"></div>
      <div><label>Timeout mỗi lần thử (giây)</label><input id="scan_unit_timeout" type="number" value="0.3" min="0.05" step="0.05"></div>
    </div>
    <button type="button" id="scan_unit_btn">Quét Unit ID</button>
  </div>

  <div class="card">
    <h2>2. Quét thanh ghi</h2>
    <p style="color:#9aa0a6; font-size:0.8rem; margin-top:0;">
      Mò chức năng 1 thiết bị đã biết Unit ID (từ mục 1) — CHƯA chắc ý
      nghĩa từng thanh ghi, chỉ khoanh vùng để đọc thử đối chiếu.
    </p>
    <div class="row">
      <div><label>Unit ID</label><input id="scan_reg_unit" type="number" value="1"></div>
      <div><label>FC (vd 3,4)</label><input id="scan_reg_fc" type="text" class="narrow" value="3,4"></div>
    </div>
    <div class="row">
      <div><label>Address từ</label><input id="scan_reg_start" type="number" value="0"></div>
      <div><label>đến</label><input id="scan_reg_end" type="number" value="50"></div>
    </div>
    <div class="row">
      <div><label>Quantity</label><input id="scan_reg_quantity" type="number" value="1" min="1"></div>
      <div><label>Timeout mỗi lần thử (giây)</label><input id="scan_reg_timeout" type="number" value="0.3" min="0.05" step="0.05"></div>
    </div>
    <button type="button" id="scan_reg_btn">Quét thanh ghi</button>
  </div>

  <div class="card">
    <h2>3. Đổi địa chỉ thiết bị</h2>
    <p style="color:#9aa0a6; font-size:0.8rem; margin-top:0;">
      Set address lần đầu — ghi giá trị vào đúng thanh ghi lưu địa chỉ (khác
      nhau tuỳ hãng, không có cách tự động chung — mò qua mục 2 nếu cần).
      Đa số dùng FC06 (mặc định) — 1 số thiết bị dùng FC05 (lưu kiểu coil)
      hoặc FC16 (ghi nhiều thanh ghi), đổi lại nếu FC06 không có phản hồi.
    </p>
    <div class="row">
      <div><label>Unit ID hiện tại</label><input id="setaddr_unit" type="number" value="1"></div>
      <div><label>Function code</label><input id="setaddr_fc" type="number" value="6"></div>
    </div>
    <label>Address thanh ghi lưu địa chỉ</label>
    <input id="setaddr_reg" type="number" value="0">
    <div class="row">
      <div><label>Địa chỉ MỚI muốn đặt</label><input id="setaddr_value" type="number" value="2"></div>
      <div><label>Chờ phản hồi (giây)</label><input id="setaddr_listen" type="number" value="0.3" min="0.05" step="0.05"></div>
    </div>
    <button type="button" id="setaddr_btn">Gửi lệnh đổi địa chỉ</button>
  </div>

  <div class="card">
    <h2>4. Gửi lặp lại + log</h2>
    <p style="color:#9aa0a6; font-size:0.8rem; margin-top:0;">Test raw/AT/UART hoặc modbus_rtu, log ở khung bên dưới.</p>
    <label>Kiểu lệnh</label>
    <select id="repeat_protocol">
      <option value="raw" selected>raw (tự gõ hex/text - AT command, string...)</option>
      <option value="modbus_rtu">modbus_rtu</option>
    </select>
    <div id="repeat_raw_fields">
      <label>Lệnh gửi</label>
      <input id="repeat_send_command" type="text" placeholder='vd: hex:01 04 00 71 CB   hoặc   text:AT+RST'>
    </div>
    <div id="repeat_modbus_fields" style="display:none;">
      <div class="row">
        <div><label>Unit ID</label><input id="repeat_unit_id" type="number" value="1"></div>
        <div><label>Function code</label><input id="repeat_function_code" type="number" value="3"></div>
      </div>
      <div class="row">
        <div><label>Start address</label><input id="repeat_start_address" type="number" value="0"></div>
        <div><label>Quantity</label><input id="repeat_quantity" type="number" value="2"></div>
      </div>
    </div>
    <div class="row">
      <div><label>Lặp lại mỗi (giây, 0 = chỉ gửi 1 lần)</label><input id="repeat_interval_s" type="number" value="2" min="0" step="0.5"></div>
      <div><label>Chờ phản hồi (giây)</label><input id="repeat_listen_s" type="number" value="0.3" min="0.05" step="0.05"></div>
    </div>
    <div class="row">
      <div><button type="button" id="repeat_btn">▶ Bắt đầu gửi</button></div>
      <div><button type="button" id="repeat_clear_btn" style="background:#333842;">🗑 Xoá log</button></div>
    </div>
  </div>
</div>

<hr style="border-color:#333842; margin:24px 0;">
<h2 style="font-size:1.05rem;">Kết quả</h2>

<div id="scan_unit_result" style="margin-top:8px;"></div>
<div id="scan_reg_result" style="margin-top:8px;"></div>
<div id="setaddr_result" style="margin-top:8px;"></div>

<div style="margin-top:12px; max-height:420px; overflow-y:auto; border:1px solid #333842; border-radius:6px;">
  <table class="log">
    <thead><tr><th>Thời gian</th><th>Gửi (TX)</th><th>Nhận (RX)</th></tr></thead>
    <tbody id="repeat_log"></tbody>
  </table>
</div>
</div>

<div id="tab_log" class="tab-content">
<h1 style="font-size:1.1rem;">Log - Kết nối &amp; lỗi</h1>
<p style="color:#9aa0a6; font-size:0.85rem;">
  Ghi lại khi nào từng port ảo mất/có lại kết nối USB, client TCP vào/ra, và
  các lỗi cần biết (cấu hình sai, không mở được port...). Chỉ lưu tạm trong
  bộ nhớ add-on (mất khi restart container) — log đầy đủ, vĩnh viễn vẫn xem
  được qua tab Log của add-on trên Supervisor.
</p>

<div class="card" style="margin-bottom:16px;">
  <h2>Test kết nối nhanh</h2>
  <p style="color:#9aa0a6; font-size:0.8rem; margin-top:0;">
    Add-on tự kết nối TCP tới chính port ảo đó (127.0.0.1) để xác nhận bridge
    đang mở và chấp nhận kết nối — không đụng gì tới thiết bị USB thật phía
    sau.
  </p>
  <div id="port_test_list"><i>Đang tải...</i></div>
</div>

<label style="margin-top:0;">Lọc theo loại</label>
<select id="log_filter" style="max-width:280px;">
  <option value="">Tất cả</option>
  <option value="connect">🟢 Kết nối (USB)</option>
  <option value="disconnect">🔴 Mất kết nối (USB)</option>
  <option value="warning">🟡 Cảnh báo</option>
  <option value="error">🛑 Lỗi</option>
  <option value="client">🔵 Client TCP</option>
  <option value="info">⚪ Thông tin khác</option>
</select>

<div style="margin-top:12px; max-height:520px; overflow-y:auto; border:1px solid #333842; border-radius:6px;">
  <table class="log">
    <thead><tr><th>Thời gian</th><th>Loại</th><th>Port</th><th>Nội dung</th></tr></thead>
    <tbody id="event_log_body"></tbody>
  </table>
</div>
</div>

<script>
function showTab(tabId) {
  document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));
  document.getElementById(tabId).classList.add('active');
  document.querySelector(`.tab-btn[data-tab="${tabId}"]`).classList.add('active');
}
document.querySelectorAll('.tab-btn').forEach(btn => {
  btn.addEventListener('click', () => showTab(btn.dataset.tab));
});
showTab('tab_overview');

fetch('api/version').then(r => r.json()).then(d => {
  document.getElementById('addon_version_label').textContent = 'v' + d.version;
}).catch(() => { /* im lặng - khong hien version cung khong sao */ });
function dotClassFor(p) {
  if (!p.enabled) return 'off';
  if (!p.device_path) return 'waiting';
  if (p.connected_clients > 0) return 'connected';
  return 'idle';
}

function fmtBytes(n) {
  if (n < 1024) return n + ' B';
  if (n < 1024 * 1024) return (n / 1024).toFixed(1) + ' KB';
  return (n / 1024 / 1024).toFixed(2) + ' MB';
}

function fmtRate(bytesPerSec) {
  if (bytesPerSec < 1) return '0 B/s';
  return fmtBytes(Math.round(bytesPerSec)) + '/s';
}

// So sánh với lần lấy mẫu trước để tính tốc độ TỨC THỜI (bytes/giây) -
// không cần gì phía server, chỉ lưu snapshot trước đó trong JS.
let _prevPortsSnapshot = null;
let _prevPortsSnapshotTs = 0;

// --- Ghi chú tự lưu (dùng chung cho cả 2 bảng: port ảo + USB vật lý) ------
// Nguyên tắc: KHÔNG ghi đè innerHTML của cả bảng nếu đang có 1 ô ghi chú
// trong CHÍNH bảng đó đang được focus (đang gõ dở) - tránh refresh 3s xoá
// mất chữ người dùng chưa gõ xong. Lưu khi rời ô (blur) hoặc bấm Enter.
function escAttr(s) {
  return String(s == null ? '' : s).replace(/&/g, '&amp;').replace(/"/g, '&quot;');
}

function isEditingWithin(containerEl) {
  const active = document.activeElement;
  return !!(active && containerEl.contains(active) && active.classList.contains('note-input'));
}

// Hien displayValue (ngan gon, vd chi ten symlink) nhung nut Copy COPY RA
// copyValue (day du, vd nguyen duong dan /dev/serial/by-id/...) - muc dich
// chinh cua bang USB la lay dung gia tri de dien vao exclude_usb, PHAI la
// duong dan day du moi dung duoc thang, khong the chi co ten symlink ngan.
// copyValue mac dinh = displayValue neu khong truyen rieng (vd cot DEVNAME
// da san la duong dan day du roi, khong can tach 2 gia tri).
function copyCellHtml(displayValue, copyValue) {
  if (!displayValue) return '-';
  const toCopy = copyValue != null ? copyValue : displayValue;
  return `<span style="display:flex; align-items:center; gap:4px;">
    <span style="word-break:break-all;">${displayValue}</span>
    <button type="button" class="copy-value-btn" data-value="${escAttr(toCopy)}"
      title="Copy: ${escAttr(toCopy)}" style="width:auto; margin-top:0; padding:2px 6px; font-size:0.9rem; flex-shrink:0;">📋</button>
  </span>`;
}

function attachCopyButtons(containerEl) {
  containerEl.querySelectorAll('.copy-value-btn').forEach(btn => {
    btn.addEventListener('click', async () => {
      try {
        await navigator.clipboard.writeText(btn.dataset.value);
        const old = btn.textContent;
        btn.textContent = '✅';
        setTimeout(() => { btn.textContent = old; }, 1200);
      } catch (e) { /* im lặng - trinh duyet chan clipboard API (vd http khong phai https) */ }
    });
  });
}

// apiUrl/keyFieldName gan THEO TUNG O (data-api/data-key-field), khong phai
// theo ca bang nua - vi trong CUNG 1 bang "Tat ca cong USB", 1 dong co the
// luu vao port_note (thiet bi dang duoc port ao nhan - dung chung ghi chu
// voi bang port ao) con dong khac luu vao usb_note (thiet bi don le).
function noteCellHtml(apiUrl, keyFieldName, noteKey, noteValue, extraButtonsHtml) {
  const id = 'note_' + btoa(unescape(encodeURIComponent(apiUrl + ':' + noteKey))).replace(/[^a-zA-Z0-9]/g, '');
  return `<td>
    <div style="display:flex; gap:4px; align-items:center;">
      <input type="text" id="${id}" class="note-input"
        data-api="${escAttr(apiUrl)}" data-key-field="${escAttr(keyFieldName)}" data-note-key="${escAttr(noteKey)}"
        value="${escAttr(noteValue)}" placeholder="Ghi chú tự do..."
        style="width:100%; font-size:0.85rem;">
      ${extraButtonsHtml || ''}
    </div>
  </td>`;
}

function attachNoteAutosave(containerEl) {
  containerEl.querySelectorAll('.note-input').forEach(inp => {
    const save = async () => {
      try {
        const body = {note: inp.value};
        body[inp.dataset.keyField] = inp.dataset.noteKey;
        await fetch(inp.dataset.api, {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(body),
        });
      } catch (e) { /* im lặng, thử lại ở lần blur/refresh sau */ }
    };
    inp.addEventListener('blur', save);
    inp.addEventListener('keydown', e => { if (e.key === 'Enter') inp.blur(); });
  });
}

async function loadPorts() {
  const out = document.getElementById('ports_list');
  if (isEditingWithin(out)) return;  // dang go ghi chu dang nay, bo qua refresh
  try {
    const res = await fetch('api/ports');
    const list = await res.json();
    if (!list.length) {
      out.innerHTML = '<i>Chưa có port ảo. Mở tab Cấu hình để thêm port.</i>';
      return;
    }
    const now = Date.now();
    const dt = _prevPortsSnapshotTs ? (now - _prevPortsSnapshotTs) / 1000 : 0;
    const prevByName = {};
    if (_prevPortsSnapshot) {
      for (const p of _prevPortsSnapshot) prevByName[p.name] = p;
    }

    let html = '<table class="ports"><tr><th></th><th>Tên</th><th>Protocol</th>'
      + '<th>Cổng kết nối</th><th>Thiết bị</th><th>Trạng thái</th>'
      + '<th>Đã nhận (serial&rarr;client)</th><th>Đã gửi (client&rarr;serial)</th>'
      + '<th>Số lần đọc/ghi</th><th>Tốc độ hiện tại</th><th></th><th>Ghi chú</th></tr>';
    for (const p of list) {
      const target = p.output_mode === 'pty'
        ? (p.pty_symlink || '(pty)')
        : (p.tcp_port ? (':' + p.tcp_port) : '');

      let rateTxt = '-';
      const prev = prevByName[p.name];
      if (prev && dt > 0.5) {
        const deltaBytes = (p.bytes_read - prev.bytes_read) + (p.bytes_written - prev.bytes_written);
        rateTxt = fmtRate(deltaBytes / dt);
      }

      // Nut Rescan CHI hien cho port ao dang bat (co thread that dang chay) -
      // port dang enabled=false khong co gi de rescan.
      const rescanBtn = p.enabled
        ? `<button type="button" class="rescan-btn" data-port-name="${escAttr(p.name)}"
            title="Dò lại thiết bị ngay, không cần đợi hết chu kỳ rescan_interval_s"
            style="width:auto; margin-top:0; padding:4px 8px; font-size:0.8rem;">🔄 Rescan</button>`
        : '';

      html += `<tr>
        <td><span class="dot ${dotClassFor(p)}"></span></td>
        <td>${p.name}</td>
        <td>${p.protocol || ''}</td>
        <td>${target}</td>
        <td>${p.device_path || '-'}</td>
        <td>${p.state}</td>
        <td>${fmtBytes(p.bytes_read)}</td>
        <td>${fmtBytes(p.bytes_written)}</td>
        <td>${p.messages_read} / ${p.messages_written}</td>
        <td>${rateTxt}</td>
        <td>${rescanBtn}</td>
        ${noteCellHtml('api/port_note', 'name', p.name, p.note || '', '')}
      </tr>`;
    }
    html += '</table>';
    out.innerHTML = html;
    attachNoteAutosave(out);
    out.querySelectorAll('.rescan-btn').forEach(btn => {
      btn.addEventListener('click', () => rescanPort(btn));
    });
    _prevPortsSnapshot = list;
    _prevPortsSnapshotTs = now;
  } catch (e) {
    out.innerHTML = '<i>Không tải được danh sách port ảo.</i>';
  }
}
async function rescanPort(btn) {
  const name = btn.dataset.portName;
  const oldText = btn.textContent;
  btn.disabled = true;
  btn.textContent = '...';
  try {
    const res = await fetch('api/rescan_port', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({name: name}),
    });
    const result = await res.json();
    if (!result.ok) {
      alert('Không rescan được: ' + (result.error || 'lỗi không rõ'));
    }
  } catch (e) {
    alert('Lỗi kết nối khi gọi rescan.');
  }
  setTimeout(() => { btn.disabled = false; btn.textContent = oldText; }, 1500);
}
loadPorts();
setInterval(loadPorts, 3000);

async function loadUsbDevices() {
  const out = document.getElementById('usb_devices_list');
  if (isEditingWithin(out)) return;
  try {
    const res = await fetch('api/usb_devices');
    const list = await res.json();
    if (!list.length) {
      out.innerHTML = '<i>Không tìm thấy cổng USB nào khớp scan_glob, và chưa có ghi chú nào.</i>';
      return;
    }
    let html = '<table class="ports"><tr><th>DEVNAME</th><th>by-id</th><th>by-path</th>'
      + '<th>VID:PID</th><th>ID_SERIAL</th><th>Manufacturer / Product</th>'
      + '<th>ID_USB_DRIVER</th><th>Trạng thái</th><th>Ghi chú</th></tr>';
    for (const d of list) {
      let statusTxt;
      let rowStyle = '';
      if (d.unavailable) {
        statusTxt = 'Không còn cắm (unavailable)';
        if (d.last_seen) statusTxt += ' — thấy lần cuối ' + new Date(d.last_seen * 1000).toLocaleString('vi-VN');
        rowStyle = ' style="opacity:0.55;"';
      } else if (d.claimed_by) {
        // note_store === 'port' cho truong hop nay - ghi chu dung CHUNG voi
        // bang "Danh sach port ao" (khoa = ten port ao), khong phai ghi chu
        // rieng cua thiet bi USB - ghi ro de nguoi dung khong tuong nham 2
        // ghi chu doc lap.
        statusTxt = `Đang dùng bởi "${d.claimed_by}" (ghi chú dùng chung với bảng port ảo)`;
      } else if (d.excluded) {
        statusTxt = 'Loại trừ (exclude_usb)';
      } else {
        statusTxt = 'Chưa dùng';
      }
      // Nut Xoa CHI hien cho thiet bi "Khong con cam" (unavailable) - thiet
      // bi con dang cam (du dang dung boi port ao hay tu do/loai tru) KHONG
      // can nut Xoa, sua truc tiep o ← noi dung ghi chu la du (rong = tu
      // dong xoa nho _save_note()). Chi hien nut khi that su la "lich su"
      // can don dep, tranh nut rac o nhung dong dang con y nghia.
      // data-note-key (khong phai onclick nhung JS) - onclick="...JSON.stringify..."
      // nhung truoc day sinh dau nhay kep NGAY TRONG thuoc tinh HTML, gay gay
      // cu phap (thuoc tinh onclick bi cat cut o dau nhay kep dau tien) - nut
      // Xoa vi vay khong bao gio chay duoc. Dung data-attribute + addEventListener
      // (gan trong attachDeleteButtons ben duoi) de tranh hoan toan van de escape.
      const deleteBtn = d.unavailable
        ? `<button type="button" class="delete-note-btn" data-note-key="${escAttr(d.note_key)}"
            title="Xoá ghi chú/lịch sử thiết bị này"
            style="width:auto; margin-top:0; padding:6px 8px; background:#4a1f1f; color:#ff8080;"
            >Xoá</button>`
        : '';
      const noteApi = d.note_store === 'port' ? 'api/port_note' : 'api/usb_note';
      const noteKeyField = d.note_store === 'port' ? 'name' : 'key';
      const vidPid = (d.id_vendor || d.id_product) ? `${d.id_vendor || '?'}:${d.id_product || '?'}` : '-';
      const manuProduct = (d.manufacturer || d.product)
        ? [d.manufacturer, d.product].filter(Boolean).join(' / ')
        : '-';
      html += `<tr${rowStyle}>
        <td>${copyCellHtml(d.devname)}</td>
        <td>${copyCellHtml(d.by_id, d.by_id ? ('/dev/serial/by-id/' + d.by_id) : null)}</td>
        <td>${copyCellHtml(d.by_path, d.by_path ? ('/dev/serial/by-path/' + d.by_path) : null)}</td>
        <td>${vidPid}</td>
        <td>${d.id_serial || '-'}</td>
        <td>${manuProduct}</td>
        <td>${d.id_usb_driver || '-'}</td>
        <td>${statusTxt}</td>
        ${noteCellHtml(noteApi, noteKeyField, d.note_key, d.note || '', deleteBtn)}
      </tr>`;
    }
    html += '</table>';
    out.innerHTML = html;
    attachNoteAutosave(out);
    attachCopyButtons(out);
    out.querySelectorAll('.delete-note-btn').forEach(btn => {
      btn.addEventListener('click', () => deleteUsbNote(btn.dataset.noteKey));
    });
  } catch (e) {
    out.innerHTML = '<i>Không tải được danh sách USB.</i>';
  }
}

async function deleteUsbNote(key) {
  if (!confirm('Xoá ghi chú/lịch sử của thiết bị này?')) return;
  try {
    await fetch('api/usb_forget', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({key: key}),
    });
  } catch (e) { /* im lặng */ }
  loadUsbDevices();
}
loadUsbDevices();
setInterval(loadUsbDevices, 3000);

let _candidateList = [];

function updateHoldBoxes() {
  const tabs = [
    ['gr', 'device', 'gr_hold_box', 'gr_busy_box'],
    ['comm', 'comm_device', 'comm_hold_box', 'comm_busy_box'],
  ];
  for (const [tab, selId, boxId, wrapId] of tabs) {
    const wrap = document.getElementById(wrapId);
    if (!wrap) continue;
    wrap.style.display = isLocalPcMode(tab) ? 'none' : 'block';
    const dev = document.getElementById(selId).value;
    const item = _candidateList.find(i => i.device === dev);
    let html = '';
    if (item && item.held_by) {
      html = `<div class="hold-on"><span>⏸ Đang giữ quyền test — port ảo "${escAttr(item.held_by)}" tạm ngắt, Node-RED/HA mất kết nối tới thiết bị này (tự trả sau 30 phút).</span>
        <button type="button" class="hold-btn release" data-act="release" data-name="${escAttr(item.held_by)}">↩ Trả quyền cho port ảo</button></div>`;
    } else if (item && item.claimed_by) {
      html = `<div class="hold-off"><span>Cổng đang được port ảo "${escAttr(item.claimed_by)}" ghim.</span>
        <button type="button" class="hold-btn take" data-act="take" data-name="${escAttr(item.claimed_by)}">⛔ Lấy quyền port ảo để test</button></div>`;
    }
    const box = document.getElementById(boxId);
    if (box.dataset.html !== html) {
      box.innerHTML = html;
      box.dataset.html = html;
    }
  }
}

document.addEventListener('click', async (ev) => {
  const btn = ev.target.closest('.hold-btn');
  if (!btn) return;
  const name = btn.dataset.name;
  const take = btn.dataset.act === 'take';
  if (take && !confirm(`Port ảo "${name}" sẽ ngắt bridge và nhả cổng USB để test. Node-RED/HA mất kết nối tới thiết bị cho tới khi bấm "Trả quyền" (tự trả sau 30 phút). Tiếp tục?`)) return;
  btn.disabled = true;
  try {
    const res = await fetch(take ? 'api/take_test_hold' : 'api/release_test_hold', {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({ name }),
    });
    const d = await res.json();
    if (!d.ok) alert('Lỗi: ' + d.error);
    else if (d.warning) alert(d.warning);
  } catch (e) {
    alert('Lỗi kết nối: ' + (e.message || e));
  }
  await loadCandidates();
});

async function loadCandidates() {
  // Cap nhat MOI select co class "device-select" (#device o tab Get Response
  // VA #comm_device o tab Communication) tu CHUNG 1 lan goi API - tranh 2
  // vong polling rieng biet lam trung lap request.
  const selects = document.querySelectorAll('.device-select');
  if (!selects.length) return;
  try {
    const res = await fetch('api/candidates');
    const list = await res.json();
    selects.forEach(sel => {
      const prev = sel.value;
      sel.innerHTML = '';
      for (const item of list) {
        const opt = document.createElement('option');
        opt.value = item.device;
        let suffix = '';
        if (item.excluded) suffix += '  exclude_usb';
        if (item.claimed_by) suffix += `  (port ảo "${item.claimed_by}")`;
        if (item.held_by) suffix += `  (⏸ đang giữ quyền test từ "${item.held_by}")`;
        if (item.by_id) suffix += `  — ${item.by_id}`;
        // Hien ghi chu nguoi dung da luu (bang USB Manager) ngay trong
        // dropdown - ap dung cho MOI thiet bi co ghi chu, khong chi loai
        // tru: dang dung boi port ao (ghi chu dung chung), da loai tru, hay
        // con tu do chua dung toi.
        if (item.note) suffix += `  — 📝 ${item.note}`;
        opt.textContent = item.device + suffix;
        sel.appendChild(opt);
      }
      if (list.some(i => i.device === prev)) sel.value = prev;
    });
    _candidateList = list;
    updateHoldBoxes();
  } catch (e) { /* im lặng, thử lại lần sau */ }
}
loadCandidates();
setInterval(loadCandidates, 3000);

async function loadMqttStatus() {
  const out = document.getElementById('mqtt_status');
  try {
    const res = await fetch('api/mqtt_status');
    const s = await res.json();
    if (s.connected) {
      out.innerHTML = `🟢 MQTT: Đã kết nối (${s.host}:${s.port})`;
      out.style.color = '#3fb950';
    } else if (s.configured) {
      out.innerHTML = `🔴 MQTT: Mất kết nối tới ${s.host}:${s.port} - entity HA sẽ không cập nhật`;
      out.style.color = '#ff6b6b';
    } else {
      out.innerHTML = '⚪ MQTT: Chưa cấu hình (điền MQTT host trong tab Cấu hình nếu muốn tạo entity HA)';
      out.style.color = '#9aa0a6';
    }
  } catch (e) {
    out.innerHTML = '⚪ MQTT: Không kiểm tra được trạng thái';
  }
}
loadMqttStatus();
setInterval(loadMqttStatus, 5000);

function toggleProtocolFields() {
  const isModbus = document.getElementById('protocol').value === 'modbus_rtu';
  document.getElementById('raw_fields').style.display = isModbus ? 'none' : 'block';
  document.getElementById('modbus_fields').style.display = isModbus ? 'block' : 'none';
}
document.getElementById('protocol').addEventListener('change', toggleProtocolFields);
toggleProtocolFields();

function toggleSpyOnly() {
  const spyOnly = document.getElementById('spy_only').checked;
  document.getElementById('send_section').style.display = spyOnly ? 'none' : 'block';
  document.getElementById('btn').textContent = spyOnly ? 'Nghe (Spy)' : 'Gửi lệnh & xem phản hồi';
}
document.getElementById('spy_only').addEventListener('change', toggleSpyOnly);
toggleSpyOnly();

function hexField(label, hex, display) {
  if (hex === undefined) return '';
  const id = 'copy_' + Math.random().toString(36).slice(2);
  return `<label>${label}</label>
    <pre>${display}</pre>
    <div class="copybox">
      <input id="${id}" readonly value="hex:${hex}">
      <button type="button" onclick="navigator.clipboard.writeText(document.getElementById('${id}').value)">Copy</button>
    </div>`;
}

// ============================================================
// "Local PC" - dung Web Serial API (navigator.serial) de noi TRUC TIEP toi
// 1 thiet bi USB cam ngay tren MAY DANG MO TRANG NAY (khong qua server add-on
// chut nao) - giong het co che ESPHome Web Installer dung de flash firmware
// tu trinh duyet. CHI chay duoc tren Chrome/Edge/Opera (KHONG Firefox/Safari)
// VA chi trong "secure context" (HTTPS hoac localhost) - trinh duyet tu chan
// hoan toan neu thieu 1 trong 2 dieu kien, khong phai loi code o day.
//
// Vi trinh duyet KHONG the goi nguoc lai Python server de tinh CRC/build
// khung Modbus, toan bo logic giao thuc (CRC16, build frame, kiem CRC,
// decode hex:/text:) duoc PORT LAI trong JS o day - da doi chieu ket qua
// voi response THAT bat duoc tu hardware log trong qua trinh phat trien
// (xem TROUBLESHOOTING_NOTES.md) de dam bao khop 100% voi ban Python goc.
// ============================================================

function hexToBytesLocal(hexStr) {
  const clean = hexStr.replace(/\\s+/g, '');
  const bytes = new Uint8Array(clean.length / 2);
  for (let i = 0; i < clean.length; i += 2) {
    bytes[i / 2] = parseInt(clean.substr(i, 2), 16);
  }
  return bytes;
}

function bytesToHexLocal(bytes) {
  return Array.from(bytes).map(b => b.toString(16).padStart(2, '0')).join('');
}

function describeBytesLocal(bytes) {
  let text = '';
  for (const b of bytes) text += (b >= 32 && b < 127) ? String.fromCharCode(b) : '.';
  return `text: ${JSON.stringify(text)} | hex: ${bytesToHexLocal(bytes)}`;
}

function decodeCommandLocal(value) {
  if (!value) return new Uint8Array(0);
  if (value.startsWith('hex:')) return hexToBytesLocal(value.slice(4).trim());
  if (value.startsWith('text:')) {
    let text = value.slice(5);
    text = text.replace(/\\\\r\\\\n/g, '\\r\\n').replace(/\\\\n/g, '\\n').replace(/\\\\t/g, '\\t');
    return new TextEncoder().encode(text);
  }
  return new TextEncoder().encode(value);
}

function modbusCrc16Local(bytes) {
  let crc = 0xFFFF;
  for (let i = 0; i < bytes.length; i++) {
    crc ^= bytes[i];
    for (let j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return new Uint8Array([crc & 0xFF, (crc >> 8) & 0xFF]);
}

function buildModbusRequestLocal(unitId, functionCode, startAddress, quantity, value) {
  const body = new Uint8Array(6);
  body[0] = unitId; body[1] = functionCode;
  body[2] = (startAddress >> 8) & 0xFF; body[3] = startAddress & 0xFF;
  if (functionCode === 5) {
    const coilValue = value ? 0xFF00 : 0x0000;
    body[4] = (coilValue >> 8) & 0xFF; body[5] = coilValue & 0xFF;
  } else if (functionCode === 6) {
    const v = value || 0;
    body[4] = (v >> 8) & 0xFF; body[5] = v & 0xFF;
  } else {
    body[4] = (quantity >> 8) & 0xFF; body[5] = quantity & 0xFF;
  }
  const crc = modbusCrc16Local(body);
  const out = new Uint8Array(8);
  out.set(body, 0);
  out.set(crc, 6);
  return out;
}

// Port tu check_modbus_response() Python - dung cho ca quet Unit ID/thanh
// ghi khi chay Local PC (khong the goi API server de kiem tra).
function checkModbusResponseLocal(unitId, functionCode, response) {
  if (response.length < 5) return false;
  const payload = response.slice(0, -2);
  const recvCrc = response.slice(-2);
  const calcCrc = modbusCrc16Local(payload);
  if (calcCrc[0] !== recvCrc[0] || calcCrc[1] !== recvCrc[1]) return false;
  if (response[0] !== unitId) return false;
  if (response[1] !== functionCode && response[1] !== (functionCode | 0x80)) return false;
  return true;
}

let _localPort = null;

function localPcSupported() {
  return 'serial' in navigator && window.isSecureContext;
}

async function localSerialConnect(baud) {
  if (!('serial' in navigator)) {
    throw new Error('Trình duyệt không hỗ trợ Web Serial API - chỉ Chrome/Edge/Opera mới có, Firefox/Safari không hỗ trợ.');
  }
  if (!window.isSecureContext) {
    throw new Error('Trang này chưa chạy qua HTTPS (hoặc localhost) - trình duyệt tự chặn Web Serial API trên HTTP thường.');
  }
  const port = await navigator.serial.requestPort();
  await port.open({ baudRate: baud });
  _localPort = port;
  return port;
}

async function localSerialDisconnect() {
  if (_localPort) {
    try { await _localPort.close(); } catch (e) { /* im lặng */ }
    _localPort = null;
  }
}

// Doc lien tuc trong listenMs mili-giay, KHONG ghi gi ca - dung cho ca
// "nghe thu dong" lan doc phan hoi sau khi gui.
async function localSerialListen(listenMs) {
  if (!_localPort || !_localPort.readable) return new Uint8Array(0);
  const reader = _localPort.readable.getReader();
  const chunks = [];
  const deadline = Date.now() + listenMs;
  try {
    while (Date.now() < deadline) {
      const remaining = deadline - Date.now();
      if (remaining <= 0) break;
      const result = await Promise.race([
        reader.read(),
        new Promise(resolve => setTimeout(() => resolve({ timeout: true }), remaining)),
      ]);
      if (result.timeout || result.done) break;
      if (result.value) chunks.push(result.value);
    }
  } finally {
    reader.releaseLock();
  }
  const total = chunks.reduce((n, c) => n + c.length, 0);
  const out = new Uint8Array(total);
  let offset = 0;
  for (const c of chunks) { out.set(c, offset); offset += c.length; }
  return out;
}

async function localSerialWrite(bytes) {
  if (!_localPort || !_localPort.writable) throw new Error('Chưa kết nối cổng USB local nào.');
  const writer = _localPort.writable.getWriter();
  try {
    await writer.write(bytes);
  } finally {
    writer.releaseLock();
  }
}

// Mo phong y het _spy_probe_port() phia Python - tra ve CUNG 1 cau truc
// object (sent_hex/passive_hex/active_hex/...) de dung lai NGUYEN VEN code
// hien thi ket qua da co (hexField(), khong can viet lai).
async function localProbe({ baud, listenS, spyOnly, protocol, sendCommand, unitId, functionCode, startAddress, quantity, value }) {
  const result = { device: '(Local PC)', baud, busy: false };
  const passiveBytes = await localSerialListen(listenS * 1000);
  result.passive_hex = bytesToHexLocal(passiveBytes);
  result.passive_display = describeBytesLocal(passiveBytes);
  if (spyOnly) return result;

  let frame;
  if (protocol === 'modbus_rtu') {
    frame = buildModbusRequestLocal(unitId, functionCode, startAddress, quantity, value);
  } else if (sendCommand) {
    frame = decodeCommandLocal(sendCommand);
  }
  if (!frame || !frame.length) return result;

  result.sent_hex = bytesToHexLocal(frame);
  result.sent_display = describeBytesLocal(frame);
  await localSerialWrite(frame);
  const activeBytes = await localSerialListen(1000);
  result.active_hex = bytesToHexLocal(activeBytes);
  result.active_display = describeBytesLocal(activeBytes);
  return result;
}

// Port tu _modbus_scan_units()/_modbus_scan_registers() phia Python - dung
// khi chay Local PC (khong the goi API server). KHONG can kiem tra "cong
// dang ban" nhu ban Python (vi Web Serial da giu doc quyen port ngay tu
// luc navigator.serial.requestPort()+open() thanh cong, khong tien trinh
// nao khac tren may nay co the mo chung 1 luc).
async function localScanUnits({ unitStart, unitEnd, functionCode, startAddress, quantity, timeoutS }) {
  const found = [];
  for (let uid = unitStart; uid <= unitEnd; uid++) {
    const frame = buildModbusRequestLocal(uid, functionCode, startAddress, quantity, null);
    await localSerialWrite(frame);
    const resp = await localSerialListen(timeoutS * 1000);
    if (checkModbusResponseLocal(uid, functionCode, resp)) {
      found.push({ unit_id: uid, response_hex: bytesToHexLocal(resp) });
    }
  }
  return { found, scanned: unitEnd - unitStart + 1 };
}

async function localScanRegisters({ unitId, functionCodes, addrStart, addrEnd, quantity, timeoutS }) {
  const found = [];
  for (const fc of functionCodes) {
    for (let addr = addrStart; addr <= addrEnd; addr++) {
      const frame = buildModbusRequestLocal(unitId, fc, addr, quantity, null);
      await localSerialWrite(frame);
      const resp = await localSerialListen(timeoutS * 1000);
      if (checkModbusResponseLocal(unitId, fc, resp)) {
        found.push({ function_code: fc, address: addr, response_hex: bytesToHexLocal(resp) });
      }
    }
  }
  return { found, scanned: (addrEnd - addrStart + 1) * functionCodes.length };
}

// --- Ghi chu rieng cho thiet bi Local PC - luu HOAN TOAN trong trinh duyet
// (localStorage), KHONG gui len server, vi day la thiet bi server khong he
// biet toi. Khoa theo vendorId:productId (lay tu port.getInfo() cua Web
// Serial) neu co, fallback ve khoa chung "unknown" neu trinh duyet khong
// cho biet (hiem).
const LOCAL_PC_NOTE_PREFIX = 'usbmgr_local_pc_note_';

function localPcNoteKey(port) {
  try {
    const info = port.getInfo ? port.getInfo() : {};
    if (info.usbVendorId != null && info.usbProductId != null) {
      return `${info.usbVendorId}:${info.usbProductId}`;
    }
  } catch (e) { /* im lặng */ }
  return 'unknown';
}

function loadLocalPcNote(key) {
  try { return localStorage.getItem(LOCAL_PC_NOTE_PREFIX + key) || ''; } catch (e) { return ''; }
}

function saveLocalPcNote(key, note) {
  try { localStorage.setItem(LOCAL_PC_NOTE_PREFIX + key, note); } catch (e) { /* im lặng - localStorage co the bi chan */ }
}

// Get Response VA Communication dung CHUNG 1 ket noi Local PC (_localPort
// toan cuc, xem dinh nghia o tren) - chon 1 lan o tab nao cung dung duoc o
// tab kia, khong phai chon lai/hien popup requestPort() lan thu 2. Moi ham
// duoi day cap nhat DONG THOI trang thai hien thi + gia tri ghi chu tren
// CA 2 tab de luon dong bo, tranh nham lan "da ket noi chua".
function updateGrSourceUI() {
  const isLocal = document.getElementById('gr_source_local').checked;
  document.getElementById('local_pc_section').style.display = isLocal ? 'block' : 'none';
  document.getElementById('device_label').style.display = isLocal ? 'none' : 'block';
  document.getElementById('device').style.display = isLocal ? 'none' : 'block';
}
document.getElementById('gr_source_server').addEventListener('change', updateGrSourceUI);
document.getElementById('gr_source_local').addEventListener('change', updateGrSourceUI);
document.getElementById('gr_source_server').addEventListener('change', updateHoldBoxes);
document.getElementById('gr_source_local').addEventListener('change', updateHoldBoxes);
document.getElementById('device').addEventListener('change', updateHoldBoxes);
updateGrSourceUI();

function updateCommSourceUI() {
  const isLocal = document.getElementById('comm_source_local').checked;
  document.getElementById('comm_local_pc_section').style.display = isLocal ? 'block' : 'none';
  document.getElementById('comm_device_row').style.display = isLocal ? 'none' : 'flex';
}
document.getElementById('comm_source_server').addEventListener('change', updateCommSourceUI);
document.getElementById('comm_source_local').addEventListener('change', updateCommSourceUI);
document.getElementById('comm_source_server').addEventListener('change', updateHoldBoxes);
document.getElementById('comm_source_local').addEventListener('change', updateHoldBoxes);
document.getElementById('comm_device').addEventListener('change', updateHoldBoxes);
updateCommSourceUI();

let _localPcNoteKey = null;

function isLocalPcMode(tab) {
  const id = tab === 'comm' ? 'comm_source_local' : 'gr_source_local';
  return document.getElementById(id).checked;
}

function updateAllLocalPcStatus(html) {
  const el1 = document.getElementById('local_pc_status');
  const el2 = document.getElementById('comm_local_pc_status');
  if (el1) el1.innerHTML = html;
  if (el2) el2.innerHTML = html;
}

function syncLocalPcNoteInputs(value) {
  const el1 = document.getElementById('local_pc_note');
  const el2 = document.getElementById('comm_local_pc_note');
  if (el1) el1.value = value;
  if (el2) el2.value = value;
}

async function connectLocalPcShared(baud) {
  if (!localPcSupported()) {
    updateAllLocalPcStatus(!('serial' in navigator)
      ? '<span style="color:#ff6b6b;">Trình duyệt không hỗ trợ Web Serial API (chỉ Chrome/Edge/Opera).</span>'
      : '<span style="color:#ff6b6b;">Cần truy cập qua HTTPS (hoặc localhost) - trang hiện KHÔNG ở secure context.</span>');
    return;
  }
  if (_localPort) {
    const info = _localPort.getInfo ? _localPort.getInfo() : {};
    updateAllLocalPcStatus(`✅ Đã kết nối (VID:PID = ${info.usbVendorId ?? '?'}:${info.usbProductId ?? '?'}) - bấm "Ngắt kết nối" nếu muốn đổi thiết bị khác`);
    return;
  }
  try {
    const port = await localSerialConnect(baud);
    const info = port.getInfo ? port.getInfo() : {};
    _localPcNoteKey = localPcNoteKey(port);
    syncLocalPcNoteInputs(loadLocalPcNote(_localPcNoteKey));
    updateAllLocalPcStatus(`✅ Đã kết nối (VID:PID = ${info.usbVendorId ?? '?'}:${info.usbProductId ?? '?'})`);
  } catch (e) {
    updateAllLocalPcStatus(`<span style="color:#ff6b6b;">Lỗi: ${e.message || e}</span>`);
  }
}

document.getElementById('local_connect_btn').addEventListener('click', () => {
  connectLocalPcShared(parseInt(document.getElementById('baud').value, 10));
});
document.getElementById('comm_local_connect_btn').addEventListener('click', () => {
  connectLocalPcShared(parseInt(document.getElementById('comm_baud').value, 10));
});

async function disconnectLocalPcShared() {
  await localSerialDisconnect();
  _localPcNoteKey = null;
  syncLocalPcNoteInputs('');
  updateAllLocalPcStatus('Chưa kết nối.');
}
document.getElementById('local_disconnect_btn').addEventListener('click', disconnectLocalPcShared);
document.getElementById('comm_local_disconnect_btn').addEventListener('click', disconnectLocalPcShared);

document.getElementById('local_pc_note').addEventListener('blur', () => {
  const value = document.getElementById('local_pc_note').value;
  if (_localPcNoteKey) saveLocalPcNote(_localPcNoteKey, value);
  syncLocalPcNoteInputs(value);
});
document.getElementById('comm_local_pc_note').addEventListener('blur', () => {
  const value = document.getElementById('comm_local_pc_note').value;
  if (_localPcNoteKey) saveLocalPcNote(_localPcNoteKey, value);
  syncLocalPcNoteInputs(value);
});

document.getElementById('btn').addEventListener('click', async () => {
  const btn = document.getElementById('btn');
  const out = document.getElementById('result');
  btn.disabled = true;
  out.innerHTML = 'Đang thực hiện...';
  try {
    const spyOnly = document.getElementById('spy_only').checked;
    const isLocal = document.getElementById('gr_source_local').checked;
    let data;

    if (isLocal) {
      if (!_localPort) throw new Error('Chưa kết nối cổng USB Local PC nào - bấm "Chọn cổng USB trên máy này" trước.');
      const params = {
        baud: parseInt(document.getElementById('baud').value, 10),
        listenS: parseFloat(document.getElementById('listen_s').value),
        spyOnly,
      };
      if (!spyOnly) {
        params.protocol = document.getElementById('protocol').value;
        if (params.protocol === 'modbus_rtu') {
          params.unitId = parseInt(document.getElementById('unit_id').value, 10);
          params.functionCode = parseInt(document.getElementById('function_code').value, 10);
          params.startAddress = parseInt(document.getElementById('start_address').value || '0', 10);
          params.quantity = parseInt(document.getElementById('quantity').value || '1', 10);
          const v = document.getElementById('value').value;
          params.value = v === '' ? null : parseInt(v, 10);
        } else {
          params.sendCommand = document.getElementById('send_command').value || null;
        }
      }
      data = await localProbe(params);
    } else {
    const body = {
      device: document.getElementById('device').value,
      baud: parseInt(document.getElementById('baud').value, 10),
      listen_s: parseFloat(document.getElementById('listen_s').value),
      spy_only: spyOnly,
      share: document.getElementById('gr_share').checked,
    };
    if (!spyOnly) {
      body.protocol = document.getElementById('protocol').value;
      if (body.protocol === 'modbus_rtu') {
        body.unit_id = parseInt(document.getElementById('unit_id').value, 10);
        body.function_code = parseInt(document.getElementById('function_code').value, 10);
        body.start_address = parseInt(document.getElementById('start_address').value || '0', 10);
        body.quantity = parseInt(document.getElementById('quantity').value || '1', 10);
        const v = document.getElementById('value').value;
        body.value = v === '' ? null : parseInt(v, 10);
      } else {
        body.send_command = document.getElementById('send_command').value || null;
      }
    }
    const res = await fetch('api/probe', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(body),
    });
    data = await res.json();
    }
    let html = '';
    if (data.error) html += `<div class="err">Lỗi: ${data.error}</div>`;
    if (data.note) html += `<div class="note">${data.note}</div>`;
    if (data.busy) html += `<span class="badge claimed">Port đang bận</span>`;
    html += hexField('Đã gửi:', data.sent_hex, data.sent_display);
    html += hexField('Nghe được (passive, trước khi gửi):', data.passive_hex, data.passive_display);
    html += hexField('Phản hồi (sau khi gửi):', data.active_hex, data.active_display);
    out.innerHTML = html || '<i>Không nhận được byte nào.</i>';
  } catch (e) {
    out.innerHTML = `<div class="err">Lỗi kết nối: ${e.message || e}</div>`;
  } finally {
    btn.disabled = false;
  }
});

// ============================================================
// Tab Communication - quet Unit ID / quet thanh ghi / doi dia chi / gui lap
// ============================================================

function commDeviceBaud() {
  return {
    device: document.getElementById('comm_device').value,
    baud: parseInt(document.getElementById('comm_baud').value, 10),
    share: document.getElementById('comm_share').checked,
  };
}

document.getElementById('scan_unit_btn').addEventListener('click', async () => {
  const btn = document.getElementById('scan_unit_btn');
  const out = document.getElementById('scan_unit_result');
  const oldText = btn.textContent;
  btn.disabled = true;
  btn.textContent = 'Đang quét... (có thể mất vài chục giây)';
  out.innerHTML = '';
  const params = {
    unit_start: parseInt(document.getElementById('scan_unit_start').value, 10),
    unit_end: parseInt(document.getElementById('scan_unit_end').value, 10),
    function_code: parseInt(document.getElementById('scan_unit_fc').value, 10),
    start_address: parseInt(document.getElementById('scan_unit_addr').value, 10),
    quantity: parseInt(document.getElementById('scan_unit_quantity').value, 10) || 1,
    timeout_s: parseFloat(document.getElementById('scan_unit_timeout').value) || 0.3,
  };
  try {
    let data;
    if (isLocalPcMode('comm')) {
      if (!_localPort) throw new Error('Chưa kết nối cổng USB Local PC nào - bấm "Chọn cổng USB trên máy này" trước.');
      data = await localScanUnits({
        unitStart: params.unit_start, unitEnd: params.unit_end,
        functionCode: params.function_code, startAddress: params.start_address,
        quantity: params.quantity, timeoutS: params.timeout_s,
      });
    } else {
      const body = Object.assign(commDeviceBaud(), params);
      const res = await fetch('api/modbus_scan_units', {
        method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
      });
      data = await res.json();
    }
    if (data.error) {
      out.innerHTML = `<div class="err">Lỗi: ${data.error}</div>`;
    } else if (!data.found.length) {
      out.innerHTML = `<i>Không tìm thấy thiết bị nào phản hồi (đã quét ${data.scanned} địa chỉ ${params.unit_start}-${params.unit_end}).</i>`;
    } else {
      let html = data.note ? `<div class="note">${data.note}</div>` : '';
      html += `<p>Tìm thấy ${data.found.length}/${data.scanned} địa chỉ có phản hồi hợp lệ:</p>`;
      html += '<table class="ports"><tr><th>Unit ID</th><th>Phản hồi (hex)</th></tr>';
      for (const f of data.found) {
        html += `<tr><td>${f.unit_id}</td><td style="word-break:break-all;">${f.response_hex}</td></tr>`;
      }
      html += '</table>';
      out.innerHTML = html;
    }
  } catch (e) {
    out.innerHTML = `<div class="err">Lỗi kết nối: ${e.message || e}</div>`;
  } finally {
    btn.disabled = false;
    btn.textContent = oldText;
    persistCommState();
  }
});

document.getElementById('scan_reg_btn').addEventListener('click', async () => {
  const btn = document.getElementById('scan_reg_btn');
  const out = document.getElementById('scan_reg_result');
  const oldText = btn.textContent;
  btn.disabled = true;
  btn.textContent = 'Đang quét... (có thể mất vài chục giây)';
  out.innerHTML = '';
  const fcList = document.getElementById('scan_reg_fc').value
    .split(',').map(s => parseInt(s.trim(), 10)).filter(n => !isNaN(n));
  const params = {
    unit_id: parseInt(document.getElementById('scan_reg_unit').value, 10),
    function_codes: fcList,
    addr_start: parseInt(document.getElementById('scan_reg_start').value, 10),
    addr_end: parseInt(document.getElementById('scan_reg_end').value, 10),
    quantity: parseInt(document.getElementById('scan_reg_quantity').value, 10) || 1,
    timeout_s: parseFloat(document.getElementById('scan_reg_timeout').value) || 0.3,
  };
  try {
    let data;
    if (isLocalPcMode('comm')) {
      if (!_localPort) throw new Error('Chưa kết nối cổng USB Local PC nào - bấm "Chọn cổng USB trên máy này" trước.');
      data = await localScanRegisters({
        unitId: params.unit_id, functionCodes: params.function_codes,
        addrStart: params.addr_start, addrEnd: params.addr_end,
        quantity: params.quantity, timeoutS: params.timeout_s,
      });
    } else {
      const body = Object.assign(commDeviceBaud(), params);
      const res = await fetch('api/modbus_scan_registers', {
        method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
      });
      data = await res.json();
    }
    if (data.error) {
      out.innerHTML = `<div class="err">Lỗi: ${data.error}</div>`;
    } else if (!data.found.length) {
      out.innerHTML = `<i>Không có thanh ghi nào phản hồi hợp lệ (đã quét ${data.scanned} tổ hợp FC/address).</i>`;
    } else {
      let html = data.note ? `<div class="note">${data.note}</div>` : '';
      html += `<p>Tìm thấy ${data.found.length}/${data.scanned} tổ hợp có phản hồi hợp lệ:</p>`;
      html += '<table class="ports"><tr><th>Function code</th><th>Address</th><th>Phản hồi (hex)</th></tr>';
      for (const f of data.found) {
        html += `<tr><td>${f.function_code}</td><td>${f.address}</td><td style="word-break:break-all;">${f.response_hex}</td></tr>`;
      }
      html += '</table>';
      out.innerHTML = html;
    }
  } catch (e) {
    out.innerHTML = `<div class="err">Lỗi kết nối: ${e.message || e}</div>`;
  } finally {
    btn.disabled = false;
    btn.textContent = oldText;
    persistCommState();
  }
});

document.getElementById('setaddr_btn').addEventListener('click', async () => {
  const btn = document.getElementById('setaddr_btn');
  const out = document.getElementById('setaddr_result');
  if (!confirm('Sắp gửi lệnh ghi thanh ghi lên thiết bị THẬT - chắc chắn đúng Unit ID/address/giá trị?')) return;
  btn.disabled = true;
  out.innerHTML = '';
  const params = {
    listenS: parseFloat(document.getElementById('setaddr_listen').value) || 0.3,
    spyOnly: false,
    protocol: 'modbus_rtu',
    unitId: parseInt(document.getElementById('setaddr_unit').value, 10),
    functionCode: parseInt(document.getElementById('setaddr_fc').value, 10),
    startAddress: parseInt(document.getElementById('setaddr_reg').value, 10),
    value: parseInt(document.getElementById('setaddr_value').value, 10),
  };
  try {
    let data;
    if (isLocalPcMode('comm')) {
      if (!_localPort) throw new Error('Chưa kết nối cổng USB Local PC nào - bấm "Chọn cổng USB trên máy này" trước.');
      data = await localProbe(Object.assign({ baud: parseInt(document.getElementById('comm_baud').value, 10) }, params));
    } else {
      const body = Object.assign(commDeviceBaud(), {
        listen_s: params.listenS, spy_only: params.spyOnly, protocol: params.protocol,
        unit_id: params.unitId, function_code: params.functionCode,
        start_address: params.startAddress, value: params.value,
      });
      const res = await fetch('api/probe', {
        method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
      });
      data = await res.json();
    }
    if (data.error) {
      out.innerHTML = `<div class="err">Lỗi: ${data.error}</div>`;
    } else {
      out.innerHTML = `<div>Đã gửi: <code>${data.sent_hex || '-'}</code><br>Phản hồi: <code>${data.active_hex || '(không có phản hồi)'}</code></div>`;
    }
  } catch (e) {
    out.innerHTML = `<div class="err">Lỗi kết nối: ${e.message || e}</div>`;
  } finally {
    btn.disabled = false;
    persistCommState();
  }
});

function toggleRepeatProtocolFields() {
  const isModbus = document.getElementById('repeat_protocol').value === 'modbus_rtu';
  document.getElementById('repeat_raw_fields').style.display = isModbus ? 'none' : 'block';
  document.getElementById('repeat_modbus_fields').style.display = isModbus ? 'block' : 'none';
}
document.getElementById('repeat_protocol').addEventListener('change', toggleRepeatProtocolFields);
toggleRepeatProtocolFields();

// --- Luu log/ket qua tab Communication vao localStorage - de con xem lai
// sau khi F5/dong trang, vi truoc gio chi luu trong bo nho JS (mat het khi
// tai lai trang). Gioi han MAX_LOG_ROWS de tranh localStorage phinh to vo
// han qua thoi gian dai chay "Gui lap lai".
const COMM_STORAGE_KEY = 'usbmgr_comm_state_v1';
const MAX_LOG_ROWS = 200;

function persistCommState() {
  try {
    const state = {
      scan_unit_result: document.getElementById('scan_unit_result').innerHTML,
      scan_reg_result: document.getElementById('scan_reg_result').innerHTML,
      setaddr_result: document.getElementById('setaddr_result').innerHTML,
      repeat_log: Array.from(document.getElementById('repeat_log').children)
        .slice(0, MAX_LOG_ROWS).map(tr => tr.outerHTML),
    };
    localStorage.setItem(COMM_STORAGE_KEY, JSON.stringify(state));
  } catch (e) { /* localStorage co the bi chan (private window, het dung luong...) - im lang */ }
}

function restoreCommState() {
  try {
    const raw = localStorage.getItem(COMM_STORAGE_KEY);
    if (!raw) return;
    const state = JSON.parse(raw);
    if (state.scan_unit_result) document.getElementById('scan_unit_result').innerHTML = state.scan_unit_result;
    if (state.scan_reg_result) document.getElementById('scan_reg_result').innerHTML = state.scan_reg_result;
    if (state.setaddr_result) document.getElementById('setaddr_result').innerHTML = state.setaddr_result;
    if (state.repeat_log && state.repeat_log.length) {
      document.getElementById('repeat_log').innerHTML = state.repeat_log.join('');
    }
  } catch (e) { /* im lặng - lan dau mo trang hoac du lieu cu bi hong thi bo qua */ }
}
restoreCommState();

// --- Icon cai dat (goc tren phai) - xem dung luong localStorage + xoa RIENG
// ket qua/log da luu, KHONG dung toi ghi chu Local PC (LOCAL_PC_NOTE_PREFIX)
// hay bat ky thu gi tren server (ghi chu port ao/USB, thiet bi unavailable -
// nhung thu do nam trong /data/ cua add-on, khong lien quan gi localStorage
// ca nen tu no da an toan, khong can code gi them de "bao ve" chung).
function fmtBytesSettings(n) {
  if (n < 1024) return n + ' B';
  if (n < 1024 * 1024) return (n / 1024).toFixed(1) + ' KB';
  return (n / 1024 / 1024).toFixed(2) + ' MB';
}

function computeLocalStorageUsage() {
  let commBytes = 0, noteBytes = 0, otherBytes = 0;
  try {
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      const value = localStorage.getItem(key) || '';
      const size = (key.length + value.length) * 2; // xap xi 2 byte/ky tu (UTF-16)
      if (key === COMM_STORAGE_KEY) commBytes += size;
      else if (key.startsWith(LOCAL_PC_NOTE_PREFIX)) noteBytes += size;
      else otherBytes += size;
    }
  } catch (e) { /* localStorage co the bi chan (private window...) - im lang */ }
  return { commBytes, noteBytes, otherBytes };
}

function updateStorageUsageInfo() {
  const { commBytes, noteBytes, otherBytes } = computeLocalStorageUsage();
  const total = commBytes + noteBytes + otherBytes;
  document.getElementById('storage_usage_info').innerHTML = `
    Tổng dung lượng đang dùng: <b>${fmtBytesSettings(total)}</b><br>
    &nbsp;&nbsp;- Kết quả/log Communication: ${fmtBytesSettings(commBytes)}<br>
    &nbsp;&nbsp;- Ghi chú thiết bị Local PC: ${fmtBytesSettings(noteBytes)}
    ${otherBytes ? '<br>&nbsp;&nbsp;- Khác: ' + fmtBytesSettings(otherBytes) : ''}
  `;
}

document.getElementById('settings_gear_btn').addEventListener('click', () => {
  const panel = document.getElementById('settings_panel');
  const willShow = panel.style.display === 'none';
  panel.style.display = willShow ? 'block' : 'none';
  if (willShow) updateStorageUsageInfo();
});

document.getElementById('clear_comm_storage_btn').addEventListener('click', () => {
  if (!confirm('Xoá toàn bộ kết quả/log đã lưu của tab Communication?\\n\\n(KHÔNG ảnh hưởng ghi chú thiết bị Local PC, KHÔNG ảnh hưởng ghi chú/dữ liệu trên server, KHÔNG ảnh hưởng thiết bị "Không còn cắm".)')) return;
  try {
    localStorage.removeItem(COMM_STORAGE_KEY);
  } catch (e) { /* im lặng */ }
  document.getElementById('scan_unit_result').innerHTML = '';
  document.getElementById('scan_reg_result').innerHTML = '';
  document.getElementById('setaddr_result').innerHTML = '';
  document.getElementById('repeat_log').innerHTML = '';
  updateStorageUsageInfo();
});

let _repeatIntervalHandle = null;

async function doRepeatSend() {
  const out = document.getElementById('repeat_log');
  const protocol = document.getElementById('repeat_protocol').value;
  const params = {
    listenS: parseFloat(document.getElementById('repeat_listen_s').value) || 0.3,
    spyOnly: false,
    protocol: protocol,
  };
  if (protocol === 'modbus_rtu') {
    params.unitId = parseInt(document.getElementById('repeat_unit_id').value, 10);
    params.functionCode = parseInt(document.getElementById('repeat_function_code').value, 10);
    params.startAddress = parseInt(document.getElementById('repeat_start_address').value, 10);
    params.quantity = parseInt(document.getElementById('repeat_quantity').value, 10);
  } else {
    params.sendCommand = document.getElementById('repeat_send_command').value || null;
  }
  const ts = new Date().toLocaleTimeString('vi-VN');
  const row = document.createElement('tr');
  try {
    let data;
    if (isLocalPcMode('comm')) {
      if (!_localPort) throw new Error('Chưa kết nối cổng USB Local PC nào.');
      data = await localProbe(Object.assign({ baud: parseInt(document.getElementById('comm_baud').value, 10) }, params));
    } else {
      const body = Object.assign(commDeviceBaud(), {
        listen_s: params.listenS, spy_only: params.spyOnly, protocol: params.protocol,
        unit_id: params.unitId, function_code: params.functionCode,
        start_address: params.startAddress, quantity: params.quantity,
        send_command: params.sendCommand,
      });
      const res = await fetch('api/probe', {
        method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
      });
      data = await res.json();
    }
    if (data.error) {
      row.classList.add('err');
      row.innerHTML = `<td>${ts}</td><td colspan="2">Lỗi: ${data.error}</td>`;
    } else {
      row.innerHTML = `<td>${ts}</td>
        <td class="mono">${data.sent_hex || '-'}</td>
        <td class="mono">${data.active_hex || '<i>(không có phản hồi)</i>'}</td>`;
    }
  } catch (e) {
    row.classList.add('err');
    row.innerHTML = `<td>${ts}</td><td colspan="2">Lỗi kết nối: ${e.message || e}</td>`;
  }
  out.insertBefore(row, out.firstChild);
  persistCommState();
}

document.getElementById('repeat_clear_btn').addEventListener('click', () => {
  document.getElementById('repeat_log').innerHTML = '';
  persistCommState();
});

document.getElementById('repeat_btn').addEventListener('click', () => {
  const btn = document.getElementById('repeat_btn');
  if (_repeatIntervalHandle) {
    clearInterval(_repeatIntervalHandle);
    _repeatIntervalHandle = null;
    btn.textContent = '▶ Bắt đầu gửi';
    return;
  }
  const intervalS = parseFloat(document.getElementById('repeat_interval_s').value) || 0;
  doRepeatSend();
  if (intervalS > 0) {
    _repeatIntervalHandle = setInterval(doRepeatSend, intervalS * 1000);
    btn.textContent = '⏸ Dừng lại';
  }
});

// --- Tab Log: event log (mất/có lại kết nối USB, client, lỗi) + test kết nối nhanh ---
const EVENT_CATEGORY_LABEL = {
  connect: '🟢 Kết nối', disconnect: '🔴 Mất kết nối', warning: '🟡 Cảnh báo',
  error: '🛑 Lỗi', client: '🔵 Client TCP', info: '⚪ Thông tin',
};

function escHtml(s) {
  return String(s == null ? '' : s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

async function loadEventLog() {
  const body = document.getElementById('event_log_body');
  if (!body) return;
  const filter = document.getElementById('log_filter').value;
  try {
    const res = await fetch('api/event_log');
    const list = await res.json();
    const filtered = filter ? list.filter(e => e.category === filter) : list;
    if (!filtered.length) {
      body.innerHTML = '<tr><td colspan="4"><i>Chưa có sự kiện nào.</i></td></tr>';
      return;
    }
    body.innerHTML = filtered.map(e => `
      <tr class="log-${escAttr(e.category)}">
        <td>${escHtml(e.ts)}</td>
        <td><span class="log-badge ${escAttr(e.category)}">${EVENT_CATEGORY_LABEL[e.category] || escHtml(e.category)}</span></td>
        <td>${escHtml(e.port)}</td>
        <td>${escHtml(e.message)}</td>
      </tr>`).join('');
  } catch (e) {
    body.innerHTML = '<tr><td colspan="4"><i>Không tải được log.</i></td></tr>';
  }
}
document.getElementById('log_filter').addEventListener('change', loadEventLog);
loadEventLog();
setInterval(loadEventLog, 3000);

async function loadPortTestList() {
  const out = document.getElementById('port_test_list');
  try {
    const res = await fetch('api/ports');
    const list = await res.json();
    const tcpPorts = list.filter(p => p.enabled && p.output_mode !== 'pty' && p.tcp_port);
    if (!tcpPorts.length) {
      out.innerHTML = '<i>Không có port ảo TCP nào đang bật.</i>';
      return;
    }
    let html = '<table class="ports"><tr><th>Tên</th><th>TCP port</th><th></th><th>Kết quả</th></tr>';
    for (const p of tcpPorts) {
      html += `<tr>
        <td>${escHtml(p.name)}</td>
        <td>:${p.tcp_port}</td>
        <td><button type="button" class="port-test-btn" data-port-name="${escAttr(p.name)}"
          style="width:auto; margin-top:0; padding:4px 8px; font-size:0.8rem;">▶ Test</button></td>
        <td id="port_test_result_${escAttr(p.name)}" style="font-size:0.85rem; color:#9aa0a6;">-</td>
      </tr>`;
    }
    html += '</table>';
    out.innerHTML = html;
    out.querySelectorAll('.port-test-btn').forEach(btn => {
      btn.addEventListener('click', () => runPortTest(btn));
    });
  } catch (e) {
    out.innerHTML = '<i>Không tải được danh sách port ảo.</i>';
  }
}

async function runPortTest(btn) {
  const name = btn.dataset.portName;
  const resultCell = document.getElementById('port_test_result_' + name);
  btn.disabled = true;
  resultCell.textContent = 'Đang test...';
  resultCell.style.color = '#9aa0a6';
  try {
    const res = await fetch('api/test_port_connection', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({name: name}),
    });
    const result = await res.json();
    if (result.ok) {
      resultCell.textContent = `✅ OK (${result.latency_ms}ms)`;
      resultCell.style.color = '#7ee787';
    } else {
      resultCell.textContent = '❌ ' + (result.error || 'lỗi không rõ');
      resultCell.style.color = '#ff6b6b';
    }
  } catch (e) {
    resultCell.textContent = '❌ Lỗi kết nối khi gọi test.';
    resultCell.style.color = '#ff6b6b';
  }
  btn.disabled = false;
  loadEventLog();
}
loadPortTestList();
setInterval(loadPortTestList, 10000);
</script>
</body>
</html>
"""


_CONFIG_UI_HTML = r"""<div id="tab_config" class="tab-content">
  <h1>Cấu hình USB Manager</h1>
  <p class="note">Nhận diện bằng phản hồi → gán thiết bị đúng → xuất TCP cố định. Cấu hình được lưu trên addon, giữ qua restart và cập nhật.</p>
  <div class="cfg-toolbar">
    <button id="cfg_add" type="button">＋ Thêm port</button>
    <button id="cfg_reload" type="button">Tải lại</button>
    <button id="cfg_export" type="button">Xuất bản sao</button>
    <button id="cfg_import" type="button">Nhập bản sao</button>
    <input id="cfg_file" type="file" accept="application/json,.json" hidden>
    <button id="cfg_save" type="button">Lưu và áp dụng</button>
  </div>
  <p id="cfg_status" role="status" aria-live="polite">Đang tải cấu hình…</p>
  <div id="cfg_ports"></div>
  <details class="card cfg-settings"><summary>Cài đặt chung · Quét USB, MQTT, log</summary>
    <div class="cfg-toolbar"><label for="cfg_exclude_device">Chọn USB cần loại trừ</label><select id="cfg_exclude_device" style="width:min(600px,100%)"></select><button id="cfg_exclude_add" type="button">Thêm vào loại trừ</button><button id="cfg_exclude_refresh" type="button">Quét danh sách USB</button></div>
    <div id="cfg_globals" class="cfg-grid"></div>
    <p>Danh sách loại trừ: mỗi dòng một đường dẫn ổn định by-id hoặc by-path. Các thiết bị này sẽ không được dò tự động.</p>
    <p>Mật khẩu MQTT để trống sẽ giữ mật khẩu đã lưu. Chọn “Xóa mật khẩu” nếu muốn bỏ.</p>
    <label><input id="cfg_clear_password" type="checkbox"> Xóa mật khẩu MQTT đã lưu</label>
    <p>Thời gian chờ USB chỉ dùng khi addon khởi động. MQTT và các cài đặt còn lại được áp dụng khi lưu.</p>
  </details>
  <dialog id="cfg_editor" class="card">
    <form id="cfg_form">
      <h2 id="cfg_editor_title">Thêm port</h2>
      <p>ID cố định giữ liên kết MQTT và lịch sử nhận diện. Dùng “Tên hiển thị” để đổi tên port đã có.</p>
      <div id="cfg_basic" class="cfg-grid"></div>
      <h3>Nhận diện thiết bị</h3>
      <p>Nhập lệnh và dấu hiệu phản hồi đặc trưng. HEX dùng <code>hex:0104</code>; văn bản dùng <code>text:GET_ID$</code>.</p>
      <div id="cfg_identity" class="cfg-grid"></div>
      <p class="note">Hai phản hồi là điều kiện HOẶC. Một dấu hiệu quá ngắn có thể khớp nhiều thiết bị; dữ liệu đo thay đổi cũng có thể làm mất khớp. Với Modbus, để trống cả hai phản hồi để kiểm CRC + địa chỉ + mã hàm.</p>
      <details><summary>Tùy chọn nâng cao</summary><div id="cfg_advanced" class="cfg-grid"></div></details>
      <details><summary>Kiểm tra quy tắc với phản hồi đã thu</summary>
        <p>Dán phản hồi từ Get Response. Phép kiểm tra này chỉ so khớp dữ liệu, không gửi lệnh ra USB.</p>
        <label for="cfg_response">Phản hồi thực tế (hex:… hoặc text:…)</label><textarea id="cfg_response" rows="3"></textarea>
        <button id="cfg_check" type="button">Kiểm tra so khớp</button><p id="cfg_check_result" role="status"></p>
      </details>
      <p id="cfg_editor_error" class="err" role="alert"></p>
      <div class="cfg-toolbar"><button type="submit">Giữ thay đổi port</button><button id="cfg_cancel" type="button">Hủy</button></div>
      <p>Nhấn “Lưu và áp dụng” ở danh sách để ghi cấu hình lên addon.</p>
    </form>
  </dialog>
</div>
<style>
 .cfg-toolbar {display:flex;gap:8px;flex-wrap:wrap;align-items:center}
 .cfg-toolbar button {width:auto;margin-top:6px}
 .cfg-grid {display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:8px 20px}
 .cfg-grid input[type=number] {max-width:none}
 #tab_config input[type=checkbox] {width:auto;margin-right:8px}
 #tab_config textarea {width:100%;box-sizing:border-box;background:#1c1f26;color:#e6e6e6;border:1px solid #333842;border-radius:6px;padding:8px;margin-top:4px}
 #cfg_editor {color:#e6e6e6;width:min(860px,90vw);max-height:85vh;overflow:auto}
 #cfg_editor::backdrop {background:#0009}
 #tab_config details {margin-top:16px;padding:12px;border:1px solid #333842;border-radius:8px}
 #tab_config summary {cursor:pointer;font-weight:600}
 .cfg-port {display:flex;gap:16px;align-items:center;justify-content:space-between;flex-wrap:wrap;margin-top:12px}
 .cfg-port h3 {margin:0 0 6px;font-size:1rem}
 .cfg-port p {margin:4px 0;color:#9aa0a6;font-size:.85rem}
 #cfg_save {background:#238636}
 #cfg_ports .cfg-disabled {opacity:.65}
 .tab-bar {flex-wrap:wrap;padding-right:46px}
</style>
"""
_CONFIG_UI_SCRIPT = r"""
(() => {
  const $ = id => document.getElementById(id);
  const clone = value => JSON.parse(JSON.stringify(value));
  let state=null, dirty=false, editIndex=-1, pollTimer=null, applying=false;
  const portDefaults={name:'',friendly_name:'',enabled:true,protocol:'raw',baud:9600,unit_id:1,function_code:3,start_address:0,quantity:1,value:0,send_command:'',send_command_2:'',expected_response:'',expected_response_2:'',match_mode:'contains',fuzzy_threshold:80,output_mode:'tcp',tcp_port:6001,mbap_rtu_bridge:false,modbus_response_timeout_s:1,pty_symlink:'',on_connect_send:''};
  const basic=[['name','ID port','text'],['friendly_name','Tên hiển thị','text'],['enabled','Bật port','checkbox'],['protocol','Giao thức thiết bị',['raw','modbus_rtu']],['baud','Baud','number',300,4000000],['output_mode','Kiểu xuất',['tcp','pty']],['tcp_port','Cổng TCP (6001–6010)','number',6001,6010],['mbap_rtu_bridge','Chuyển Modbus TCP ↔ RTU','checkbox']];
  const identity=[['send_command','Lệnh nhận diện (Raw)','text'],['unit_id','Địa chỉ Modbus','number',1,247],['function_code','Mã hàm Modbus','number',1,127],['start_address','Địa chỉ bắt đầu','number',0,65535],['quantity','Số lượng','number',1,2000],['value','Giá trị ghi (FC05/06)','number',0,65535],['expected_response','Phản hồi nhận diện 1','textarea'],['expected_response_2','Phản hồi nhận diện 2 (tùy chọn)','textarea'],['match_mode','Cách so khớp',['contains','exact','startswith','fuzzy']]];
  const advanced=[['send_command_2','Lệnh dự phòng','text'],['fuzzy_threshold','Ngưỡng fuzzy (%)','number',1,100],['probe_timeout_s','Timeout dò (giây; trống = mặc định)','number',.05,30],['rescan_interval_s','Chu kỳ dò lại (giây; trống = mặc định)','number',1,3600],['modbus_response_timeout_s','Timeout phản hồi Modbus TCP (giây)','number',.1,30],['pty_symlink','Đường dẫn PTY','text'],['on_connect_send','Dữ liệu chào client TCP (tùy chọn)','text']];
  const globals=[['loglevel','Mức log',['error','warning','info','debug']],['scan_glob','Mẫu quét USB','text'],['exclude_usb','USB loại trừ (mỗi dòng một đường dẫn)','textarea'],['settle_delay_s','Chờ USB khi khởi động (giây)','number',0,120],['default_probe_timeout_s','Timeout dò mặc định (giây)','number',.05,30],['default_rescan_interval_s','Chu kỳ dò mặc định (giây)','number',1,3600],['passive_listen_s','Thời gian nghe trước khi gửi (giây)','number',0,30],['mqtt_host','MQTT host (trống = tự động)','text'],['mqtt_port','MQTT port','number',1,65535],['mqtt_username','MQTT username','text'],['mqtt_password','Mật khẩu MQTT mới','password']];
  async function api(path,body) {
    const response=await fetch(path,{cache:'no-store',...(body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{})});
    const result=await response.json();
    if(!response.ok || result.ok===false) throw new Error(result.error || 'Không thể thực hiện');
    return result;
  }
  function status(message,error=false) {$('cfg_status').textContent=message;$('cfg_status').className=error?'err':'';}
  function markDirty() {dirty=true;status('Có thay đổi chưa lưu. Lưu sẽ dò lại các port bị sửa; client TCP của các port đó cần kết nối lại.');}
  function fields(container,defs,values,prefix) {
    $(container).replaceChildren();
    for(const [key,label,type,min,max] of defs) {
      const wrapper=document.createElement('div');wrapper.dataset.field=key;
      const caption=document.createElement('label');caption.textContent=label;caption.htmlFor=prefix+key;
      const input=document.createElement(Array.isArray(type)?'select':type==='textarea'?'textarea':'input');
      input.id=prefix+key;input.dataset.key=key;
      if(Array.isArray(type)) for(const v of type){const o=document.createElement('option');o.value=v;o.textContent=({raw:'Raw UART',modbus_rtu:'Modbus RTU',tcp:'TCP',pty:'PTY',contains:'Chứa chuỗi',exact:'Giống toàn bộ',startswith:'Bắt đầu bằng',fuzzy:'Gần giống'})[v]||v;input.append(o);}
      else if(type!=='textarea') input.type=type;
      const value=values[key];
      if(type==='checkbox')input.checked=!!value;
      else input.value=Array.isArray(value)?value.join('\n'):(value??'');
      if(type==='number'){input.min=min;input.max=max;input.step=['probe_timeout_s','modbus_response_timeout_s','default_probe_timeout_s','passive_listen_s'].includes(key)?'any':'1';}
      if(type==='textarea') input.rows=3;
      if(type==='password'){input.autocomplete='new-password';input.placeholder=state?.has_mqtt_password?'Đã có mật khẩu; trống = giữ nguyên':'Chưa lưu mật khẩu';}
      wrapper.append(caption,input);$(container).append(wrapper);
    }
  }
  function read(defs,prefix) {
    const result={};
    for(const [key,label,type] of defs) {
      const input=$(prefix+key);
      if(type==='checkbox')result[key]=input.checked;
      else if(type==='number'){if(input.value!=='')result[key]=Number(input.value);}
      else result[key]=input.value;
    }
    return result;
  }
  function visibility() {
    const modbus=$('cp_protocol').value==='modbus_rtu',tcp=$('cp_output_mode').value==='tcp';
    for(const key of ['unit_id','function_code','start_address','quantity','value']) $('cp_'+key).parentElement.hidden=!modbus;
    $('cp_send_command').parentElement.hidden=modbus;
    $('cp_tcp_port').parentElement.hidden=!tcp;
    $('cp_pty_symlink').parentElement.hidden=tcp;
    $('cp_mbap_rtu_bridge').parentElement.hidden=!modbus||!tcp;
    $('cp_modbus_response_timeout_s').parentElement.hidden=!modbus||!tcp||!$('cp_mbap_rtu_bridge').checked;
    $('cp_fuzzy_threshold').parentElement.hidden=$('cp_match_mode').value!=='fuzzy';
  }
  function readPort() {
    const result=read([...basic,...identity,...advanced],'cp_');
    if(result.protocol!=='modbus_rtu'||result.output_mode!=='tcp')result.mbap_rtu_bridge=false;
    return result;
  }
  function edit(index) {
    if(applying)return;
    editIndex=index;
    const next=6001+Array.from({length:10},(_,i)=>i).find(i=>!state.options.ports.some(p=>p.output_mode==='tcp'&&p.tcp_port===6001+i));
    const value=index<0?{...portDefaults,tcp_port:Number.isFinite(next)?next:6001}:{...portDefaults,...state.options.ports[index]};
    fields('cfg_basic',basic,value,'cp_');fields('cfg_identity',identity,value,'cp_');fields('cfg_advanced',advanced,value,'cp_');
    $('cp_name').readOnly=index>=0;
    $('cfg_editor_title').textContent=index<0?'Thêm port':'Sửa port '+value.name;
    $('cfg_editor_error').textContent='';$('cfg_check_result').textContent='';$('cfg_response').value='';
    for(const key of ['protocol','output_mode','mbap_rtu_bridge','match_mode'])$('cp_'+key).onchange=visibility;
    visibility();$('cfg_editor').showModal();
  }
  function renderPorts() {
    $('cfg_ports').replaceChildren();
    for(const [index,p] of state.options.ports.entries()) {
      const card=document.createElement('div');card.className='card cfg-port'+(p.enabled?'':' cfg-disabled');
      const info=document.createElement('div');const title=document.createElement('h3');title.textContent=p.friendly_name||p.name;
      const line=document.createElement('p');line.textContent=p.name+' · '+p.protocol+' · '+p.baud+' baud · '+(p.output_mode==='pty'?p.pty_symlink:':'+p.tcp_port)+(p.mbap_rtu_bridge?' · Modbus TCP':'')+' · '+(p.enabled?'Đã bật':'Đã tắt');
      const rule=document.createElement('p');rule.textContent='Nhận diện: '+(p.expected_response||p.expected_response_2||'CRC + địa chỉ + mã hàm Modbus');
      info.append(title,line,rule);const buttons=document.createElement('div');buttons.className='cfg-toolbar';
      for(const [label,action] of [['Sửa',()=>edit(index)],[p.enabled?'Tắt':'Bật',()=>{p.enabled=!p.enabled;markDirty();renderPorts();}],['Xóa',()=>{if(confirm('Xóa port '+p.name+'? Sau khi lưu, TCP của port này sẽ dừng.')){state.options.ports.splice(index,1);markDirty();renderPorts();}}]]) {
        const b=document.createElement('button');b.type='button';b.textContent=label;b.disabled=applying;b.onclick=action;buttons.append(b);
      }
      card.append(info,buttons);$('cfg_ports').append(card);
    }
    if(!state.options.ports.length)$('cfg_ports').textContent='Chưa có port. Nhấn “Thêm port” để cấu hình nhận diện và TCP.';
  }
  function lock(value) {
    applying=value;$('cfg_save').disabled=value;$('cfg_add').disabled=value;$('cfg_import').disabled=value;
    $('cfg_globals').querySelectorAll('input,select,textarea').forEach(input=>input.disabled=value);
    $('cfg_clear_password').disabled=value;renderPorts();
    $('cfg_exclude_add').disabled=value;
  }
  async function exclusions() {
    try {
      const devices=await api('api/usb_devices');$('cfg_exclude_device').replaceChildren();
      for(const d of devices) {
        if(d.unavailable)continue;
        const path=d.by_path?'/dev/serial/by-path/'+d.by_path:d.by_id?'/dev/serial/by-id/'+d.by_id:null;
        if(!path)continue;
        const o=document.createElement('option');o.value=path;o.textContent=d.devname+' · '+(d.note||d.product||'USB')+' · '+path;$('cfg_exclude_device').append(o);
      }
      if(!$('cfg_exclude_device').options.length){const o=document.createElement('option');o.value='';o.textContent='Không có USB với đường dẫn ổn định';$('cfg_exclude_device').append(o);}
    }catch(e){status(e.message,true);}
  }
  async function load(snapshot=null) {
    try {
      state=snapshot||await api('api/config');dirty=false;
      fields('cfg_globals',globals,state.options,'cg_');$('cfg_clear_password').checked=false;
      $('cfg_globals').oninput=markDirty;lock(state.applying);
      status(state.error || (state.applying?'Đang áp dụng cấu hình…':'Đã tải cấu hình. '+state.options.ports.length+' port.'),!!state.error);
      if(state.applying)poll();
      exclusions();
    } catch(e){status(e.message,true);}
  }
  function draftOptions() {
    const g=read(globals,'cg_');g.exclude_usb=g.exclude_usb.split(/\r?\n/).map(s=>s.trim()).filter(Boolean);
    if(!g.mqtt_password&&!$('cfg_clear_password').checked)delete g.mqtt_password;
    return {...state.options,...g,ports:clone(state.options.ports)};
  }
  async function poll() {
    clearTimeout(pollTimer);
    pollTimer=setTimeout(async()=>{
      try {
        const current=await api('api/config');
        if(current.applying){status('Đang áp dụng; chờ port cũ đóng kết nối…');poll();return;}
        await load(current);status(current.error||'Đã lưu và áp dụng. Các port bị sửa đang tự dò thiết bị.',!!current.error);
      }catch(e){status('Đang chờ addon: '+e.message,true);poll();}
    },1500);
  }
  $('cfg_form').onsubmit=e=>{
    e.preventDefault();const p=readPort();
    if(!/^[A-Za-z0-9_-]{1,64}$/.test(p.name)){$('cfg_editor_error').textContent='ID port cần 1–64 chữ không dấu, số, dấu - hoặc _.';return;}
    if(state.options.ports.some((other,i)=>i!==editIndex&&other.name===p.name)){$('cfg_editor_error').textContent='ID port đã tồn tại.';return;}
    if(p.enabled&&p.protocol==='raw'&&!p.expected_response&&!p.expected_response_2){$('cfg_editor_error').textContent='Cần ít nhất một phản hồi nhận diện cho Raw.';return;}
    if(editIndex<0)state.options.ports.push(p);else state.options.ports[editIndex]=p;
    $('cfg_editor').close();markDirty();renderPorts();
  };
  $('cfg_add').onclick=()=>edit(-1);$('cfg_cancel').onclick=()=>$('cfg_editor').close();
  $('cfg_reload').onclick=()=>{if(!dirty||confirm('Bỏ thay đổi chưa lưu và tải cấu hình trên addon?'))load();};
  $('cfg_clear_password').onchange=markDirty;
  $('cfg_exclude_refresh').onclick=exclusions;
  $('cfg_exclude_add').onclick=()=>{
    const path=$('cfg_exclude_device').value;if(!path||applying)return;
    const values=$('cg_exclude_usb').value.split(/\r?\n/).map(s=>s.trim()).filter(Boolean);
    if(!values.includes(path)){values.push(path);$('cg_exclude_usb').value=values.join('\n');markDirty();}
  };
  $('cfg_save').onclick=async()=>{
    if(!state||applying)return;
    try {
      const result=await api('api/config',{revision:state.revision,options:draftOptions()});
      state.revision=result.revision;dirty=false;lock(true);status('Đã ghi cấu hình. Đang áp dụng…');poll();
    } catch(e){status(e.message,true);}
  };
  $('cfg_check').onclick=async()=>{
    try {const result=await api('api/config/check-response',{port:readPort(),response:$('cfg_response').value});$('cfg_check_result').textContent=(result.matched?'Khớp':'Không khớp')+' · '+result.bytes+' byte';}
    catch(e){$('cfg_check_result').textContent=e.message;}
  };
  $('cfg_export').onclick=()=>{
    if(!state)return;
    const options=draftOptions();delete options.mqtt_password;
    const blob=new Blob([JSON.stringify({version:1,options},null,2)],{type:'application/json'}),url=URL.createObjectURL(blob);
    const a=document.createElement('a');a.href=url;a.download='usb-manager-config.json';a.click();URL.revokeObjectURL(url);
    status('Đã xuất cấu hình hiện trên form. Bản sao không chứa mật khẩu MQTT.');
  };
  $('cfg_import').onclick=()=>$('cfg_file').click();
  $('cfg_file').onchange=async()=>{
    try {
      const file=$('cfg_file').files[0];if(!file)return;
      if(file.size>1048576)throw new Error('Bản sao quá lớn (tối đa 1 MiB).');
      const doc=JSON.parse(await file.text());
      if(doc.version!==1||!doc.options||!Array.isArray(doc.options.ports))throw new Error('Bản sao không hợp lệ.');
      if(!confirm('Thay danh sách port và cài đặt trên form bằng bản sao? Chưa áp dụng tới khi bạn lưu.'))return;
      state.options={...state.options,...doc.options};delete state.options.mqtt_password;
      fields('cfg_globals',globals,state.options,'cg_');$('cfg_clear_password').checked=false;markDirty();renderPorts();
    }catch(e){status(e.message,true);}finally{$('cfg_file').value='';}
  };
  window.addEventListener('beforeunload',e=>{if(dirty){e.preventDefault();e.returnValue='';}});
  load();
})();
"""
_SPY_PAGE_HTML = _SPY_PAGE_HTML.replace("<script>", _CONFIG_UI_HTML + "<script>", 1).replace("</script>", _CONFIG_UI_SCRIPT + "</script>", 1)

# Configuration owned by the Ingress UI; Supervisor options are imported once.
UI_CONFIG_PATH = "/data/usb-manager-config.json"
_config_lock = threading.RLock()
_config_apply_event = threading.Event()
_config_state = {"options": None, "revision": 0, "applied_revision": 0,
                 "applying": False, "error": None}
_detection_lock = threading.Lock()
GLOBAL_DEFAULTS = {
    "loglevel": "info", "scan_glob": "/dev/ttyUSB*,/dev/ttyACM*", "exclude_usb": [],
    "mqtt_host": "", "mqtt_port": 1883, "mqtt_username": "", "mqtt_password": "",
    "settle_delay_s": 8, "default_probe_timeout_s": 0.4,
    "default_rescan_interval_s": 15, "passive_listen_s": 2.0,
}
PORT_DEFAULTS = {
    "friendly_name": "", "enabled": True, "protocol": "raw", "baud": 9600,
    "unit_id": 1, "function_code": 3, "start_address": 0, "quantity": 1, "value": 0,
    "send_command": "", "send_command_2": "", "expected_response": "",
    "expected_response_2": "", "match_mode": "contains", "fuzzy_threshold": 80,
    "output_mode": "tcp", "tcp_port": 6001, "mbap_rtu_bridge": False,
    "modbus_response_timeout_s": 1.0, "pty_symlink": "", "on_connect_send": "",
}


def _number(obj, key, low, high, integer=False):
    value = obj[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key}: cần nhập số")
    if not low <= value <= high or (integer and int(value) != value):
        raise ValueError(f"{key}: phải trong khoảng {low}–{high}")
    if integer:
        obj[key] = int(value)


def validate_ui_config(payload):
    if not isinstance(payload, dict):
        raise ValueError("Cấu hình phải là một object")
    unknown = set(payload) - set(GLOBAL_DEFAULTS) - {"ports"}
    if unknown:
        raise ValueError("Cài đặt không hỗ trợ: " + ", ".join(sorted(unknown)))
    options = {**GLOBAL_DEFAULTS, **payload}
    for key in ("scan_glob", "mqtt_host", "mqtt_username", "mqtt_password"):
        if not isinstance(options[key], str) or len(options[key]) > 4096:
            raise ValueError(f"{key}: cần chuỗi tối đa 4096 ký tự")
    if options["loglevel"] not in {"error", "warning", "info", "debug"}:
        raise ValueError("Mức log không hợp lệ")
    if not options["scan_glob"].strip():
        raise ValueError("Mẫu quét USB không được trống")
    exclusions = options["exclude_usb"]
    if not isinstance(exclusions, list) or any(not isinstance(p, str) or not p.startswith("/dev/") or "\x00" in p for p in exclusions):
        raise ValueError("USB loại trừ phải là danh sách đường dẫn /dev/")
    _number(options, "mqtt_port", 1, 65535, True)
    _number(options, "settle_delay_s", 0, 120, True)
    _number(options, "default_probe_timeout_s", 0.05, 30)
    _number(options, "default_rescan_interval_s", 1, 3600, True)
    _number(options, "passive_listen_s", 0, 30)
    ports = options.get("ports", [])
    if not isinstance(ports, list) or len(ports) > 100:
        raise ValueError("ports phải là danh sách tối đa 100 port")
    normalized, names, tcp_ports, pty_paths = [], set(), set(), set()
    for cfg in ports:
        if not isinstance(cfg, dict):
            raise ValueError("Mỗi port phải là một object")
        if set(cfg) - set(PORT_DEFAULTS) - {"name", "probe_timeout_s", "rescan_interval_s"}:
            raise ValueError("Port chứa tùy chọn không hỗ trợ")
        port = {**PORT_DEFAULTS, **cfg}
        name = port.get("name")
        if not isinstance(name, str) or not NAME_RE.fullmatch(name) or len(name) > 64:
            raise ValueError("ID port: 1–64 chữ không dấu, số, dấu - hoặc _")
        if name in names:
            raise ValueError(f"Trùng ID port: {name}")
        names.add(name)
        for key in ("friendly_name", "send_command", "send_command_2", "expected_response", "expected_response_2", "pty_symlink", "on_connect_send"):
            if port[key] is None:
                port[key] = ""
            if not isinstance(port[key], str) or len(port[key]) > 16384:
                raise ValueError(f"{name}: {key} phải là chuỗi tối đa 16384 ký tự")
        for key in ("enabled", "mbap_rtu_bridge"):
            if not isinstance(port[key], bool):
                raise ValueError(f"{name}: {key} phải là true/false")
        if port["protocol"] not in {"raw", "modbus_rtu"} or port["output_mode"] not in {"tcp", "pty"}:
            raise ValueError(f"{name}: giao thức hoặc kiểu xuất không hợp lệ")
        if port["match_mode"] not in {"exact", "contains", "startswith", "fuzzy"}:
            raise ValueError(f"{name}: cách so khớp không hợp lệ")
        # Supervisor list() historically serializes baud as a numeric string.
        if isinstance(port["baud"], str) and port["baud"].isdigit():
            port["baud"] = int(port["baud"])
        _number(port, "baud", 300, 4000000, True)
        _number(port, "fuzzy_threshold", 1, 100, True)
        _number(port, "modbus_response_timeout_s", 0.1, 30)
        for key, low, high, integer in (("probe_timeout_s", 0.05, 30, False), ("rescan_interval_s", 1, 3600, True)):
            if key in port:
                _number(port, key, low, high, integer)
        if port["output_mode"] == "tcp":
            _number(port, "tcp_port", 6001, 6010, True)
            if port["tcp_port"] in tcp_ports:
                raise ValueError(f"Trùng TCP :{port['tcp_port']}")
            tcp_ports.add(port["tcp_port"])
        else:
            p = port["pty_symlink"]
            if not p.startswith("/dev/") or "\x00" in p or ".." in p.split("/") or p in pty_paths:
                raise ValueError(f"{name}: đường dẫn PTY phải bắt đầu /dev/ và không trùng")
            pty_paths.add(p)
        if port["protocol"] == "modbus_rtu":
            for key, low, high in (("unit_id", 1, 247), ("function_code", 1, 127), ("start_address", 0, 65535), ("quantity", 1, 2000), ("value", 0, 65535)):
                _number(port, key, low, high, True)
        if port["enabled"] and port["protocol"] == "raw" and not (port["expected_response"] or port["expected_response_2"]):
            raise ValueError(f"{name}: cần phản hồi nhận diện cho Raw")
        if port["mbap_rtu_bridge"] and (port["protocol"] != "modbus_rtu" or port["output_mode"] != "tcp"):
            raise ValueError(f"{name}: Modbus TCP chỉ dùng với Modbus RTU và xuất TCP")
        for key in ("send_command", "send_command_2", "expected_response", "expected_response_2", "on_connect_send"):
            if port[key]:
                decode_command(port[key])
        # Validate frames even for disabled ports, before persisting anything.
        VirtualPort({**port, "enabled": True}, {
            "probe_timeout_s": options["default_probe_timeout_s"],
            "rescan_interval_s": options["default_rescan_interval_s"],
        })
        normalized.append(port)
    options["ports"] = normalized
    return options


def _write_ui_config(options, revision):
    directory = os.path.dirname(UI_CONFIG_PATH) or "."
    os.makedirs(directory, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".usb-manager-", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump({"version": 1, "revision": revision, "options": options}, stream, ensure_ascii=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, UI_CONFIG_PATH)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def load_ui_options():
    with _config_lock:
        if os.path.exists(UI_CONFIG_PATH):
            # Never silently replace an unreadable saved config with sample ports.
            with open(UI_CONFIG_PATH, encoding="utf-8") as stream:
                document = json.load(stream)
            if document.get("version") != 1 or not isinstance(document.get("options"), dict):
                raise ValueError("File cấu hình UI không hợp lệ; hãy khôi phục bản sao lưu")
            options, revision = document["options"], document.get("revision", 1)
        else:
            options, revision = load_options(), 1
            options = {**GLOBAL_DEFAULTS, **options}
            options.setdefault("ports", [])
            _write_ui_config(options, revision)
        _config_state.update(options=options, revision=revision, applied_revision=0)
        return options


def ui_config_snapshot():
    with _config_lock:
        options = json.loads(json.dumps(_config_state["options"] or {**GLOBAL_DEFAULTS, "ports": []}))
        # Show effective legacy defaults without changing the saved detection rule.
        options["ports"] = [{**PORT_DEFAULTS, "match_mode": "exact", **p}
                            for p in options.get("ports", [])]
        has_password = bool(options.pop("mqtt_password", ""))
        return {"options": options, "has_mqtt_password": has_password,
                **{k: _config_state[k] for k in ("revision", "applied_revision", "applying", "error")}}


def save_ui_options(payload):
    with _config_lock:
        if _config_state["applying"]:
            raise ValueError("Đang áp dụng cấu hình trước; vui lòng đợi")
        if payload.get("revision") != _config_state["revision"]:
            raise ValueError("Cấu hình đã thay đổi ở cửa sổ khác; tải lại trước khi sửa")
        draft = payload.get("options")
        if not isinstance(draft, dict):
            raise ValueError("Thiếu options")
        draft = dict(draft)
        if "mqtt_password" not in draft:
            draft["mqtt_password"] = (_config_state["options"] or {}).get("mqtt_password", "")
        options = validate_ui_config(draft)
        revision = _config_state["revision"] + 1
        _write_ui_config(options, revision)
        _config_state.update(options=options, revision=revision, applying=True, error=None)
        _config_apply_event.set()
        return {"ok": True, "revision": revision, "applying": True}


def check_ui_response(payload):
    # Pure matching: no UART is opened and no command is sent.
    cfg = payload["port"]
    options = validate_ui_config({"ports": [cfg]})
    vport = VirtualPort(options["ports"][0], {"probe_timeout_s": 0.4, "rescan_interval_s": 15})
    actual = decode_command(payload.get("response", ""))
    return {"ok": True, "matched": vport.check_passive_buffer(actual), "bytes": len(actual)}


class UIRuntime:
    def __init__(self, claimed_lock, claimed):
        self.claimed_lock, self.claimed = claimed_lock, claimed
        self.options, self.workers = {}, {}
        self.mqtt_client = None
        self.mqtt_stop = threading.Event()
        self.mqtt_thread = None

    def apply(self, options):
        old_ports = {p["name"]: p for p in self.options.get("ports", [])}
        new_ports = {p["name"]: p for p in options.get("ports", [])}
        scan_keys = ("scan_glob", "exclude_usb", "passive_listen_s", "default_probe_timeout_s", "default_rescan_interval_s")
        global_changed = any(self.options.get(k) != options.get(k) for k in scan_keys)
        def effective(cfg):
            if cfg is None:
                return None
            result = {**PORT_DEFAULTS, "match_mode": "exact", **cfg}
            if isinstance(result["baud"], str) and result["baud"].isdigit():
                result["baud"] = int(result["baud"])
            for key, value in list(result.items()):
                if value is None and key in PORT_DEFAULTS and isinstance(PORT_DEFAULTS[key], str):
                    result[key] = ""
            return result
        changed = {name for name in old_ports.keys() | new_ports.keys() | self.workers.keys()
                   if global_changed or effective(old_ports.get(name)) != effective(new_ports.get(name))
                   or (name in self.workers and self.workers[name][0].stop_event.is_set())}
        stopping = [self.workers[name] for name in changed if name in self.workers]
        for vport, thread in stopping:
            vport.stop_event.set()
            vport.rescan_event.set()
        deadline = time.monotonic() + 35
        for vport, thread in stopping:
            thread.join(max(0, deadline - time.monotonic()))
            if thread.is_alive():
                raise RuntimeError(f"Port {vport.name} chưa dừng; hãy khởi động lại addon để áp dụng")
        for name in changed:
            self.workers.pop(name, None)
            with _vport_registry_lock:
                _vport_registry.pop(name, None)
        defaults = {"probe_timeout_s": options["default_probe_timeout_s"], "rescan_interval_s": options["default_rescan_interval_s"]}
        _spy_runtime.update(scan_glob=options["scan_glob"], exclude_usb=options["exclude_usb"], default_listen_s=options["passive_listen_s"])
        for name in changed:
            cfg = new_ports.get(name)
            if cfg is None:
                continue
            if not cfg.get("enabled", True):
                with _vport_registry_lock:
                    _vport_registry[name] = {**cfg, "vport": None}
                continue
            vport = VirtualPort(cfg, defaults)
            thread = threading.Thread(target=run_virtual_port_lifecycle,
                args=(vport, options["scan_glob"], options["passive_listen_s"], self.claimed_lock, self.claimed, options["exclude_usb"]),
                daemon=True, name=name)
            self.workers[name] = (vport, thread)
            with _vport_registry_lock:
                _vport_registry[name] = {"enabled": True, "vport": vport}
            thread.start()
        # Keep registry order equal to the persisted UI order.
        with _vport_registry_lock:
            entries = {name: _vport_registry[name] for name in new_ports if name in _vport_registry}
            _vport_registry.clear()
            _vport_registry.update(entries)
        mqtt_keys = ("mqtt_host", "mqtt_port", "mqtt_username", "mqtt_password")
        if any(self.options.get(k) != options.get(k) for k in mqtt_keys):
            self.mqtt_stop.set()
            if self.mqtt_thread:
                self.mqtt_thread.join(4)
            if self.mqtt_client:
                self.mqtt_client.disconnect()
                self.mqtt_client.loop_stop()
            _mqtt_state.update(client=None, host=None, port=None)
            self.mqtt_client = mqtt_setup(options)
            self.mqtt_stop = threading.Event()
            if self.mqtt_client:
                self.mqtt_thread = threading.Thread(target=mqtt_state_loop,
                    args=(self.mqtt_client, 3.0, self.mqtt_stop), daemon=True)
                self.mqtt_thread.start()
        if self.mqtt_client:
            for name in old_ports.keys() - new_ports.keys():
                device_id = f"usbmgr_{name}"
                for kind, suffix in (("sensor", "device_path"), ("binary_sensor", "connected")):
                    self.mqtt_client.publish(f"{MQTT_DISCOVERY_PREFIX}/{kind}/{device_id}_{suffix}_usb_manager_distribution/config", "", retain=True)
            for cfg in new_ports.values():
                mqtt_publish_discovery(self.mqtt_client, cfg["name"], cfg.get("friendly_name") or cfg["name"], cfg.get("enabled", True))
        logging.getLogger().setLevel(getattr(logging, options["loglevel"].upper()))
        self.options = json.loads(json.dumps(options))


class SpyHTTPHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format_str, *args):  # noqa: A002 - override chuan cua BaseHTTPRequestHandler
        logging.debug("[spy-http] " + format_str, *args)

    def _send_html(self, html: str) -> None:
        data = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, obj) -> None:
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):  # noqa: N802 - ten chuan cua BaseHTTPRequestHandler
        path = self.path.split("?", 1)[0].rstrip("/")
        if path in ("", "/index.html"):
            self._send_html(_SPY_PAGE_HTML)
        elif path == "/api/candidates":
            # Danh sach nay hien TAT CA candidate khop scan_glob (KE CA
            # thiet bi trong exclude_usb) - cong cu Spy/Test la thao tac THU
            # CONG, co y nghia debug (vd tu kiem tra dung thiet bi bi loai
            # tru chua) - chi vong quet TU DONG (find_candidates goi tu
            # run_virtual_port_lifecycle) moi thuc su bo qua exclude_usb.
            #
            # Dung lai list_usb_device_inventory() (thay vi tu tinh rieng)
            # de lay luon "note" nguoi dung da ghi (uu tien ghi chu port ao
            # neu dang duoc claim, roi toi ghi chu thiet bi rieng) - hien
            # ngay trong dropdown chon cong, khong can mo tab USB Manager
            # doi chieu rieng. Loai bo cac dong "unavailable" (thiet bi da
            # rut, chi con ghi chu) vi day la dropdown chon THIET BI DANG
            # CAM THAT, khong phai bang tra cuu lich su.
            claimed_lock = _spy_runtime["claimed_lock"]
            claimed_dict = _spy_runtime["claimed"]
            with claimed_lock:
                claimed_snapshot = dict(claimed_dict)
            inventory = list_usb_device_inventory(
                _spy_runtime["scan_glob"], claimed_snapshot, _spy_runtime["exclude_usb"]
            )
            held = _held_devices()
            data = [
                {
                    "device": row["devname"],
                    "claimed_by": row["claimed_by"],
                    "held_by": held.get(row["devname"]),
                    "excluded": row["excluded"],
                    "note": row["note"],
                    "by_id": row["by_id"],
                }
                for row in inventory
                if not row["unavailable"]
            ]
            self._send_json(data)
        elif path == "/api/config":
            self._send_json(ui_config_snapshot())
        elif path == "/api/ports":
            self._send_json(_ports_status_snapshot())
        elif path == "/api/usb_devices":
            claimed_lock = _spy_runtime["claimed_lock"]
            claimed_dict = _spy_runtime["claimed"]
            with claimed_lock:
                claimed_snapshot = dict(claimed_dict)
            self._send_json(list_usb_device_inventory(
                _spy_runtime["scan_glob"], claimed_snapshot, _spy_runtime["exclude_usb"]
            ))
        elif path == "/api/mqtt_status":
            self._send_json(mqtt_status_snapshot())
        elif path == "/api/version":
            self._send_json({"version": get_addon_version()})
        elif path == "/api/event_log":
            self._send_json(event_log_snapshot())
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):  # noqa: N802 - ten chuan cua BaseHTTPRequestHandler
        path = self.path.split("?", 1)[0].rstrip("/")
        if path in ("/api/config", "/api/config/check-response"):
            try:
                length = int(self.headers.get("Content-Length", 0) or 0)
                if not 0 < length <= 1048576:
                    raise ValueError("Nội dung phải từ 1 byte tới 1 MiB")
                payload = json.loads(self.rfile.read(length))
                result = save_ui_options(payload) if path == "/api/config" else check_ui_response(payload)
                self._send_json(result)
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)})
        elif path == "/api/probe":
            length = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(length) if length else b"{}"
            try:
                payload = json.loads(body)
                device = payload["device"]
                baud = int(payload.get("baud", 9600))
                listen_s = float(payload.get("listen_s", _spy_runtime["default_listen_s"]))
                spy_only = bool(payload.get("spy_only", True))
                result = _spy_probe_port(
                    device, baud, listen_s, spy_only,
                    protocol=payload.get("protocol", "raw"),
                    send_command=payload.get("send_command") or None,
                    unit_id=payload.get("unit_id"),
                    function_code=payload.get("function_code"),
                    start_address=payload.get("start_address", 0),
                    quantity=payload.get("quantity", 1),
                    value=payload.get("value"),
                    share=bool(payload.get("share", False)),
                )
            except Exception as exc:  # noqa: BLE001 - khong duoc lam sap http server
                result = {"error": str(exc)}
            self._send_json(result)
        elif path == "/api/usb_note":
            length = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(length) if length else b"{}"
            try:
                payload = json.loads(body)
                key = str(payload["key"])
                note = str(payload.get("note", "")).strip()
                _save_note(USB_NOTES_PATH, key, note)
                result = {"ok": True}
            except Exception as exc:  # noqa: BLE001 - khong duoc lam sap http server
                result = {"ok": False, "error": str(exc)}
            self._send_json(result)
        elif path == "/api/usb_forget":
            length = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(length) if length else b"{}"
            try:
                _forget_usb_device(str(json.loads(body)["key"]))
                result = {"ok": True}
            except Exception as exc:  # noqa: BLE001 - khong duoc lam sap http server
                result = {"ok": False, "error": str(exc)}
            self._send_json(result)
        elif path == "/api/port_note":
            length = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(length) if length else b"{}"
            try:
                payload = json.loads(body)
                name = str(payload["name"])
                if not NAME_RE.match(name):
                    raise ValueError(f"ten port ao khong hop le: {name!r}")
                note = str(payload.get("note", "")).strip()
                _save_note(PORT_NOTES_PATH, name, note)
                result = {"ok": True}
            except Exception as exc:  # noqa: BLE001 - khong duoc lam sap http server
                result = {"ok": False, "error": str(exc)}
            self._send_json(result)
        elif path == "/api/rescan_port":
            length = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(length) if length else b"{}"
            try:
                payload = json.loads(body)
                name = str(payload["name"])
                with _vport_registry_lock:
                    entry = _vport_registry.get(name)
                vport = entry.get("vport") if entry else None
                if vport is None:
                    result = {"ok": False, "error": f"khong tim thay port ao dang bat ten {name!r}"}
                else:
                    vport.rescan_event.set()
                    logging.info("[%s] Nguoi dung yeu cau Rescan ngay qua Spy/Test UI", name)
                    result = {"ok": True}
            except Exception as exc:  # noqa: BLE001 - khong duoc lam sap http server
                result = {"ok": False, "error": str(exc)}
            self._send_json(result)
        elif path == "/api/modbus_scan_units":
            length = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(length) if length else b"{}"
            try:
                payload = json.loads(body)
                result = _modbus_scan_units(
                    payload["device"], int(payload.get("baud", 9600)),
                    int(payload["unit_start"]), int(payload["unit_end"]),
                    int(payload.get("function_code", 3)),
                    int(payload.get("start_address", 0)),
                    int(payload.get("quantity", 1)),
                    timeout_s=float(payload.get("timeout_s", 0.3)),
                    share=bool(payload.get("share", False)),
                )
            except Exception as exc:  # noqa: BLE001 - khong duoc lam sap http server
                result = {"error": str(exc)}
            self._send_json(result)
        elif path == "/api/modbus_scan_registers":
            length = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(length) if length else b"{}"
            try:
                payload = json.loads(body)
                result = _modbus_scan_registers(
                    payload["device"], int(payload.get("baud", 9600)),
                    int(payload["unit_id"]),
                    [int(x) for x in payload.get("function_codes", [3, 4])],
                    int(payload["addr_start"]), int(payload["addr_end"]),
                    int(payload.get("quantity", 1)),
                    timeout_s=float(payload.get("timeout_s", 0.3)),
                    share=bool(payload.get("share", False)),
                )
            except Exception as exc:  # noqa: BLE001 - khong duoc lam sap http server
                result = {"error": str(exc)}
            self._send_json(result)
        elif path in ("/api/take_test_hold", "/api/release_test_hold"):
            length = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(length) if length else b"{}"
            try:
                name = str(json.loads(body)["name"])
                if path == "/api/take_test_hold":
                    result = _take_test_hold(name)
                else:
                    result = _give_back_test_hold(name)
            except Exception as exc:  # noqa: BLE001 - khong duoc lam sap http server
                result = {"ok": False, "error": str(exc)}
            self._send_json(result)
        elif path == "/api/test_port_connection":
            length = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(length) if length else b"{}"
            try:
                payload = json.loads(body)
                result = _test_port_connection(str(payload["name"]))
            except Exception as exc:  # noqa: BLE001 - khong duoc lam sap http server
                result = {"ok": False, "error": str(exc)}
            self._send_json(result)
        else:
            self.send_response(404)
            self.end_headers()


def start_spy_http_server(scan_glob: str, claimed_lock: threading.Lock, claimed: dict,
                           default_listen_s: float, exclude_usb: list | None = None) -> None:
    _spy_runtime["scan_glob"] = scan_glob
    _spy_runtime["claimed_lock"] = claimed_lock
    _spy_runtime["claimed"] = claimed
    _spy_runtime["default_listen_s"] = default_listen_s
    _spy_runtime["exclude_usb"] = exclude_usb or []
    try:
        server = http.server.ThreadingHTTPServer(("0.0.0.0", SPY_HTTP_PORT), SpyHTTPHandler)
    except OSError as exc:
        logging.error("Khong mo duoc Spy/Test web UI port %d: %s", SPY_HTTP_PORT, exc)
        return
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    logging.info("Spy/Test web UI dang chay tai port %d (xem qua Ingress tren sidebar)", SPY_HTTP_PORT)


def main() -> None:
    options = load_ui_options()
    os.environ.setdefault("TZ", "Asia/Ho_Chi_Minh")
    if hasattr(time, "tzset"):
        time.tzset()
    logging.basicConfig(
        level=getattr(logging, options.get("loglevel", "info").upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S %z")
    claimed_lock, claimed = threading.Lock(), {}
    start_spy_http_server(options["scan_glob"], claimed_lock, claimed, options["passive_listen_s"], options["exclude_usb"])
    runtime = UIRuntime(claimed_lock, claimed)
    with _config_lock:
        _config_state["applying"] = True
    logging.info("Cho %ss de USB enumerate", options["settle_delay_s"])
    time.sleep(options["settle_delay_s"])
    # Keep the process alive even with zero ports, so the UI can add the first.
    _config_apply_event.set()
    while True:
        _config_apply_event.wait()
        _config_apply_event.clear()
        with _config_lock:
            options = json.loads(json.dumps(_config_state["options"]))
            revision = _config_state["revision"]
            _config_state["applying"] = True
        try:
            runtime.apply(options)
            with _config_lock:
                _config_state.update(applied_revision=revision, error=None)
            log_event("info", None, f"Đã áp dụng cấu hình UI #{revision}")
        except Exception as exc:
            logging.exception("Khong ap dung duoc cau hinh UI")
            with _config_lock:
                _config_state["error"] = str(exc)
            log_event("error", None, f"Không áp dụng được cấu hình: {exc}")
        finally:
            with _config_lock:
                _config_state["applying"] = False


if __name__ == "__main__":
    main()
