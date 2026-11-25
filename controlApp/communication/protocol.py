# protocol - the drone's packets, read from its own headers (utilities/cHeaders.py), and the framing:
#   A5 5A | type u8 | length u32 | payload | CRC16-CCITT (0xFFFF) over type + length + payload
# Everything is named as on the drone: MSG.structs["HoverTelem"], MSG.value("PKT_SEG"), STATES[4] == "FLY".

import binascii
import os
import struct

from utilities.cHeaders import Header

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
MSG = Header(os.path.join(ROOT, "whoopCTRL", "shared", "headers", "messages.h"))
SM = Header(os.path.join(ROOT, "whoopCTRL", "M4", "CTRL", "headers", "StateMachine.h"))

STATES = {v: k for k, v in SM.enums["State"].items()}             # ctrl::StateMachine::State: value -> name (5 unused)
PACKET_NAMES = {v: k for k, v in MSG.enums["PacketType"].items()}
MAX_PAYLOAD = 4096


def packet(ptype, payload):
    head = struct.pack("<BI", ptype, len(payload))
    return b"\xA5\x5A" + head + payload + struct.pack("<H", binascii.crc_hqx(head + payload, 0xFFFF))


def parse(data):                        # one datagram -> [(type, payload)]; a broken packet is skipped
    buf, out = bytearray(data), []
    while True:
        i = buf.find(b"\xA5\x5A")
        if i < 0 or len(buf) < i + 7:
            return out
        ptype, n = struct.unpack_from("<BI", buf, i + 2)
        if n > MAX_PAYLOAD:
            del buf[:i + 2]
            continue
        if len(buf) < i + 9 + n:
            return out
        body = bytes(buf[i + 2:i + 7 + n])
        if binascii.crc_hqx(body, 0xFFFF) == struct.unpack_from("<H", buf, i + 7 + n)[0]:
            out.append((ptype, body[5:]))
            del buf[:i + 9 + n]
        else:
            del buf[:i + 2]
