#!/usr/bin/env python3
"""
bsu_usb_protocol.py - Протокол USB для BSU_test_board (для отладки на ПК)

Устройства на шине: 1 ППКУ + 3 МКУ + 3 исполнителя = 7
- ППКУ: h_adr из UID, d_type=0
- МКУ 1 (h_adr=1): igniter l_adr=1
- МКУ 2 (h_adr=2): igniter l_adr=1
- МКУ 3 (h_adr=3): ДПТ l_adr=1

Запуск: python bsu_usb_protocol.py COM5 [--debug] [--read]
  --read  отправить команду чтения настроек ППКУ
"""

import struct
import sys

try:
    import serial
except ImportError:
    print("Установите pyserial: pip install pyserial")
    sys.exit(1)

from typing import Optional, List, Tuple

BSU_PREAMBLE = 0xAA55  # плата шлёт 0x55,0xAA → LE = 0xAA55
BSU_PKT_TYPE_CAN = 0

# CAN-ID: zone(7) | l_adr(6) | h_adr(8) | d_type(7) | dir(1)
def can_id_parts(can_id: int) -> Tuple[int, int, int, int]:
    zone = can_id & 0x7F
    l_adr = (can_id >> 7) & 0x3F
    h_adr = (can_id >> 13) & 0xFF
    d_type = (can_id >> 21) & 0x7F
    return zone, l_adr, h_adr, d_type

def device_name(d_type: int) -> str:
    names = {10: "PPKY", 11: "Igniter", 12: "DPT", 13: "MCU_IGN", 14: "MCU_TC"}
    return names.get(d_type, f"d_type{d_type}")

def checksum(data: bytes) -> int:
    return sum(data) & 0xFFFF

def build_can_packet(can_id: int, data: bytes, seq: int = 0) -> bytes:
    """Собрать CAN-пакет для отправки."""
    if len(data) < 8:
        data = data + b'\x00' * (8 - len(data))
    data = data[:8]

    payload = struct.pack('<I', can_id & 0x1FFFFFFF) + data
    total_size = 22

    pkt = bytearray()
    pkt.extend(struct.pack('<H', BSU_PREAMBLE))
    pkt.extend(struct.pack('<H', total_size))
    pkt.extend(struct.pack('<H', BSU_PKT_TYPE_CAN))
    pkt.extend(struct.pack('<H', seq))
    pkt.extend(payload)

    crc = checksum(pkt)
    pkt.extend(struct.pack('<H', crc))
    return bytes(pkt)

def parse_packet(buf: bytes) -> Optional[Tuple[int, int, bytes]]:
    """Разобрать пакет. Возвращает (type, seq, payload) или None."""
    if len(buf) < 10:
        return None
    if struct.unpack_from('<H', buf, 0)[0] != BSU_PREAMBLE:
        return None
    size = struct.unpack_from('<H', buf, 2)[0]
    if size < 10 or len(buf) < size:
        return None
    pkt = buf[:size]
    calc_crc = checksum(pkt[:-2])
    recv_crc = struct.unpack_from('<H', pkt, size - 2)[0]
    if calc_crc != recv_crc:
        return None
    pkt_type = struct.unpack_from('<H', pkt, 4)[0]
    seq = struct.unpack_from('<H', pkt, 6)[0]
    payload = bytes(pkt[8:size-2])
    return (pkt_type, seq, payload)

class BSUDevice:
    """Класс для работы с BSU_test_board по USB."""
    def __init__(self, port: str, baudrate: int = 115200, debug: bool = False):
        self.ser = serial.Serial(port, baudrate)
        self.rx_buf = bytearray()
        self.seq = 0
        self.debug = debug

    def send_can(self, can_id: int, data: bytes) -> None:
        pkt = build_can_packet(can_id, data)
        self.ser.write(pkt)

    def receive(self, timeout: float = 0.1) -> List[Tuple[int, bytes]]:
        """Принять пакеты. Возвращает список (can_id, data)."""
        self.ser.timeout = timeout
        raw = self.ser.read(256)
        if self.debug and raw:
            print(f"  RAW [{raw.hex()}]")
        self.rx_buf.extend(raw)
        result = []
        while len(self.rx_buf) >= 10:
            parsed = parse_packet(bytes(self.rx_buf))
            if parsed is None:
                if self.rx_buf[0] == 0x55 and len(self.rx_buf) >= 4:
                    size = self.rx_buf[2] | (self.rx_buf[3] << 8)
                    if len(self.rx_buf) < size:
                        break
                    self.rx_buf = self.rx_buf[size:]
                    continue
                self.rx_buf.pop(0)
                continue
            pkt_type, seq, payload = parsed
            size = self.rx_buf[2] | (self.rx_buf[3] << 8)
            self.rx_buf = self.rx_buf[size:]
            if pkt_type == BSU_PKT_TYPE_CAN and len(payload) >= 12:
                can_id = struct.unpack_from('<I', payload, 0)[0]
                data = bytes(payload[4:12])
                result.append((can_id, data))
        return result

    def close(self):
        self.ser.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Использование: python bsu_usb_protocol.py COM5 [--debug]")
        print("  --debug   показать все сырые байты")
        sys.exit(1)

    debug = "--debug" in sys.argv
    port = [a for a in sys.argv[1:] if not a.startswith("--")][0]

    dev = BSUDevice(port, debug=debug)
    print("Подключено. Ожидание пакетов (ППКУ d_type=10, Igniter=11, DPT=12)...")
    if debug:
        print("Режим отладки: вывод сырых байт")
    try:
        while True:
            for can_id, data in dev.receive(timeout=0.5):
                zone, l_adr, h_adr, d_type = can_id_parts(can_id)
                dev_name = device_name(d_type)
                line = f"  CAN 0x{can_id:08X} h={h_adr} l={l_adr} {dev_name} [{data.hex()}]"
                if data[0] >= 150 and data[0] <= 155:
                    line += f"  # ServiceCmd {data[0]}"
                print(line)
    except KeyboardInterrupt:
        pass
    dev.close()
