import threading
import time
from typing import Optional, Dict, Any

import can


CAN_ID_DRIVE_CMD = 0x1F0
CAN_ID_HEARTBEAT = 0x110
CAN_ID_TELEMETRY = 0x102
CAN_ID_ESTOP = 0x103
CAN_ID_ESP_HB = 0x120
CAN_ID_ESP_STATUS = 0x121
CAN_ID_ESP_TLM = 0x122


class CANBridge:
    def __init__(self, channel: str = "can0", bustype: str = "socketcan", verbose_rx: bool = False):
        self.channel = channel
        self.bustype = bustype
        self.verbose_rx = verbose_rx

        self.bus: Optional[can.Bus] = None
        self.seq = 0
        self.hb_counter = 0

        self._rx_thread: Optional[threading.Thread] = None
        self._hb_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self.latest: Dict[str, Any] = {
            "esp_hb": None,
            "esp_status": None,
            "esp_tlm": None,
            "estop": None,
            "last_msg": None,
        }

    def connect(self) -> None:
        self.bus = can.Bus(channel=self.channel, interface=self.bustype)
        self._stop_event.clear()

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)

        self._rx_thread.start()
        self._hb_thread.start()

        print(f"[CAN] Connected to {self.channel}")

    def close(self) -> None:
        self._stop_event.set()

        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self._hb_thread and self._hb_thread.is_alive():
            self._hb_thread.join(timeout=1.0)

        if self.bus is not None:
            self.bus.shutdown()
            print("[CAN] Bus closed")

    def _send(self, arbitration_id: int, data: list[int]) -> None:
        if self.bus is None:
            raise RuntimeError("CAN bus not connected")

        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
        )

        with self._lock:
            self.bus.send(msg)

        print(f"[TX] ID=0x{arbitration_id:03X} DATA={' '.join(f'{b:02X}' for b in data)}")

    def send_drive_lr(self, left: int, right: int) -> None:
        left_u8 = left & 0xFF
        right_u8 = right & 0xFF
        moving = not (left == 0 and right == 0)

        data = [
            0x01 if moving else 0x00,
            0x01,
            left_u8,
            right_u8,
            self.seq & 0xFF,
            0x01 if moving else 0x00,
            0x00,
            0x00,
        ]
        self.seq = (self.seq + 1) & 0xFF
        self._send(CAN_ID_DRIVE_CMD, data)

    def stop(self) -> None:
        self.send_drive_lr(0, 0)

    def estop(self) -> None:
        self._send(CAN_ID_ESTOP, [0x01])

    def send_heartbeat(self) -> None:
        self._send(CAN_ID_HEARTBEAT, [self.hb_counter & 0xFF])
        self.hb_counter = (self.hb_counter + 1) & 0xFF

    def _heartbeat_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.send_heartbeat()
            except Exception as exc:
                print(f"[HB] error: {exc}")
            time.sleep(1.0)

    def _rx_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                if self.bus is None:
                    time.sleep(0.1)
                    continue

                msg = self.bus.recv(timeout=0.2)
                if msg is None:
                    continue

                self.latest["last_msg"] = msg
                self._handle_rx(msg)

            except Exception as exc:
                print(f"[RX] error: {exc}")
                time.sleep(0.2)

    def _handle_rx(self, msg: can.Message) -> None:
        data = list(msg.data)

        if self.verbose_rx:
            print(f"[RX] ID=0x{msg.arbitration_id:03X} DATA={' '.join(f'{b:02X}' for b in data)}")

        if msg.arbitration_id == CAN_ID_ESTOP:
            self.latest["estop"] = {"raw": data}

        elif msg.arbitration_id == CAN_ID_ESP_HB:
            self.latest["esp_hb"] = {
                "raw": data,
                "manual": data[1] if len(data) > 1 else None,
                "estop": data[2] if len(data) > 2 else None,
                "pi_alive": data[3] if len(data) > 3 else None,
            }

        elif msg.arbitration_id == CAN_ID_ESP_STATUS:
            self.latest["esp_status"] = {
                "raw": data,
                "manual": data[0] if len(data) > 0 else None,
                "estop": data[1] if len(data) > 1 else None,
                "pi_alive": data[2] if len(data) > 2 else None,
            }

        elif msg.arbitration_id == CAN_ID_ESP_TLM:
            self.latest["esp_tlm"] = {
                "raw": data,
                "left": self._u8_to_i8(data[0]) if len(data) > 0 else None,
                "right": self._u8_to_i8(data[1]) if len(data) > 1 else None,
                "seq": data[2] if len(data) > 2 else None,
                "flags": data[3] if len(data) > 3 else None,
            }

    @staticmethod
    def _u8_to_i8(v: int) -> int:
        return v - 256 if v > 127 else v
