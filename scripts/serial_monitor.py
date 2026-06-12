#!/usr/bin/env python3
"""
串口监听工具 — 接收 9600 波特率串口数据并输出到终端。

用法：
    ./serial_monitor.py                               # 列出可用串口，让用户选择
    ./serial_monitor.py /dev/ttyUSB0                  # 指定串口设备
    ./serial_monitor.py /dev/ttyUSB0 --hex            # 十六进制显示
    ./serial_monitor.py /dev/ttyUSB0 --modbus-poll    # 每 100ms 发送 Modbus 读请求
    ./serial_monitor.py --help                        # 查看所有选项
"""

import argparse
import sys
import time
import threading
import serial
from serial.tools import list_ports


def crc16_modbus(data: bytes) -> bytes:
    """计算 Modbus RTU CRC-16，返回小端序 2 字节"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def list_available_ports():
    """列出所有可用串口"""
    ports = list_ports.comports()
    if not ports:
        print("未检测到串口设备。")
        sys.exit(1)

    print("可用串口：")
    for i, port in enumerate(ports):
        desc = port.description if port.description else "(无描述)"
        print(f"  [{i}] {port.device} — {desc}")

    return ports


def format_hex(data: bytes) -> str:
    """将字节数据格式化为十六进制字符串"""
    return " ".join(f"{b:02X}" for b in data)


def format_ascii(data: bytes) -> str:
    """将字节数据格式化为可打印 ASCII，不可打印字符用 . 代替"""
    return "".join(chr(b) if 32 <= b < 127 else "." for b in data)


def modbus_poll_sender(ser: serial.Serial, slave: int, reg_addr: int, reg_count: int,
                       interval: float, stop_event: threading.Event):
    """后台线程：每 interval 秒发送一次 Modbus 读保持寄存器请求"""
    while not stop_event.is_set():
        frame = bytes([slave, 0x03,
                       (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
                       (reg_count >> 8) & 0xFF, reg_count & 0xFF])
        frame += crc16_modbus(frame)
        try:
            ser.write(frame)
        except serial.SerialException:
            break
        stop_event.wait(interval)


def alarm_toggle_sender(ser: serial.Serial, slave: int, stop_event: threading.Event):
    """后台线程：每 1s 交替发送报警置位/清除指令（寄存器 0x0004）"""
    alarm = 0
    while not stop_event.is_set():
        frame = bytes([slave, 0x06, 0x00, 0x04, 0x00, alarm])
        frame += crc16_modbus(frame)
        try:
            ser.write(frame)
            print(f"  [alarm_toggle] 发送: {format_hex(frame)}  (alarm={alarm})")
        except serial.SerialException:
            break
        alarm ^= 1
        stop_event.wait(1.0)


def monitor(port: str, baud: int, show_hex: bool, show_ascii: bool, raw: bool,
            modbus_poll: bool = False, poll_slave: int = 1,
            poll_reg_addr: int = 0, poll_reg_count: int = 1,
            alarm_toggle: bool = False, alarm_slave: int = 1):
    """打开串口并持续监听输出"""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,        # 非阻塞读，允许 Ctrl+C 响应
        )
    except serial.SerialException as e:
        print(f"打开串口失败：{e}")
        sys.exit(1)

    print(f"监听 {port} @ {baud} 8N1  (Ctrl+C 退出)")

    # 启动 Modbus 轮询发送线程
    stop_event = threading.Event()
    if modbus_poll:
        frame_bytes = bytes([poll_slave, 0x03,
                             (poll_reg_addr >> 8) & 0xFF, poll_reg_addr & 0xFF,
                             (poll_reg_count >> 8) & 0xFF, poll_reg_count & 0xFF])
        frame_with_crc = frame_bytes + crc16_modbus(frame_bytes)
        print(f"Modbus 轮询：从站 {poll_slave}，"
              f"寄存器 0x{poll_reg_addr:04X}，读 {poll_reg_count} 个寄存器")
        print(f"发送帧：{format_hex(frame_with_crc)}  间隔 100ms")
        t = threading.Thread(target=modbus_poll_sender,
                             args=(ser, poll_slave, poll_reg_addr, poll_reg_count, 0.1, stop_event),
                             daemon=True)
        t.start()

    if alarm_toggle:
        print(f"报警切换：从站 {alarm_slave}，寄存器 0x0004，每 1s 交替 0→1→0")
        t2 = threading.Thread(target=alarm_toggle_sender,
                              args=(ser, alarm_slave, stop_event),
                              daemon=True)
        t2.start()

    print("-" * 50)

    try:
        while True:
            data = ser.read(256)        # 最多一次读 256 字节
            if not data:
                continue

            ts = time.strftime("%H:%M:%S.") + f"{time.time() % 1:.3f}".split(".")[1]

            if raw:
                # 原始模式：直接输出字节到 stdout（适合管道处理）
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
                continue

            if show_hex:
                print(f"[{ts}] HEX: {format_hex(data)}")

            if show_ascii:
                print(f"[{ts}] ASC: {format_ascii(data)}")

            if not show_hex and not show_ascii:
                # 默认：显示十六进制
                print(f"[{ts}] {format_hex(data)}")

    except KeyboardInterrupt:
        print("\n退出。")
    finally:
        stop_event.set()
        if 'ser' in locals() and ser.is_open:
            ser.close()


def main():
    parser = argparse.ArgumentParser(
        description="串口监听工具 — 接收 9600 波特率串口数据并输出到终端",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("port", nargs="?", help="串口设备路径（如 /dev/ttyUSB0）。不指定则列出可用串口")
    parser.add_argument("-b", "--baud", type=int, default=9600, help="波特率（默认：9600）")
    parser.add_argument("--hex", action="store_true", help="十六进制模式显示")
    parser.add_argument("--ascii", action="store_true", help="ASCII 模式显示（不可打印字符替换为 .）")
    parser.add_argument("--raw", action="store_true", help="原始字节输出（不经格式化，适合管道）")
    parser.add_argument("-l", "--list", action="store_true", help="列出可用串口并退出")

    # Modbus 轮询选项
    parser.add_argument("--modbus-poll", action="store_true",
                        help="启用 Modbus 轮询：每 100ms 发送读保持寄存器请求")
    parser.add_argument("--slave", type=int, default=1,
                        help="Modbus 从站地址（默认：1）")
    parser.add_argument("--reg-addr", type=lambda x: int(x, 0), default=0x0000,
                        help="寄存器起始地址（默认：0，支持 0x 十六进制）")
    parser.add_argument("--reg-count", type=int, default=1,
                        help="读取寄存器数量（默认：1）")

    # 报警切换选项
    parser.add_argument("--alarm-toggle", action="store_true",
                        help="每 1s 交替发送报警置位/清除指令（寄存器 0x0004）")
    parser.add_argument("--alarm-slave", type=int, default=1,
                        help="报警指令目标从站地址（默认：1，与 --slave 相同时可省略）")

    args = parser.parse_args()

    if args.list:
        list_available_ports()
        sys.exit(0)

    # 未指定端口时让用户交互选择
    port = args.port
    if not port:
        ports = list_available_ports()
        try:
            idx = int(input("\n选择串口编号：").strip())
            port = ports[idx].device
        except (ValueError, IndexError):
            print("无效选择。")
            sys.exit(1)

    # 同时显示十六进制和 ASCII 的简写
    show_hex = args.hex or not args.ascii      # 默认显示 hex
    show_ascii = args.ascii

    monitor(port, args.baud, show_hex, show_ascii, args.raw,
            modbus_poll=args.modbus_poll,
            poll_slave=args.slave,
            poll_reg_addr=args.reg_addr,
            poll_reg_count=args.reg_count,
            alarm_toggle=args.alarm_toggle,
            alarm_slave=args.alarm_slave)


if __name__ == "__main__":
    main()
