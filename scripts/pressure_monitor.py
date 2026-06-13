#!/usr/bin/env python3
"""
风压传感器串口监测工具 — 接收 115200 波特率数据并解析风压值。

监测风压传感器（STC8H1K28）UART1 与风压芯片之间的通信。
风压芯片返回帧格式：FE <压力高> <压力低> <4字节忽略> DC

用法：
    ./pressure_monitor.py                             # 列出可用串口，让用户选择
    ./pressure_monitor.py /dev/ttyUSB0                # 指定串口设备
    ./pressure_monitor.py /dev/ttyUSB0 --hex          # 仅十六进制显示（不解析）
    ./pressure_monitor.py /dev/ttyUSB0 --raw          # 原始字节输出
    ./pressure_monitor.py --help                      # 查看所有选项
"""

import argparse
import sys
import time
import serial
from serial.tools import list_ports


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
    return " ".join(f"{b:02X}" for b in data)


def parse_pressure_frame(data: bytes) -> int:
    """
    解析风压芯片数据帧。

    帧格式：
        FE <hi> <lo> <xx> <xx> <xx> <xx> DC
        ^                              ^
        |— 起始字节                   |— 结束字节

    返回解析出的压力值，非完整帧返回 -1。
    """
    if len(data) < 3:
        return -1

    # 查找帧起始 FE
    start = data.find(b'\xFE')
    if start < 0:
        return -1

    # 帧至少需要 7 字节：FE + hi + lo + 4 个忽略 + DC
    if len(data) - start < 7:
        return -1

    # 查找帧结束 DC
    end = data.find(b'\xDC', start + 1)
    if end < 0:
        return -1

    # 检查是否是完整帧（FE 到 DC 之间 6 字节）
    payload = data[start + 1:end]
    if len(payload) != 6:
        return -1

    hi = payload[0]
    lo = payload[1]
    value = (hi << 8) | lo

    # 有效范围 0-999
    if value > 999:
        return -1

    return value


def monitor(port: str, baud: int, show_hex: bool, show_ascii: bool, raw: bool):
    """打开串口并持续监听数据"""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
    except serial.SerialException as e:
        print(f"打开串口失败：{e}")
        sys.exit(1)

    print(f"监听 {port} @ {baud} 8N1  (Ctrl+C 退出)")
    print("-" * 50)

    # 帧拼合缓冲（风压芯片帧可能被多次 read 分割）
    buf = b""

    try:
        while True:
            data = ser.read(256)
            if not data:
                continue

            ts = time.strftime("%H:%M:%S.") + f"{time.time() % 1:.3f}".split(".")[1]

            if raw:
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
                continue

            if show_hex:
                print(f"[{ts}] HEX: {format_hex(data)}")
                continue

            if show_ascii:
                readable = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
                print(f"[{ts}] ASC: {readable}")
                continue

            # 默认模式：解析风压值
            buf += data

            # 从缓冲中提取所有完整帧
            while True:
                value = parse_pressure_frame(buf)
                if value < 0:
                    # 缓冲过长但没有找到完整帧 → 丢弃前部冗余字节
                    if len(buf) > 64:
                        fe_idx = buf.find(b'\xFE')
                        if fe_idx > 0:
                            buf = buf[fe_idx:]
                        else:
                            buf = b""
                    break

                # 找到完整帧 → 输出并移除
                fe_idx = buf.find(b'\xFE')
                end_idx = buf.find(b'\xDC', fe_idx) + 1
                frame = buf[fe_idx:end_idx]
                buf = buf[end_idx:]

                print(f"[{ts}] 风压 = {value:3d} Pa  "
                      f"帧: {format_hex(frame)}")

    except KeyboardInterrupt:
        print("\n退出。")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


def main():
    parser = argparse.ArgumentParser(
        description="风压传感器串口监测工具 — 接收 115200 波特率数据并解析风压值",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("port", nargs="?",
                        help="串口设备路径（如 /dev/ttyUSB0）。不指定则列出可用串口")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        help="波特率（默认：115200）")
    parser.add_argument("--hex", action="store_true",
                        help="十六进制模式显示（不解析风压值）")
    parser.add_argument("--ascii", action="store_true",
                        help="ASCII 模式显示")
    parser.add_argument("--raw", action="store_true",
                        help="原始字节输出（不经格式化，适合管道）")
    parser.add_argument("-l", "--list", action="store_true",
                        help="列出可用串口并退出")

    args = parser.parse_args()

    if args.list:
        list_available_ports()
        sys.exit(0)

    port = args.port
    if not port:
        ports = list_available_ports()
        try:
            idx = int(input("\n选择串口编号：").strip())
            port = ports[idx].device
        except (ValueError, IndexError):
            print("无效选择。")
            sys.exit(1)

    monitor(port, args.baud, args.hex, args.ascii, args.raw)


if __name__ == "__main__":
    main()
