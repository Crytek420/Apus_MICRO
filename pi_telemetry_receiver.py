#!/usr/bin/env python3
"""
Raspberry Pi Telemetry Receiver for ESP32 Flight Controller

This script receives and decodes telemetry data from the ESP32-S3 flight controller
via UART. It demonstrates the binary protocol implementation on the Pi side.

Hardware Setup:
- Pi GPIO 14 (UART TX) -> ESP32 GPIO 16 (RX)
- Pi GPIO 15 (UART RX) -> ESP32 GPIO 15 (TX)
- Common GND

Usage:
    python3 pi_telemetry_receiver.py [--port /dev/serial0] [--baud 115200]
"""

import serial
import struct
import time
import argparse
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional, Union

# Protocol Constants
PACKET_SYNC1 = 0xAA
PACKET_SYNC2 = 0x55
MAX_PAYLOAD_SIZE = 128

class PacketType(IntEnum):
    """Packet type identifiers"""
    IMU = 0x01
    CRSF_CHANNELS = 0x02
    CRSF_LINK_STATS = 0x03
    SYSTEM_INFO = 0x04
    HEARTBEAT = 0x05

@dataclass
class IMUData:
    """IMU sensor data"""
    acc_x: float
    acc_y: float
    acc_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    mag_x: float
    mag_y: float
    mag_z: float
    roll: float
    pitch: float
    yaw: float
    temperature: float
    timestamp_ms: int

@dataclass
class CRSFChannels:
    """CRSF RC channel data"""
    channels: list[int]  # 16 channels
    timestamp_ms: int
    failsafe: bool

@dataclass
class LinkStats:
    """CRSF link statistics"""
    uplink_rssi_ant1: int
    uplink_rssi_ant2: int
    uplink_link_quality: int
    uplink_snr: int
    active_antenna: int
    rf_mode: int
    uplink_tx_power: int
    downlink_rssi: int
    downlink_link_quality: int
    downlink_snr: int
    timestamp_ms: int

@dataclass
class SystemInfo:
    """ESP32 system information"""
    cpu_temp: float
    supply_voltage: float
    free_heap: int
    uptime_ms: int
    cpu_usage: int
    timestamp_ms: int

@dataclass
class Heartbeat:
    """Heartbeat packet"""
    sequence: int
    timestamp_ms: int


class TelemetryReceiver:
    """ESP32 telemetry receiver for Raspberry Pi"""
    
    def __init__(self, port: str = '/dev/serial0', baudrate: int = 115200):
        """
        Initialize the telemetry receiver
        
        Args:
            port: Serial port path (default: /dev/serial0 for Pi UART0)
            baudrate: Baud rate (default: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.packet_count = 0
        self.error_count = 0
        
        # Statistics
        self.last_imu = None
        self.last_channels = None
        self.last_link_stats = None
        self.last_system_info = None
        self.last_heartbeat = None
        
    def connect(self) -> bool:
        """
        Connect to the serial port
        
        Returns:
            True if successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the serial port"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")
    
    @staticmethod
    def calculate_crc16(data: bytes) -> int:
        """
        Calculate CRC16-CCITT checksum
        
        Args:
            data: Data to calculate checksum for
            
        Returns:
            CRC16 checksum
        """
        crc = 0xFFFF
        
        for byte in data:
            crc ^= byte << 8
            
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                    
                crc &= 0xFFFF  # Keep it 16-bit
        
        return crc
    
    def read_packet(self) -> Optional[tuple[PacketType, bytes]]:
        """
        Read and validate a packet from the serial port
        
        Returns:
            Tuple of (packet_type, payload) or None if no valid packet
        """
        if not self.serial or not self.serial.is_open:
            return None
        
        # Find sync bytes
        while True:
            byte = self.serial.read(1)
            if not byte:
                return None
            
            if byte[0] == PACKET_SYNC1:
                byte = self.serial.read(1)
                if byte and byte[0] == PACKET_SYNC2:
                    break
        
        # Read header (type and length)
        header = self.serial.read(2)
        if len(header) < 2:
            self.error_count += 1
            return None
        
        packet_type = header[0]
        payload_length = header[1]
        
        if payload_length > MAX_PAYLOAD_SIZE:
            self.error_count += 1
            return None
        
        # Read payload
        payload = self.serial.read(payload_length)
        if len(payload) < payload_length:
            self.error_count += 1
            return None
        
        # Read CRC
        crc_bytes = self.serial.read(2)
        if len(crc_bytes) < 2:
            self.error_count += 1
            return None
        
        received_crc = struct.unpack('<H', crc_bytes)[0]
        
        # Calculate and verify CRC
        packet_data = bytes([PACKET_SYNC1, PACKET_SYNC2, packet_type, payload_length]) + payload
        calculated_crc = self.calculate_crc16(packet_data)
        
        if received_crc != calculated_crc:
            self.error_count += 1
            print(f"CRC mismatch: received {received_crc:04X}, calculated {calculated_crc:04X}")
            return None
        
        self.packet_count += 1
        return (PacketType(packet_type), payload)
    
    def parse_imu(self, payload: bytes) -> Optional[IMUData]:
        """Parse IMU data packet"""
        if len(payload) < 56:  # 13 floats + 1 uint32 = 56 bytes
            return None
        
        data = struct.unpack('<13fI', payload)
        return IMUData(
            acc_x=data[0], acc_y=data[1], acc_z=data[2],
            gyro_x=data[3], gyro_y=data[4], gyro_z=data[5],
            mag_x=data[6], mag_y=data[7], mag_z=data[8],
            roll=data[9], pitch=data[10], yaw=data[11],
            temperature=data[12],
            timestamp_ms=data[13]
        )
    
    def parse_crsf_channels(self, payload: bytes) -> Optional[CRSFChannels]:
        """Parse CRSF channel data packet"""
        if len(payload) < 37:  # 16 uint16 + 1 uint32 + 1 uint8 = 37 bytes
            return None
        
        data = struct.unpack('<16HIB', payload)
        return CRSFChannels(
            channels=list(data[0:16]),
            timestamp_ms=data[16],
            failsafe=bool(data[17])
        )
    
    def parse_link_stats(self, payload: bytes) -> Optional[LinkStats]:
        """Parse link statistics packet"""
        if len(payload) < 14:  # 10 uint8/int8 + 1 uint32 = 14 bytes
            return None
        
        data = struct.unpack('<BBBbBBBBBbI', payload)
        return LinkStats(
            uplink_rssi_ant1=data[0],
            uplink_rssi_ant2=data[1],
            uplink_link_quality=data[2],
            uplink_snr=data[3],
            active_antenna=data[4],
            rf_mode=data[5],
            uplink_tx_power=data[6],
            downlink_rssi=data[7],
            downlink_link_quality=data[8],
            downlink_snr=data[9],
            timestamp_ms=data[10]
        )
    
    def parse_system_info(self, payload: bytes) -> Optional[SystemInfo]:
        """Parse system info packet"""
        if len(payload) < 21:  # 2 floats + 3 uint32 + 1 uint8 = 21 bytes
            return None
        
        data = struct.unpack('<ffIIBI', payload)
        return SystemInfo(
            cpu_temp=data[0],
            supply_voltage=data[1],
            free_heap=data[2],
            uptime_ms=data[3],
            cpu_usage=data[4],
            timestamp_ms=data[5]
        )
    
    def parse_heartbeat(self, payload: bytes) -> Optional[Heartbeat]:
        """Parse heartbeat packet"""
        if len(payload) < 8:  # 2 uint32 = 8 bytes
            return None
        
        data = struct.unpack('<II', payload)
        return Heartbeat(
            sequence=data[0],
            timestamp_ms=data[1]
        )
    
    def process_packet(self, packet_type: PacketType, payload: bytes):
        """Process a received packet"""
        try:
            if packet_type == PacketType.IMU:
                self.last_imu = self.parse_imu(payload)
                if self.last_imu:
                    print(f"[IMU] Roll: {self.last_imu.roll:6.1f}° "
                          f"Pitch: {self.last_imu.pitch:6.1f}° "
                          f"Yaw: {self.last_imu.yaw:6.1f}° "
                          f"Temp: {self.last_imu.temperature:4.1f}°C")
            
            elif packet_type == PacketType.CRSF_CHANNELS:
                self.last_channels = self.parse_crsf_channels(payload)
                if self.last_channels:
                    ch = self.last_channels.channels
                    fs = " [FAILSAFE]" if self.last_channels.failsafe else ""
                    print(f"[CRSF] CH0-3: {ch[0]:4d} {ch[1]:4d} {ch[2]:4d} {ch[3]:4d}{fs}")
            
            elif packet_type == PacketType.CRSF_LINK_STATS:
                self.last_link_stats = self.parse_link_stats(payload)
                if self.last_link_stats:
                    print(f"[LINK] RSSI: {self.last_link_stats.uplink_rssi_ant1:3d} dBm, "
                          f"LQ: {self.last_link_stats.uplink_link_quality:3d}%, "
                          f"SNR: {self.last_link_stats.uplink_snr:+3d} dB")
            
            elif packet_type == PacketType.SYSTEM_INFO:
                self.last_system_info = self.parse_system_info(payload)
                if self.last_system_info:
                    uptime_sec = self.last_system_info.uptime_ms / 1000
                    free_kb = self.last_system_info.free_heap / 1024
                    print(f"[SYS] Uptime: {uptime_sec:.1f}s, Free Heap: {free_kb:.1f} KB")
            
            elif packet_type == PacketType.HEARTBEAT:
                self.last_heartbeat = self.parse_heartbeat(payload)
                if self.last_heartbeat:
                    print(f"[HB] Seq: {self.last_heartbeat.sequence:5d}, "
                          f"Time: {self.last_heartbeat.timestamp_ms} ms")
        
        except Exception as e:
            print(f"Error processing packet: {e}")
            self.error_count += 1
    
    def run(self):
        """Main receive loop"""
        print("Starting telemetry receiver...")
        print("Press Ctrl+C to exit\n")
        
        try:
            while True:
                packet = self.read_packet()
                if packet:
                    packet_type, payload = packet
                    self.process_packet(packet_type, payload)
                else:
                    time.sleep(0.001)  # Small delay if no packet
        
        except KeyboardInterrupt:
            print("\n\nStopping telemetry receiver...")
            print(f"Statistics: {self.packet_count} packets received, {self.error_count} errors")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Raspberry Pi telemetry receiver for ESP32 flight controller'
    )
    parser.add_argument(
        '--port',
        default='/dev/serial0',
        help='Serial port (default: /dev/serial0)'
    )
    parser.add_argument(
        '--baud',
        type=int,
        default=115200,
        help='Baud rate (default: 115200)'
    )
    
    args = parser.parse_args()
    
    receiver = TelemetryReceiver(port=args.port, baudrate=args.baud)
    
    if receiver.connect():
        receiver.run()
        receiver.disconnect()
    else:
        print("Failed to connect to serial port")
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
