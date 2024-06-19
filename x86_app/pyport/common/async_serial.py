import asyncio

import serial
import serial_asyncio
import struct
from .errors import CliError
import rust_module


class SerialController:
    def __init__(self, port=None, probe=None, baud_rate=115200, timeout=10, with_len=False):
        self.port_name = port if port is not None else self.find_serial_device(probe)
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.with_len = with_len
        self.reader = None
        self.writer = None
        self.command_lock = asyncio.Lock()

    async def __aenter__(self):
        self.reader, self.writer = await serial_asyncio.open_serial_connection(
            url=self.port_name, baudrate=self.baud_rate)
        print(f'SerialController.writer = {self.writer}')
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()

    @staticmethod
    def find_serial_device(probe):
        import serial.tools.list_ports
        try:
            vid, pid = [int(x, 16) for x in probe.split(':')]
        except ValueError:
            raise CliError('Invalid probe format. Expected format is VID:PID.')

        for port in serial.tools.list_ports.comports():
            if port.vid == vid and port.pid == pid:
                return port.device

        raise CliError('Device not found.')

    async def send_and_recv(self, command):
        async with self.command_lock:
            data = rust_module.command_to_binvec(command)
            if len(data) == 0:
                raise CliError(f"Unacceptable command: '{command}'")

            try:
                if self.with_len:
                    dl = len(data)
                    print(f"Send data len: {dl}")
                    self.writer.write(struct.pack('<B', dl))

                print(f"Line: '{command}', data = {data}")
                self.writer.write(bytes(data))
                # self.writer.write(data)
                await self.writer.drain()
                print(f'Data sent')

                buf = await self.reader.read(64)
                len_received = len(buf)
                print(f"Got response data: {buf[:len_received+1]}, len: {len_received}")

                if self.with_len:
                    if len_received == 0 or len_received - 1 != buf[0]:
                        raise CliError("Incorrect response length")
                    resp_data = buf[1:len_received+1]
                else:
                    resp_data = buf[:len_received+1]

                print(f'Got data: {resp_data}')
                return resp_data

            except serial.SerialException as e:
                raise CliError(f"Serial port error: {e}")

            except Exception as e:
                raise CliError(f"Unexpected error: {e}")

    async def send_and_expect(self, command, expected):
        resp = await self.send_and_recv(command)
        if resp != expected:
            raise CliError(f'Unexpected response: {resp}(proto: {proto_binary_to_json(resp)}), expected: {expected}')


def proto_binary_to_json(data):
    resp = rust_module.binvec_to_json(data)
    print(f'data: {data}, convert to proto: {resp}')
    return resp
