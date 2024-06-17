import asyncio
import pytest
from unittest import mock
from pyport.common.async_serial import SerialController, CliError


class DummySerialDevice:
    def __init__(self):
        self.received_data = b''
        self.to_send = b''

    def write(self, data):
        if isinstance(data, list):
            data = bytes(data)
        self.received_data += data
        if self.received_data == b'\x02\x04\x01':
            self.to_send = b'\x02\x04\x02'
        return len(data)

    async def read(self, size):
        if self.to_send:
            data = self.to_send[:size]
            self.to_send = self.to_send[size:]
            return data
        return b''

    async def drain(self):
        pass

    def close(self):
        pass

    async def wait_closed(self):
        pass


@pytest.mark.asyncio
async def test_send_and_recv():
    dummy_device = DummySerialDevice()

    with mock.patch('serial_asyncio.open_serial_connection', return_value=(dummy_device, dummy_device)):
        async with SerialController(port='dummy_port', with_len=True) as controller:
            resp = await controller.send_and_recv("hpd ping")
            expected_response = b'\x04\x02'
            assert resp == expected_response


@pytest.mark.asyncio
async def test_send_and_expect():
    dummy_device = DummySerialDevice()

    with mock.patch('serial_asyncio.open_serial_connection', return_value=(dummy_device, dummy_device)):
        async with SerialController(port='dummy_port', with_len=True) as controller:
            await controller.send_and_expect("hpd ping", b'\x04\x02')

if __name__ == "__main__":
    pytest.main()
