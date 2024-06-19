import asyncio
import json
import logging
import threading

from common.async_serial import proto_binary_to_json, SerialController
from common.errors import CliError

HPD_ACK_RESPONSE: bytes = b'\x04\x03'
MMD_ACK_RESPONSE: bytes = b'\x03\x03'
HPD_UNAVAILABLE_RESPONSE: bytes = b'\x07\x22'
MMD_UNAVAILABLE_RESPONSE: bytes = b'\x07\x17\x00'
START_POS = 90
CENTER_POS = 570
CLOSE_TO_CENTER_POS = 520
PP_ZERO_SPD = 180
PP_MAX_SPD = 400
STEPPER_SPD = 500


class PizzaMaker:
    def __init__(self, controller: SerialController):
        self.controller = controller

    async def wait_for_stepper_available(self):
        while True:
            resp = await self.controller.send_and_recv('mmd wait_idle')
            print(f'WaitIdle response: {resp}, proto: {proto_binary_to_json(resp)}')
            if resp != MMD_UNAVAILABLE_RESPONSE:
                break
            await asyncio.sleep(0.1)

    async def wait_for_linear_bull_available(self):
        while True:
            resp = await self.controller.send_and_recv('hpd wait_idle')
            print(f'WaitIdle response: {resp}, proto: {proto_binary_to_json(resp)}')
            if resp != HPD_UNAVAILABLE_RESPONSE:
                break
            await asyncio.sleep(1)

    async def send_and_ack(self, command):
        resp = await self.controller.send_and_recv(command)
        if command.startswith('hpd'):
            ack = HPD_ACK_RESPONSE
        elif command.startswith('mmd'):
            ack = MMD_ACK_RESPONSE
        else:
            ack = None

        if resp != ack:
            raise CliError(f'Unexpected response: {resp}(proto: {proto_binary_to_json(resp)}), expected ACK: {ack}')

    async def system_reset(self):
        await self.send_and_ack(f'mmd home')
        await self.send_and_ack(f'mmd belt_off')
        await self.send_and_ack(f'mmd pp_off')
        await self.send_and_ack(f'mmd pr_off')
        await self.send_and_ack(f'mmd dispenser0_off')
        await self.send_and_ack(f'mmd dispenser1_off')
        await self.wait_for_stepper_available()

    async def make_pie_base(self):
        await self.send_and_ack(f'hpd move_to 56800')
        await self.wait_for_linear_bull_available()
        # TODO(zephyr): report errors if the position isn't close to the target
        # send_command(f'hpd get_pos', s)
        await asyncio.sleep(7)
        await self.send_and_ack(f'hpd move_to 50000')
        await self.wait_for_linear_bull_available()
        await asyncio.sleep(0.5)
        await self.send_and_ack(f'hpd move_to 56800')
        await self.wait_for_linear_bull_available()
        await asyncio.sleep(7)

        await self.send_and_ack(f'hpd home')
        await self.wait_for_linear_bull_available()
        await asyncio.sleep(0.5)

    async def accelerate_pr(self, target_speed):
        for speed in range(100, target_speed + 1, 100):
            await self.send_and_ack(f'mmd pr_spd {speed}')
            await asyncio.sleep(0.2)

    async def add_ketchup(self):
        await self.accelerate_pr(600)

        for y in range(START_POS, CLOSE_TO_CENTER_POS, 30):
            x = - int(1.5 * (PP_ZERO_SPD + (PP_MAX_SPD - PP_ZERO_SPD) * (CENTER_POS - y) / (CENTER_POS - START_POS)))
            print(f"xfguo_ketchup() 5: x = {x}, y = {y}")
            await self.send_and_ack(f'mmd move_to {y}')
            await self.wait_for_stepper_available()
            await self.send_and_ack(f'mmd pp_spd {x}')
            await asyncio.sleep(0.7)
        await self.send_and_ack(f'mmd move_to 200')
        await self.send_and_ack(f'mmd pr_off')
        await self.send_and_ack(f'mmd pp_off')
        await self.wait_for_stepper_available()
        print("Done")

    async def dispenser0_future(self, speed, stop_notifier):
        while not stop_notifier.is_set():
            await self.send_and_ack(f'mmd dispenser0_spd {speed}')
            print(f'Current speed: {speed}')
            speed = -speed
            await asyncio.sleep(30)
        await self.send_and_ack(f'mmd dispenser0_off')

    async def add_cheese(self):
        await self.send_and_ack(f'mmd home')
        await self.send_and_ack(f'mmd belt_off')
        await self.send_and_ack(f'mmd pr_off')
        await self.wait_for_stepper_available()

        stop_dispenser0 = threading.Event()
        f_dispenser0 = asyncio.create_task(self.dispenser0_future(600, stop_dispenser0))

        d1_spd = -300
        (onhold_secs, keep_secs) = (1, 4)
        print(d1_spd, onhold_secs, keep_secs)
        await self.send_and_ack(f'mmd dispenser1_spd {d1_spd}')
        for i in range(1):
            await self.send_and_ack(f'mmd move_to 100')
            await self.send_and_ack(f'mmd belt_spd 50')
            await self.send_and_ack(f'mmd pr_spd 70')
            await asyncio.sleep(onhold_secs)

            await self.send_and_ack(f'mmd move_to 130')
            await self.send_and_ack(f'mmd belt_spd 0')
            await self.send_and_ack(f'mmd pr_spd 100')
            await asyncio.sleep(0.2)
            await self.send_and_ack(f'mmd pr_spd 200')
            await asyncio.sleep(0.2)
            await self.send_and_ack(f'mmd pr_spd 300')
            await asyncio.sleep(0.2)
            await self.send_and_ack(f'mmd pr_spd 400')
            await asyncio.sleep(0.2)
            await self.send_and_ack(f'mmd pr_spd 500')
            await asyncio.sleep(0.2)
            await self.send_and_ack(f'mmd pr_spd 600')
            await asyncio.sleep(0.2)
            await self.send_and_ack(f'mmd pr_spd 700')
            await self.send_and_ack(f'mmd belt_spd 100')
            await asyncio.sleep(2)

        MIN_POS = 100
        PRODUCT = (540 - MIN_POS) * 50 / 75
        for p in range(140, 460, 60):
            # d1_spd = -1000 if d1_spd <= -1000 else d1_spd - 80
            await self.send_and_ack(f'mmd dispenser1_spd {d1_spd}')
            r = 540 - p
            a = (p - MIN_POS) * 2 + 50
            # a = 700
            v_b = round(a * r / PRODUCT) // 10 * 10
            print(d1_spd, p, a, v_b)
            await self.send_and_ack(f'mmd move_to {p}')
            await self.send_and_ack(f'mmd belt_spd 150')
            await self.send_and_ack(f'mmd pr_spd 700')
            await self.wait_for_stepper_available()
            await asyncio.sleep(keep_secs * r / 540)

        await self.send_and_ack(f'mmd move_to 460')
        stop_dispenser0.set()
        await self.send_and_ack(f'mmd dispenser0_off')
        await self.send_and_ack(f'mmd dispenser1_off')
        await asyncio.sleep(5)
        await self.wait_for_stepper_available()

        await self.send_and_ack(f'mmd home')
        await self.send_and_ack(f'mmd belt_off')
        await self.send_and_ack(f'mmd pr_off')
        await self.wait_for_stepper_available()
        await f_dispenser0

