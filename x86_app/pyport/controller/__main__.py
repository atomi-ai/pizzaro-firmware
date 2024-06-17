import asyncio
import sys

from absl import app, flags, logging
from absl.logging import info, error
from common.async_serial import proto_binary_to_json, SerialController

FLAGS = flags.FLAGS

# Define command line arguments
flags.DEFINE_string('port', None, 'The serial port to use')
flags.DEFINE_string('probe', None, 'The probe to use for finding the device')
flags.DEFINE_bool('with_len', False, 'Flag to indicate if the length should be included')


async def exp():
    async with SerialController(port=FLAGS.port, probe=FLAGS.probe, with_len=True) as controller:
        info(f'Controller: {controller}')
        await controller.send_and_expect(f'hpd ping', b'\x04\x02')


def main(argv):
    logging.set_verbosity(logging.INFO)

    # Check for required arguments
    if not FLAGS.port and not FLAGS.probe:
        error('Either --port or --probe must be provided.')
        sys.exit(1)

    info(f'Port: {FLAGS.port}, Probe: {FLAGS.probe}, With length: {FLAGS.with_len}')
    asyncio.run(exp())


if __name__ == '__main__':
    app.run(main)
