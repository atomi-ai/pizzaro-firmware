""" Run all test.

Usage:
    python smoke_test/run_all_tests.py --verbosity=1
"""
import time

from absl import app
from absl import flags
from absl import logging
import subprocess
import sys
from logging import debug, info

FLAGS = flags.FLAGS
# flags.DEFINE_string('log', 'info', 'Logging level (e.g., debug, info, warning, error, fatal)')


def run_test(task_name, timeout=30):
    max_attempts = 3
    for attempt in range(max_attempts):
        global process
        try:
            logging.info(f"Running test: {task_name}, Attempt: {attempt + 1}")
            process = subprocess.Popen(
                ["cargo", "run", "--package", "integration", "--example", task_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )

            # 记录开始时间
            start_time = time.time()
            for line in iter(process.stdout.readline, ''):
                logging.debug(line.strip())
                if time.time() - start_time > timeout:
                    raise subprocess.TimeoutExpired(process.args, timeout)

                if "[TEST] Done" in line:
                    logging.info(f"Test {task_name} passed.")
                    process.terminate()
                    return True
                elif "[TEST] Failed" in line:
                    logging.info(f"Test {task_name} failed in attempt {attempt + 1}.")
                    process.terminate()
                    break
            process.terminate()

        except subprocess.TimeoutExpired:
            logging.info(f"Test {task_name} timed out on attempt {attempt + 1}.")
            process.kill()
        except Exception as e:
            logging.info(f"Error running test {task_name}: {e}")
            return False

        if attempt < max_attempts - 1:
            logging.info(f"Retrying test: {task_name}")

    return False  # If all attempts fail


def main(argv):
    # logging.set_verbosity(FLAGS.log)
    test_tasks = ["demo_linear_actuator_async"]
    all_passed = 0

    for task in test_tasks:
        test_passed = run_test(task)
        if not test_passed:
            info(f"Test failed: {task}")
            all_passed = 1
        else:
            info(f"Test passed: {task}")

    if all_passed == 0:
        info("All tests passed successfully!")
    else:
        info("Some tests failed.")
    return all_passed


if __name__ == "__main__":
    app.run(main)
