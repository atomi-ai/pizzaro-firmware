import subprocess
import os


# 烧写函数，返回命令执行结果
def burn_binary(binary_name, probe_id):
    cmd = f'cargo flash --release --target thumbv6m-none-eabi --chip RP2040 --probe {probe_id} --protocol swd --package integration --bin {binary_name}'
    print(f'Run command: "{cmd}"')
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode == 0:
        return f"{binary_name} 烧写成功"
    else:
        return f"{binary_name} 烧写失败: {result.stderr}"


def run_tests(tests, probe="16c0:27dd"):
    # 保存当前目录
    current_dir = os.getcwd()

    try:
        # 切换到x86_app目录
        os.chdir("x86_app")
        for test in tests:
            print(f"Running test: {test}")
            # 构建并运行测试命令
            cmd = f"cargo run --example {test} -- --probe {probe}"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            if result.returncode == 0:
                print(f"Test {test} passed")
            else:
                print(f"Test {test} failed with error: {result.stderr}")
    finally:
        # 切换回原始目录
        os.chdir(current_dir)


def main():
    binaries = {
        "mc_main": "2e8a:000c:4150335631373503",
        "mmd_main": "2e8a:000c:65D77F96043E4858",
        "hpd_main": "2e8a:000c:65CD5C1714224E53",
    }

    for binary, probe in binaries.items():
        print(burn_binary(binary, probe))

    run_tests(["ping_test"])
    print("All done")


if __name__ == "__main__":
    main()
