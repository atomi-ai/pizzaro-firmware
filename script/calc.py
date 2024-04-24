def calc(r_inner=510, r_outer=200, r_offset=27, ratio=83.6/400, n=5, max_rpm=300, line_spd=16500, delay_c=1, revert=False, debug=False):
    """计算等速同心圆
    r_inner: 内圈位置(pulse)
    r_outer：外圈位置(pulse)
    r_offset: 内圈与圆心的偏置距离(mm)
    ratio: 实际距离与脉冲数的转换: mm/pulse
    n: 切成多少道环
    max_rpm: 允许的最高转速
    line_spd: 线速度
    delay_c: 延迟系数
    revert: True=>从外到内, False=>从内到外
    debug: debug输出还是rust脚本
    """

    for i in range(n) if not revert else range(n-1, -1, -1):
        r = abs(r_outer-r_inner) * i/n * ratio +  r_offset
        # print(r_min, (r_max-r_min), i, n)
        c = r*2*3.14
        rpm = line_spd / r
        delay = c/line_spd # 周长除以线速度得到时间
        # rpm = delay*r     # 速度 = 时间*半径*系数

        if rpm > max_rpm:
            print("WARNING: rpm too high")

        pos = max(r_inner, r_outer) - abs(r_outer-r_inner) * i/(n-1)
        if not debug:
            print("""
        self.mmd_move_to({pos}, 500).await?;
        self.mmd_pr({rpm}).await?;
        self.wait_for_stepper_available().await?;
            Delay::new({delay}.millis()).await;""".format(delay=int(delay*1000*delay_c), pos=int(pos), rpm=int(rpm)))
        else:
            print("move_to:{}, r:{}, spd:{}, delay:{}".format(pos, r, rpm, delay*delay_c))

if __name__ == "__main__":
    # 番茄酱
    # offset=27mm
    # 260.6 - 240-198 -177
    # 42mm ~ 200 pulse
    # 63mm ~ 300 pulse
    # 62.6mm ~ 300 pulse
    # 83.6mm ~ 400 pulse
    # pos_ratio = 83.6/400

    # 番茄酱
    #calc(n=4, max_rpm=600, delay_c=220, line_spd=16500, revert=True, debug=False)
    # 起司
    calc(n=3, max_rpm=800, delay_c=220, line_spd=20500, revert=False, debug=False )
    
