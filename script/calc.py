def calc(r_min, r_max, n=5, spd_init=300, delay_a=1, revert=False, debug=False):
    """计算等速螺旋线
    r_min: 最小半径
    r_max: 最大半径
    n: 切成多少道环
    spd_init: 初始转动速度
    delay_a: 延迟时间系数
    """
    for i in range(1, n+1) if not revert else range(n, 0, -1):
        r1 = (r_max-r_min) * i/n +r_min
        # print(r_min, (r_max-r_min), i, n)
        c = r1*2*3.14
        delay = c/delay_a       # 在这一圈等待的时间与周长成正比
        spd = i / n * spd_init

        pos = r_max - r1 + r_min + 1/n * (r_max-r_min)
        if not debug:
            print("""
        self.mmd_move_to({pos}, 500).await?;
        self.mmd_pr({spd}).await?;
        self.wait_for_linear_stepper_available().await?;
            Delay::new({delay}.millis()).await;""".format(delay=int(delay*1000), pos=int(pos), spd=int(spd)))
        else:
            print("move_to:{}, spd:{}, delay:{}".format(pos, spd, delay))

if __name__ == "__main__":
    # 番茄酱
    calc(r_min=140, r_max=500, n=6, spd_init=400, delay_a=900, revert=True, debug=False)
    # 起司
    #calc(r_min=10, r_max=500, n=3, delay_a=1000)
    
