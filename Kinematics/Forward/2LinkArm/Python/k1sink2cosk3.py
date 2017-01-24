# -*- coding: utf-8 -*-
from math import *

def main():

    k1=1
    k2=2
    k3=2
    # 角度計算
    theta_p = atan2(k1,k2) + atan2(sqrt(k1^2+k2^2-k3^2),k3)
    theta_m = atan2(k1,k2) - atan2(sqrt(k1^2+k2^2-k3^2),k3)
    # ラジアンからdegに変換
    theta_p = degrees(theta_p)
    theta_m = degrees(theta_m)
    # 計算結果表示
    print u"θ+ = " + str(theta_p)
    print u"θ- = " + str(theta_m)

if __name__ == '__main__':
    main()
