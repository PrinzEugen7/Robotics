# -*- coding: utf-8 -*-
import numpy as np
from numpy import sin,cos
import matplotlib.pyplot as plt

# 並進行列(x軸方向に並進)
def L(l):
    Li = np.matrix((
        ( 1., 0., l),
        ( 0., 1., 0.),
        ( 0., 0., 1.)
    ));
    return Li

# 回転行列(z軸周りに回転)
def R(th):
    Ri = np.matrix((
        (cos(th), -sin(th), 0.),
        (sin(th),  cos(th), 0.),
        (0., 0., 1.)
    ));
    return Ri

# 順運動学の計算
def arm2_fk(l1, l2, th1, th2):
    # 原点座標（縦ベクトル）
    vec = np.array([[0.],[0.],[1.]] );
    # 順運動学の計算
    (x1, y1, z1) = R(th1)*L(l1)*vec;                # 第1関節の位置
    (x2, y2, z2) = R(th1)*L(l1)*R(th2)*L(l2)*vec;   # 第2関節の位置
    return x1, y1, x2, y2


# グラフの描画
def plot(x, y):
    fn = "Times New Roman"
    # グラフ表示の設定
    fig = plt.figure()
    ax = fig.add_subplot(111, axisbg="w")
    ax.tick_params(labelsize=13)                    # 軸のフォントサイズ
    plt.xlabel("$x [m]$", fontsize=20, fontname=fn)
    plt.ylabel("$y [m]$", fontsize=20, fontname=fn)
    plt.plot(x, y,"-g",lw=5,label="link")           # リンクの描画
    plt.plot(x, y,"or",lw=5, ms=10,label="joint")   # 関節の描画
    plt.xlim(-1.2,1.2)
    plt.ylim(-1.2,1.2)
    plt.grid()
    plt.legend(fontsize=20) # 凡例
    plt.show()


def main():
    # 2リンクアームのパラメータ
    (l1, l2) = (0.5, 0.5);              #リンク1と2の長さ
    (th1, th2) = np.radians((30, 60))   # 第1, 2の関節角度
    # 順運動学の計算
    (x1, y1, x2, y2) = arm2_fk(l1, l2, th1, th2)
    # ロボットアームの描画
    x = (0, x1, x2)
    y = (0, y1, y2)
    plot(x, y)


if __name__ == '__main__':
    main()
