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
def Rz(th):
    R = np.matrix((
        (cos(th), -sin(th), 0.),
        (sin(th),  cos(th), 0.),
        (0., 0., 1.)
    ));
    return R

# 座標変換行列の微係数(z軸周り)
def dRz(th):
    dR = np.matrix( (
        (-sin(th),-cos(th),0.),
        ( cos(th),-sin(th),0.),
        (0.,0.,1.)
    ));
    return dR

# グラフの描画
def plot(x, y, X):
    fn = "Times New Roman"
    # グラフ表示の設定
    plt.xlabel("$x [m]$", fontsize=20, fontname=fn)
    plt.ylabel("$y [m]$", fontsize=20, fontname=fn)
    plt.plot(x, y,"-g",lw=5,label="link")           # リンクの描画
    plt.plot(x, y,"or",lw=5, ms=10,label="joint")   # 関節の描画
    plt.plot(X[0], X[1],"ob",lw=5, ms=10)   # 関節の描画
    #line.set_ydata(y)
    plt.xlim(-1.2,1.2)
    plt.ylim(-1.2,1.2)
    plt.grid()
    plt.legend(fontsize=20) # 凡例
    plt.draw()
    plt.clf()

def arm2_ik((x, y), (l1, l2), (th1,th2)):
    X = np.matrix(((np.array(x)),(np.array(y))));
    # 収束計算を50回繰り返す
    for j in range(20):
        # 原点座標(縦ベクトル)
        vec = np.array([[0.],[0.],[1.]] );
        P = np.matrix((
            (1.,0.,0.),
            (0.,1.,0.),
            ));
        # 現在の手先位置を求める
        Xg = P*Rz(th1)*L(l1)*Rz(th2)*L(l2)*vec;
        # ヤコビ行列を求める
        J1 = dRz(th1)*L(l1)*vec;
        J2 = Rz(th1)*L(l1)*dRz(th2)*L(l2)*vec;
        JJ = np.c_[J1,J2];                     # 3つの列ベクトルを連結する
        J = P*JJ;                              #ヤコビ行列
        invJ = J.T*np.linalg.inv(J*J.T);       #ヤコビ行列の逆行列
        dx = X - Xg;           #位置の変位量
        th = 0.1*invJ*dx;      #逆運動学の式
        th1 = th1 + th[0,0];
        th2 = th2 + th[1,0];
    return th1, th2

# 順運動学の計算
def arm2_fk((l1,l2), (th1,th2)):
    vec = np.array([[0.],[0.],[1.]] );
    (x1, y1, z1) = Rz(th1)*L(l1)*vec;                # 第1関節の位置
    (x2, y2, z2) = Rz(th1)*L(l1)*Rz(th2)*L(l2)*vec;  # 第2関節の位置
    return x1, y1, x2, y2

# メイン
def main():
    # パラメータ
    L = (0.6, 0.6)  # リンク1, 2の長さ
    th = (np.pi/2, np.pi/2)   # 初期関節角度(仮の解)
    X = np.array([0.6,0.4])
    dX = np.array([-0.01,-0.01])
    # インタラクティブモード
    plt.ion()
    while(1):
        # 逆運動学の計算
        th = arm2_ik(X, L, th)
        # 順運動学の計算
        (x1, y1, x2, y2) = arm2_fk(L, th)
        # ロボットアームの描画
        x = (0, x1, x2)
        y = (0, y1, y2)
        plot(x, y, X)
        X = X + dX
        if(X[0] < 0):
            plt.close()
            break

if __name__ == '__main__':
    main()
