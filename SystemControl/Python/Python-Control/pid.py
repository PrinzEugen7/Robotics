from control.matlab import *
from matplotlib import pyplot as plt
    
def main():
  # PID制御器のパラメータ
  Kp = 0.6  # 比例
  Ki = 0.03 # 積分
  Kd = 0.03 # 微分
  num = [Kd, Kp, Ki]
  den = [1, 0]
  K = tf(num, den)
  # 制御対象
  Kt = 1
  J = 0.01
  C = 0.1
  num = [Kt]
  den = [J, C, 0]
  G = tf(num, den)
  # フィードバックループ
  sys = feedback(K*G, 1)
  t = np.linspace(0, 3, 1000)
  y, T = step(sys, t)
  plt.plot(T, y)
  ｐｌｔ.grid()
  plt.axhline(1, color="b", linestyle="--")
  plt.xlim(0, 3)
  
if __name__ == "__main__":
  main()
