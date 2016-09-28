from control.matlab import *
from matplotlib import pyplot as plt
    
def main():
  # 伝達関数のパラメータ
  num = [2, 5, 1]     # 分子の係数
  den = [1, 2, 3]     # 分母の係数
  sys = tf(num, den)  # 伝達関数モデルの作成
  bode(sys)           # ボード線図のプロット
  plt.show()

if __name__ == "__main__":
  main()
