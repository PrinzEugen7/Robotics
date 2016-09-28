from control.matlab import *
import numpy as np
    
def main():
  # システム行列の定義
  A = np.array([[-2, 1],
                [2, -3]])
  Q = np.array([[1, 0],
                [0, 1]])
  # リアプノフ方程式の解Pを計算
  P = lyap(A, Q)
  # 結果表示
  print("リアプノフ方程式の解P=:\n",P)

if __name__ == "__main__":
  main()
