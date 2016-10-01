from control.matlab import *
import numpy as np

def main():
  # システム行列の定義
  A = np.array([[0, 1],
                [0, -1]])
  B = np.array([[0],
                [1]])
  Q = np.array([[1, 0],
                [0, 1]])
  R = np.array([[1]])

  # LQRでリカッチ方程式の解Pを計算
  K, P, e = lqr(A, B, Q, R)
  # 結果表示
  print("リカッチ方程式の解:\n",P)

if __name__ == "__main__":
main()
