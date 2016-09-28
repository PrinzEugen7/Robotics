from control.matlab import *
import numpy as np
    
def main():
  # システム行列の定義
  A = np.array([[0, 1, 0],
	       [0, -1, 1],
	       [0, 0, 0]])
  B = np.array([[0],
	       [0],
	       [1]])
  C = np.array([[1, 0, 0]])
  Q = C.T*C
  R = np.array([[1]])
  # リカッチ方程式を解く
  K, P, e = lqr(A, B, Q, R)
  Z = np.array([[0, 1, 0],
                [0, -1, 1],
                [1, 0, 0]])
  K = K.dot(np.linalg.inv(Z))
  K1 = [K[0][0], K[0][1]]
  K2 = K[0][2]
  # 結果表示
  print("K1 =\n", K1)
  print("K2 =\n", K2)

if __name__ == "__main__":
  main()
