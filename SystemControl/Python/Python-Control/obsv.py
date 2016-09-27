from control.matlab import *
import numpy as np

def main():
  # システム行列の定義
  A = "0 1; -1 -1"
  B = "0; 1"
  C= "1 0"
  D = "0"
  # 状態空間モデルの作成
  sys = ss(A, B, C, D)
  Uo = obsv (A,C) # 可制御性行列の計算
  Nu = np.linalg.matrix_rank(Uo)  # Ucのランクを計算
  (N, N) = np.matrix(A).shape     # 正方行列Aのサイズ(N*N)
  # 可制御性の判別
  if Nu == N: 
    print("システムは可観測である") 
  else: 
    print("システムは可観測でない") 

  
if __name__ == "__main__":
    main()
