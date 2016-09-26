from control.matlab import *
import numpy as np

def main():
  # システム行列の定義
  A = "0 0; 0 -1"
  B = "1; 1"
  C= "2 0"
  D = "0"
  # 状態空間モデルの作成
  sys = ss(A, B, C, D)
  Uc =ctrb(A, B) # 可制御性行列の計算
  Nu = np.linalg.matrix_rank(Uc)  # Ucのランクを計算
  (N, N) = np.matrix(A).shape     # 正方行列Aのサイズ(N*N)
  # 可制御性の判別
  if Nu == N: 
    print("システムは可制御である") 
  else: 
    print("システムは可制御でない") 

  
if __name__ == "__main__":
    main()
