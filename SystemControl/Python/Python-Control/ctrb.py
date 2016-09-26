from control.matlab import *
import numpy as np

def main():
  A = "0 0; 0 -1"
  B = "1; 1"
  C= "2 0"
  D = "0"
  # 状態空間モデルの作成
  sys = ss(A, B, C, D)
  Uc =ctrb(A,B) # 
  Nu = np.linalg.matrix_rank(Uc)  # Ucのランクを計算
  N = 2 # 正方行列Aのサイズ(2*2)
  if Nu == N: 
    print("システムは可制御である") 
  else: 
    print("システムは可制御でない") 

  
if __name__ == "__main__":
    main()
