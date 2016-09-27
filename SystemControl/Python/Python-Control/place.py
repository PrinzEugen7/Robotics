from control.matlab import *
import numpy as np

def check_ctrb(A, B):
  Uc = ctrb(A, B) # 可制御性行列の計算
  Nu = np.linalg.matrix_rank(Uc)  # Ucのランクを計算
  (N, N) = np.matrix(A).shape     # 正方行列Aのサイズ(N*N)
  # 可制御性の判別
  if Nu == N: return 0            # 可制御
  else: return -1                 # 可制御でない
    
def main():
  # システム行列の定義
  A = np.array([[1, 0],
                [0, 2]])
  B = np.array([[1],
                [1]])
  poles = [-2, -3]
  # システムが可制御でなければ終了
  if check_ctrb(A, B) == -1 : exit
  # 虚数の定義
  i = np.sqrt(-1)
  poles = [-1,-2]
  F = place(A, B, poles)
  print("ゲイン:", F)
  print("設計したゲインの極:", np.linalg.eigvals(A-B*F))

if __name__ == "__main__":
  main()
