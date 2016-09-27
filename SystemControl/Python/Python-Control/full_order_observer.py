from control.matlab import *
import numpy as np

# 可制御性のチェック
def check_ctrb(A, B):
  Uc = ctrb(A, B) # 可制御性行列の計算
  Nu = np.linalg.matrix_rank(Uc)  # Ucのランクを計算
  (N, N) = np.matrix(A).shape     # 正方行列Aのサイズ(N*N)
  # 可制御性の判別
  if Nu == N: return 0            # 可制御
  else: return -1                 # 可制御でない

  # 可観測性のチェック
def check_obsv(A, C):
  Uo = ctrb(A, C) # 可制御性行列の計算
  No = np.linalg.matrix_rank(Uo)  # Ucのランクを計算
  (N, N) = np.matrix(A).shape     # 正方行列Aのサイズ(N*N)
  # 可制御性の判別
  if No == N: return 0            # 可制御
  else: return -1                 # 可制御でない
    
def main():
  # システム行列の定義
  A = np.array([[1, 1],
                [0, 1]])
  B = np.array([[0],
                [1]])
  C = np.array([[1, 1]])
  #F = np.array([5, 1])
  # システムが可制御・可観測でなければ終了
  if check_ctrb(A, B) and check_obsv(A, B) == -1 : exit
  # 同一次元オブザーバの極
  observer_poles=[-1+1j,-1-1j] 
  # 同一次元オブザーバゲインの設計（状態フィードバックの双対） 
  G = place(A.T, C.T, observer_poles).T
  print("オブザーバゲイン:\n",G)

if __name__ == "__main__":
  main()
