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
  Uo = obsv(C, A) # 可制御性行列の計算
  No = np.linalg.matrix_rank(Uo)  # Ucのランクを計算
  (N, N) = np.matrix(A).shape     # 正方行列Aのサイズ(N*N)
  # 可制御性の判別
  if No == N: return 0            # 可観測
  else: return -1                 # 可観測でない
    
def main():
  # システム行列の定義
  A = np.array([[0, 1],
                [0, -1]])
  B = np.array([[0],
                [1]])
  Q = np.array([[1, 0],
                [0, 1]])
  R = np.array([[1]])
  # システムが可制御・可観測でなければ終了
  if check_ctrb(A, B) == -1 :
    print("システムが可制御でないので終了")
    return 0
  if check_obsv(np.sqrt(Q), A) == -1 :
    print("システムが可観測でないので終了")
    return 0
  # 最適レギュレータの設計
  K, P, e = lqr(A, B, Q, R)
  # 結果表示
  print("リカッチ方程式の解:\n",P)
  print("状態フィードバックゲイン:\n",K)
  print("閉ループ系の固有値:\n",e)

if __name__ == "__main__":
  main()
