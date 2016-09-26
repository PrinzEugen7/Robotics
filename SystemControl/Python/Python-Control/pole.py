from control.matlab import *

def main():
  A = "0 1; -3 -2"
  B = "0; 1"
  C= "2 0"
  D = "0"
  # 状態空間モデルの作成
  sys = ss(A, B, C, D)
  poles = pole(sys)
  print(poles)
  
if __name__ == "__main__":
    main()
