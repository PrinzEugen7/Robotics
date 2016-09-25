from control.matlab import *

def main():
  # 伝達関数モデルの作成
  G = tf([1], [1., 3])
  print(G)
  
if __name__ == "__main__":
    main()
