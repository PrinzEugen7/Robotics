# -*- coding:utf-8 -*-
from control.matlab import *

def main():
  A = "0 1; -1 -1"
  B = "0; 1"
  C= "2 0"
  D = "0"
  // 状態空間モデルの作成
  sys = ss(A, B, C, D)
  print(sys)
  
if __name__ == "__main__":
  main()
