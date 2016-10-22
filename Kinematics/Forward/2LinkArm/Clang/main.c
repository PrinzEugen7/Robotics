#include <stdio.h>
#include <math.h>
 
#define PI 3.14159265
 
// 順運動学の計算(リンク1の長さ, リンク2の長さ, 関節1の角度, 関節2の角度, 手先位置x, 手先位置y)
void forwardKinematics(double l1, double l2, double deg1, double deg2, double *x, double *y){
	// 度をラジアンに変換
    double rad1 = deg1*(PI/180);
    double rad2 = deg2*(PI/180);
    // 手先位置を順運動学の式で算出
	*x = cos(rad1)+cos(rad1+rad2);
	*y = sin(rad1)+sin(rad1+rad2);
}
 
int main(void)
{
	// 変数の定義：手先位置
	double x, y;
	// 順運動学の計算(2リンク)
	forwardKinematics(0.5, 0.6, 30, 40, &x, &y);
	// 計算結果
	printf("(x, y)=(%lf, %lf)", x, y);
}
