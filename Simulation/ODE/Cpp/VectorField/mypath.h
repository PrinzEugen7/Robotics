#include <opencv2/opencv.hpp>
#include <opencv2/opencv_lib.hpp>
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include "mytxt.h"

Txt path, obst, wall; // テキストファイルの処理
using namespace cv;
using namespace std;
const int w = 100, h = 100;			// 地図のサイズ
static int MAP[w][h];
static int closed_map[w][h];			// Closedノード用マップ
static int open_map[w][h] = {0};		// Openノード用マップ
static int dir_map[w][h] = {0};		// 方向用マップ
const int dir = 8;					// 移動可能な方向数
static int dx[dir] = { 1, 1, 0, -1, -1, -1, 0, 1 };
static int dy[dir] = { 0, 1, 1, 1, 0, -1, -1, -1 };

class node
{
	int xp, yp;		// 現在位置
	int level;		// 移動した距離
	int priority;  	// 優先度

public:
	node(int xP, int yP, int d, int p)
	{
		xp = xP; yp = yP; level = d; priority = p;
	}

	int getxPos() const { return xp; }
	int getyPos() const { return yp; }
	int getLevel() const { return level; }
	int getPriority() const { return priority; }

	void updatePriority(const int & xDest, const int & yDest)
	{
		priority = level + estimate(xDest, yDest) * 10; //A*
	}

	// 斜め方向へ進む代わりにより良い優先順位を与える
	void nextLevel(const int & i) 
	{
		level += (dir == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
	}

	// ゴールまでの推定距離を求める関数
	const int & estimate(const int & dx, const int & dy) const
	{	// ユークリッド距離
		static int d = static_cast<int>(sqrt((dx - xp)*(dx - xp) + (dy - yp)*(dy - yp)*1.0)); 
		return(d);
	}
};

// 優先順位の決定
bool operator<(const node & a, const node & b)
{
	return a.getPriority() > b.getPriority();
}

// A-Starアルゴリズムによる経路探索
string pathFind(const int & x1, const int & y1, const int & x2, const int & y2)
{
	static priority_queue<node> pq[2];
	static int pqi;
	static node* w0;
	static node* h0;
	static int i, j, x, y, xdx, ydy;
	static char c;
	pqi = 0;
	// 開始ノードを作成しOpenノードの配列に挿入
	w0 = new node(x1, y1, 0, 0);
	w0->updatePriority(x2, y2);
	pq[pqi].push(*w0);
	open_map[x][y] = w0->getPriority(); // Openノードマップ上にマーク
	// A-Statで探索
	while (!pq[pqi].empty()){
		// Openノードの配列から最も優先度の高い現在のノードを取得
		w0 = new node(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), pq[pqi].top().getLevel(), pq[pqi].top().getPriority());
		x = w0->getxPos(); y = w0->getyPos();
		pq[pqi].pop();				// オープンリストからノードを削除
		open_map[x][y] = 0;
		closed_map[x][y] = 1;			// 閉じたノードマップ上にマーク
		// ゴールまで到達したら探索終了
		if (x == x2 && y == y2){
			string path = "";
			while (!(x == x1 && y == y1)){
				j = dir_map[x][y];
				c = '0' + (j + dir / 2) % dir;
				path = c + path;
				x += dx[j];
				y += dy[j];
			}
			delete w0;
			while (!pq[pqi].empty()) pq[pqi].pop();		// 残りのノードを空にする
			return path;
		}
		// 移動可能な全方向に移動する子ノードを生成
		for (i = 0; i<dir; i++){
			xdx = x + dx[i]; ydy = y + dy[i];
			// 子ノードを作成
			if (!(xdx<0 || xdx>w - 1 || ydy<0 || ydy>h - 1 || MAP[xdx][ydy] == 1 || closed_map[xdx][ydy] == 1)){
				h0 = new node(xdx, ydy, w0->getLevel(), w0->getPriority());
				h0->nextLevel(i);
				h0->updatePriority(x2, y2);
				// オープンリストにない場合は追加
				if (open_map[xdx][ydy] == 0){
					open_map[xdx][ydy] = h0->getPriority();
					pq[pqi].push(*h0);
					dir_map[xdx][ydy] = (i + dir / 2) % dir;	// 親ノードの方向をマーク
				}
				else if (open_map[xdx][ydy]>h0->getPriority()){
					open_map[xdx][ydy] = h0->getPriority();		// 優先度の情報を更新
					dir_map[xdx][ydy] = (i + dir / 2) % dir;	// 親ノードの情報を更新

					while (!(pq[pqi].top().getxPos() == xdx && pq[pqi].top().getyPos() == ydy)){
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pq[pqi].pop();									// 希望のノードを削除
					// より大きなサイズのPQを空にする
					if (pq[pqi].size()>pq[1 - pqi].size()) pqi = 1 - pqi;
					while (!pq[pqi].empty()){
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pqi = 1 - pqi;
					pq[pqi].push(*h0); //より良いノードを代わりに追加
				}
				else delete h0;
			}
		}
		delete w0;
	}
}

void serchObstacle(Mat im){

}

void loadMap(Mat im){
	Mat gray;												// 画像格納用
	cvtColor(im, gray, CV_RGB2GRAY);							// グレースケール変換
	threshold(gray, gray, 0, 255, THRESH_BINARY | THRESH_OTSU);	// 2値化
	int h = gray.rows;
	int w = gray.cols;
	for (int y = 0; y < h; y++){
		for (int x = 0; x < w; x++){
			if (gray.data[y * h + x] == 255){
				MAP[x][y] = 0;
			}
			else{
				MAP[x][y] = 1;
				wall.saveTxt("wall.csv",x,y);			
			}
		}
	}
}

void obstDetection(Mat im){
	hsv, mask;					        // 画像オブジェクトの宣言
	cvtColor(im, hsv, CV_BGR2HSV);      // 画像をRGBからHSVに変換
	inRange(hsv, Scalar(150, 70, 70), Scalar(360, 255, 255), mask);	// 色検出でマスク画像の作成
	int h = mask.rows;
	int w = mask.cols;
	for (int y = 0; y < h; y++){
		for (int x = 0; x < w; x++){
			if (mask.data[y * h + x] == 255){
				wall.saveTxt("obst.csv",x,y); 
			}
		}
	}
}

int serchPath(char* fname, int x1, int y1, int x2, int y2)
{
	Mat im = imread(fname);	// 画像の取得
	obstDetection(im);
	loadMap(im);
	string route = pathFind(x1, y1, x2, y2);
	// エラー処理(経路が見つからなかい場合)
	if (route.length() == 0) return -1;
	int j; char c;
	int x = x1;
	int y = y1;
	for (int i = 0; i<route.length(); i++){
		c = route.at(i);
		j = atoi(&c);
		x = x + dx[j];
		y = y + dy[j];
		// 算出した最短経路を地図画像に描く
		path.saveTxt("path.csv",x,y);
	}
	return 0;
}

int main(int argc, char *argv[])
{
	serchPath("map.jpg", 0, 0, 90, 90);
 	return(0);
}
