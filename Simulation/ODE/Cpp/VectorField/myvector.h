class VecForward{
private:
public:
	double x, y;

	// 経路に沿って進む速度ベクトルの計算
	void calc(double xi, double yi, double px[], double py[], int n)
	{
		int i;
		double rx, ry, r, rmin = 100.0;
		for (i = 0; i < n; i++){
			rx = px[i] - xi;
			ry = py[i] - yi;	
			r = sqrt(rx*rx+ry*ry);	// 現在位置と経路点のユークリッド距離 
			if (r <= rmin) {	
				if (i == n) {	// 最短点が終点の場合
					x = px[i] - xi;
					y = py[i] - yi;
				}
				
				else{			// 最短点が終点以外の場合{
					x = px[i+1] - xi;
					y = py[i+1] - yi;
					rmin = r;
				}
			}
		}
	}
};

// 最近傍の障害物を回避 
class VecAvoid{
private:
public:
	double x, y;
	// 障害物を回避する速度ベクトルの計算
	void calc(double xi, double yi, double ox[], double oy[], int n, double a, double b, double c)
	{
		double rx, ry, r, rmin = 100.0;
		int i,imin;

		for (i = 0; i < n; i++){
			rx = ox[i] - xi;
			ry = oy[i] - yi;	
			r = sqrt(rx*rx+ry*ry);	// 現在位置と経路点のユークリッド距離 
			if (r <= rmin) {	
				r = sqrt(rx*rx+ry*ry);				// 現在位置と障害物の距離
				x = -(rx/r)*c/(1+exp(a*r-b));
				y = -(ry/r)*c/(1+exp(a*r-b));
				rmin = r;
				imin = i;
			}
		}
		//printf("%f %f", ox[imin], oy[imin]);
	}
};

class PRep{
private:
public:
	double x, y;

	// 経路に沿って進む速度ベクトルの計算
	void calc(double xi, double yi, double ox[], double oy[], int n, double a)
	{
		int i;
		double rx, ry;
		x = 0;
		y = 0;
		for (i = 0; i < n; i++){
			rx = xi - ox[i];
			ry = yi - oy[i];	
			// 現在位置と障害物の距離
			x += a/(rx*rx*rx);
			y += a/(ry*ry*ry);
		}
	}
};

class PGoal{
private:
public:
	double x, y;

	// 経路に沿って進む速度ベクトルの計算
	void calc(double xi, double yi, double gx, double gy, double a)
	{
		double rx, ry;
		rx = xi - gx;
		ry = yi - gy;
		x = -a/(rx*rx*rx);
		y = -a/(ry*ry*ry);
	}
};
