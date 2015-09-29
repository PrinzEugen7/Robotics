#include <stdio.h>
#include <iostream>
#include <string>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "mytxt.h"
#include "myvector.h"
#include "myode.h"
 
// オブジェクトの宣言
Omni robot;			// 全方向ロボット
Box wall1, wall2, wall3, wall4, wall5, wall6, wall7;
VecForward vf;		// ベクトル場(経路進む)
VecAvoid vw, vo;		// ベクトル場(障害物回避)
PRep pr;
PGoal pg;
Txt p, w, o, robo;	// テキストファイルの処理
int i;

// カメラの設定
void SetCamera(double x, double y, double z, double u, double v, double w)
{
    float xyz[3] = {x, y, z};   // カメラ座標
    float hpr[3] = {u, v, w};   // カメラ方向
    dsSetViewpoint(xyz,hpr);
}

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  int i,n;
  
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
  
  static const int N = 10;
  dContact contact[N];
  n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
        dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu       = dInfinity;
      contact[i].surface.slip1    = 0.1;
      contact[i].surface.slip2    = 0.1;
      contact[i].surface.soft_erp = 0.8;
      contact[i].surface.soft_cfm = 1e-5;
      dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
      dJointAttach(c,b1,b2);
    }
  }
}

// シミュレーションループ
static void simLoop(int pause)
{
    double x1, y1,vx, vy;

    dSpaceCollide(space, 0, &nearCallback);			// 衝突の計算
    robot.getPosition(&x1, &y1);					// ロボットの位置取得
    vf.calc(x1, y1, p.x, p.y, p.lines-2);			// 経路を進むベクトル場の計算を計算
	vw.calc(x1, y1, w.x, w.y, w.lines-1, 1, 1, 1);	// 障害物を回避するベクトル場を計算
	vo.calc(x1, y1, o.x, o.y, o.lines-1, 1, 1, 1);	// 障害物を回避するベクトル場を計算
	// ベクトル場の加算
	vx = 2*vf.x + 0 * vw.x + 1 * vo.x;
	vy = 2*vf.y + 0 * vw.y + 1 * vo.y;
	//if(vx<0.0001 && vy<0.0001){
		//vx = vf.x;
		//vy = vf.y;
	//}
    robot.control(vx, vy, 0);						// ロボットの速度制御
	if(i==30){
		robo.saveTxt("robot2-2-1.csv",x1,y1,0);			// ロボットの移動経路を定期的に保存
		i = 0;
	}
    // シミュレーションの精度
    dWorldStep(world, 0.07);
    dJointGroupEmpty(contactgroup);
	// 各物体の描画 
    robot.drow(0.2, 0.2, 0.2, 0.8, 0.2, 0.2);
	wall1.drow(0.7, 0.7, 0.7, 1);
	wall2.drow(0.7, 0.7, 0.7, 1);
	wall3.drow(0.7, 0.7, 0.7, 1);
	wall4.drow(0.7, 0.7, 0.7, 1);
	wall5.drow(0.7, 0.7, 0.7, 1);
	i++;
}
     
// スタート関数
static void start()
{
    float xyz[3] = { 10.0, 10.0, 13.00};     // カメラ座標
    float hpr[3] = { 90.0, -90.0, 180.0};    // カメラ方向 
    dsSetViewpoint(xyz,hpr);
    dsSetSphereQuality(3);
}   
// 描画関数の設定
static void setDrawStuff()
{
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.path_to_textures = "img";
}

// メイン関数
int main(int argc, char *argv[])
{
    p.loadTxt("path.csv");	// 経路の位置を取得
    w.loadTxt("wall.csv");	// 壁の位置を取得
    o.loadTxt("obst.csv");	// 壁の位置を取得
    dInitODE();				// ODEの初期化
    setDrawStuff();			// 描画関数の設定
    // ワールド, スペース, 接触点グループの生成
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    // 地面, 重力, CFM, ERPの設定
    ground = dCreatePlane(space,0,0,1,0);
    dWorldSetGravity(world, 0.0, 0.0, -9.8);
    dWorldSetCFM(world,1e-3);
    dWorldSetERP(world,0.8);
	// 各物体の生成と初期化 ∗ /
	wall1.init(1.5, 10, 0.5, 3, 20, 1, 14, 1);
	wall2.init(14, 1.5, 0.5, 14, 3, 1, 14, 1);
	wall3.init(22.5, 8, 0.5, 3, 16, 1, 14, 1);
	wall4.init(14, 11.5, 0.5, 14, 9, 1, 14, 1);
	wall5.init(10.5, 21.5, 0.5, 21, 3, 1, 14, 1);
	robot.init(0.1, 0.024, 0.01, 1.0, 18, 5, 0.3, 0.4, 0.4, 0.1, 5);
    /* シミュレーションループ */
    dsSimulationLoop(argc,argv,640,480,&fn);
    /* 接触点グループ, スペース, ワールドの破壊, ODEの終了 */
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
