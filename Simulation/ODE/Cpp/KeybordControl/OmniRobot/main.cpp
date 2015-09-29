#include <stdio.h>
#include <conio.h>
#include <math.h>
#include <process.h>
#include <stdlib.h>
#include <time.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
      
#ifdef _MSC_VER
#pragma warning(disable:4244 4305)
#endif
      
#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawBox     dsDrawBoxD
#define dsDrawLine    dsDrawLineD
#define dsDrawSphere  dsDrawSphereD
#endif
  
static dWorldID    world;
static dSpaceID    space;
static dGeomID     ground;
static dJointGroupID contactgroup;
static dsFunctions fn;
      
/* 各物体の個数 */
#define WHEEL_NUM 4
#define OMNI_NUM 1
#define BOX_NUM 2
      
/* 箱のパラメータ */
double boxM[BOX_NUM]={10, 10};
double boxSize[BOX_NUM][3]={{1, 1, 0.0}, {1, 1, 0.0}};
double boxPos[BOX_NUM][3]={{-0.5, 2.0, 0.001}, {0.5, 4.0, 0.001}};
dJointID fixed[BOX_NUM];
/* 土台のパラメータ */
double baseM[OMNI_NUM] = {9.4};
double baseSize[OMNI_NUM][3] = {0.40, 0.40, 0.1};
double basePos[OMNI_NUM][3] = {0, 0, 0.5+0.1};
/* 車輪のパラメータ */
double wheelSize[OMNI_NUM][3] = {0.1, 0.024, 0.010};
double wheelM[OMNI_NUM] = {0.15};
      
/* 構造体の定義 */
typedef struct{
    dBodyID  body;
    dGeomID  geom;
    dJointID joint;
} MyObject;
MyObject wheel[OMNI_NUM][WHEEL_NUM], base[OMNI_NUM], box[BOX_NUM];
  
/* グローバル変数の定義 */
double routeX[1000], routeY[1000], routeZ[1000];
double obstX[1000], obstY[1000], obstZ[1000];
double omniX[1000], omniY[1000], omniZ[1000];
int line, lineRoute, lineObst, lineOmni;
int t1, t2, dt, t;
double omniVX=0, omniVY=0;

/* --------------------------------
** 全方向移動ロボットの生成, 描画
----------------------------------- */
void MakeOmni()
{
    /* ローカル変数の定義 */
    dMass mass;
    double r, w, d, m_w, x, y, z, lx, ly, lz, m_b;
    /* ロボットの個数分だけループ */
    for (int i=0; i< OMNI_NUM; i++)
    {
        /* 車輪 */
        r = wheelSize[i][0];
        w = wheelSize[i][1];
        d = wheelSize[i][2];
        m_w = wheelM[i];
        /* 土台 */
        x = basePos[i][0];
        y = basePos[i][1];
        z = basePos[i][2];
        lx = baseSize[i][0];
        ly = baseSize[i][1];
        lz = baseSize[i][2];
        m_b = baseM[i];
        /* 土台の生成 */
        base[i].body  = dBodyCreate(world);
        dMassSetZero(&mass);
        dMassSetBoxTotal(&mass, m_b, lx, ly, lz);
        dBodySetMass(base[i].body,&mass);
        base[i].geom = dCreateBox(space, lx, ly, lz);
        dGeomSetBody(base[i].geom, base[i].body);
        dBodySetPosition(base[i].body, x, y, z); /* 座標 */
        /* 車輪の生成 */
        double wx[WHEEL_NUM] = {lx/2+w/2+d, - (lx/2+w/2+d), 0, 0};
        double wy[WHEEL_NUM] = {0, 0, ly/2+w/2+d, -(ly/2+w/2+d)};
        double wz[WHEEL_NUM] = {z, z, z, z};
        double jx[WHEEL_NUM] = {lx/2, -lx/2, 0, 0};
        double jy[WHEEL_NUM] = {0, 0, ly/2, -ly/2};
        double jz[WHEEL_NUM] = {z, z, z, z};
      
        for (int j = 0; j < WHEEL_NUM; j++)
        {
            wheel[i][j].body = dBodyCreate(world);
            dMatrix3 R;
            if (j >= 2)
            {
                dRFromAxisAndAngle(R,1,0,0,M_PI/2.0);
                dBodySetRotation(wheel[i][j].body,R);
            }
            else
            {
                dRFromAxisAndAngle(R, 0, 1, 0, M_PI/2.0);
                dBodySetRotation(wheel[i][j].body, R);
            }
            dMassSetZero(&mass);
            if (j < 2)
            {
                dMassSetCylinderTotal(&mass, m_w, 1, r, w);
            }
            else
            {
                dMassSetCylinderTotal(&mass, m_w, 2, r, w);
            }
            dBodySetMass(wheel[i][j].body,&mass);
            wheel[i][j].geom = dCreateCylinder(space, r, w);
            dGeomSetBody(wheel[i][j].geom, wheel[i][j].body);
            dBodySetPosition(wheel[i][j].body, wx[j], wy[j], wz[j]);
            wheel[i][j].joint = dJointCreateHinge(world, 0);
            dJointAttach(wheel[i][j].joint, base[i].body, wheel[0][j].body);
            if (j < 2)
            {
                dJointSetHingeAxis(wheel[i][j].joint, 1, 0, 0);
            }
            else
            {
                dJointSetHingeAxis(wheel[i][j].joint, 0, -1, 0);
            }
            dJointSetHingeAnchor(wheel[i][j].joint, jx[j], jy[j], jz[j]);
        }
    }
}
      
void DrawOmni(){
    /* 車輪の描画 */
    dReal radius, length;
    dsSetColor(0.3, 0.3, 0.3);
    for (int i=0; i<OMNI_NUM; i++)
    {
        for (int j=0; j< WHEEL_NUM; j++)
        {
            dGeomCylinderGetParams(wheel[i][j].geom, &radius, &length);
            dsDrawCylinder(dGeomGetPosition(wheel[i][j].geom), dGeomGetRotation(wheel[i][j].geom),length, radius);
        }
        /* 土台の描画 */
        dsSetColor(0.3, 0.1, 0.1);
        dsDrawBox(dBodyGetPosition(base[i].body),dBodyGetRotation(base[i].body),baseSize[i]);
    }
}
      
/* ------------------------
* ロボットの生成・制御・座標
------------------------ */
void ControlOmni(double vx, double vy, int n)
{
    /* ローカル変数の定義 */
    double wheelV[4] = {vy, vy, vx, vx};  /* 角速度 */
    for (int i=0; i< OMNI_NUM; i++)
    {
        for (int j=0; j<WHEEL_NUM;j++)
        {
            /* モータの目標速度，目標角速度 */
            dJointSetHingeParam(wheel[i][j].joint, dParamVel , wheelV[j]);
            /* 目標速度を達成するために発揮可能なトルクの最大値 */
            dJointSetHingeParam(wheel[i][j].joint, dParamFMax, 0.1);
            /* 1ステップでモータに過剰なトルクがかかるのを防ぐ */
            dJointSetHingeParam(wheel[i][j].joint, dParamFudgeFactor, 0.1);
        }
    }
}
      
void GetOmniPotision(double *x, double *y, int n)
{
    /* ローカル変数の定義 */
    const dReal *pb;
    pb  = dBodyGetPosition(base[n].body);   /* 台車の絶対座標 */
    *x = pb[0];
    *y = pb[1];
}
      
   
/* ------------------------
* 赤線の描画
------------------------ */
static void DrawLineOmni()
{
    /* ローカル変数の定義 */
    double pos[100][3];
    for (int i = 0 ; i < lineOmni ; i++)
    {
        pos[i][0] = omniX[i];
        pos[i][1] = omniY[i];
        pos[i][2] = 0.51;
    }
    dsSetColor(1.0, 0.2, 0.2);
    for (int i = 0; i < lineOmni-2; i++)
    {
        dsDrawLineD(pos[i],pos[i+1]);
    }
}
/* ------------------------
* 箱の生成,描画
------------------------ */
void MakeBox()
{
    /* ローカル変数の定義 */
    dMass mass;
    for(int i = 0; i < BOX_NUM; i++)
    {
        /* ボディを生成 */
        box[i].body = dBodyCreate(world);
        dMassSetZero(&mass);
        /* 質量を設定 */
        dMassSetBoxTotal(&mass, boxM[i], boxSize[i][0], boxSize[i][1], boxSize[i][2]);
        dBodySetMass(box[i].body, &mass);
        /* ジオメトリ生成 */
        box[i].geom = dCreateBox(space, boxSize[i][0], boxSize[i][1], boxSize[i][2]);
        /* ジオメトリをセット */
        dGeomSetBody(box[i].geom, box[i].body);
        dBodySetPosition(box[i].body, boxPos[i][0], boxPos[i][1], boxPos[i][2]);
        /* 箱と地面の結合 */
        fixed[i] = dJointCreateFixed(world, 0);
        dJointAttach(fixed[i], box[i].body, 0);
        dJointSetFixed(fixed[i]);
    }
}
      
void DrawBox()
{
    for(int i = 0; i < BOX_NUM; i++)
    {
        dsSetColorAlpha(0.3, 0.2, 0.1, 1.0);
        dsSetTexture(DS_WOOD);
        dsDrawBoxD(dBodyGetPosition(box[i].body),dBodyGetRotation(box[i].body), boxSize[i]);
    }
}
   
/* ------------------------
* 線の描画
------------------------ */
static void DrawLine(double x[], double y[], double z, int n, double r, double g, double b)
{
    /* ローカル変数の定義 */
    double pos[1000][3];
    for (int i = 0 ; i < n-1 ; i++)
    {
        pos[i][0] = x[i];
        pos[i][1] = y[i];
        pos[i][2] = z;
    }
    dsSetColor(r, g, b);
    for (int i = 0; i < n-2; i++)
    {
        dsDrawLineD(pos[i],pos[i+1]);
    }
}
   
/* ------------------------
* コールバック関数
------------------------ */
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    /* ローカル変数の定義 */
    dVector3 tmp_fdir = {0, 0, 0, 0};
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
      
    int wheel_flag = 0;
    for (int i=0; i<OMNI_NUM; i++)
    {
        for (int j=0; j<WHEEL_NUM; j++)
        {
            if ((o1 == wheel[i][j].geom)||(o2 == wheel[i][j].geom))
            {
                wheel_flag = 1;
                dJointGetHingeAxis(wheel[i][j].joint,tmp_fdir);
                break;
            }
        }
    }
    static const int N = 10;
    dContact contact[N];
    int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
    if (n > 0)
    {
        if (wheel_flag == 1)
        {
            for (int j=0; j<n; j++)
            {
                contact[j].surface.mode =  dContactFDir1| dContactMu2 | dContactSoftERP | dContactSoftCFM;
                contact[j].fdir1[0] = tmp_fdir[0];   /* 第１摩擦方向の設定(x軸成分) */
                contact[j].fdir1[1] = tmp_fdir[1];   /* 第１摩擦方向の設定(y軸成分) */
                contact[j].fdir1[2] = tmp_fdir[2];   /* 第１摩擦方向の設定(z軸成分) */
                contact[j].surface.mu =  0.01;        /* 車軸方向の摩擦係数 */
                contact[j].surface.mu2 = dInfinity;  /* 車輪方向の摩擦係数 */
                contact[j].surface.soft_erp = 0.9;
                contact[j].surface.soft_cfm = 0.001;
                dJointID c = dJointCreateContact(world,contactgroup,&contact[j]);
                dJointAttach(c,b1,b2);
            }
        }
        else
        {
            for (int j=0; j<n; j++)
            {
                contact[j].surface.mode = dContactSoftERP | dContactSoftCFM;
                contact[j].surface.mu   = dInfinity;
                contact[j].surface.soft_erp = 0.8;
                contact[j].surface.soft_cfm = 1e-5;
                dJointID c = dJointCreateContact(world,contactgroup,&contact[j]);
                dJointAttach(c,b1,b2);
            }
        }
    }
}
      
/* ------------------------
* txtデータの読み込み
------------------------ */
void LoadTxt(char filename[], double x[], double y[], double z[], int *line)
{
    /* ローカル変数の定義 */
    FILE *fp;
    int i = 0;
    /* エラー処理 */
    if((fp=fopen(filename,"r"))==NULL)
    {
        printf("FILE not open\n");
        exit(1);
    }
    /* データ読み込み */
    while(feof(fp) == 0)
    {
        fscanf(fp, "%lf%lf%lf", &x[i], &y[i], &z[i]);
        i+=1;
    }
    fclose(fp);
    *line = i+1;
}
 
/* テキストファイルに書き込み */
void SaveTxt(char filename[], double x[], double y[], double z[], int n)
{
    /* ローカル変数の定義 */
    FILE *fp;
    int i = 0;
    /* エラー処理 */
    if((fp=fopen(filename,"w"))==NULL)
    {
        printf("FILE not open\n");
        exit(1);
    }
    /* データ書き込み */
    while(i < n-1){
        fprintf(fp, "%0.2f\t%0.2f\t%0.2f\n", x[i], y[i], z[i]);
        i++;
    }
    /* 最後のデータだけ改行なしで書き込み */
    fprintf(fp, "%0.2f\t%0.2f\t%0.2f", x[n-1], y[n-1], z[n-1]);
    fclose(fp);
}
 
/* ------------------------
* 経路を進むベクトル
------------------------ */
void VectorRoute(double x, double y, double kx[], double ky[], int n, double *vx, double *vy)
{
    /* ローカル変数の定義 */
    int i;
    double a, b, r, rmin = 100.0;
    /* 経路に沿って進む速度ベクトルの計算 */
    for (i = 0; i < n; i++)
    {
        a = kx[i]-x;
        b = ky[i]-y;
        /* 現在位置と経路点のユークリッド距離 */
        r = a*a+b*b;
        if (r <= rmin)
        {
            /* 最短点が終点の場合 */
            if (i == n)
            {
                *vx = kx[i]-x;
                *vy = ky[i]-y;
            }
            /* 最短点が終点以外の場合 */
            else
            {
                *vx = kx[i+1]-x;
                *vy = ky[i+1]-y;
                rmin = r;
            }
        }
    }
}
      
/* ------------------------
* 障害物を回避するベクトル
------------------------ */
void VectorObst(double x, double y, double ox[], double oy[], int n, double a, double b, double c, double *vx, double *vy)
{
    /* ローカル変数の定義 */
    double rx, ry, r;
    int i;
    *vx = 0;
    *vy = 0;
    /* 障害物を回避する速度ベクトルの計算 */
    for (i = 0; i < n; i++)
    {
        rx = ox[i]-x;
        ry = oy[i]-y;
        r = sqrt(rx*rx+ry*ry);
        *vx += -(rx/r)*c/(1+exp(a*r-b));
        *vy += -(ry/r)*c/(1+exp(a*r-b));
    }
}

/*** キーボードコマンドの処理関数 ***/
static void command(int cmd)
{
    /* ローカル変数の定義 */
    double vmax=2.0;    /* 速度の限界値 */
    switch (cmd)
    {
        /* 前進 */
        case 'w':
            omniVX =  0.0;
            omniVY += 0.1;
            break;
        /* 後進 */
        case 'z':
            omniVX =   0.0;
            omniVY += -0.1;
            break;
        /* 左進 */
        case 'a':
            omniVX += -0.1;
            omniVY =   0.0;
            break;
        /* 右進 */
        case 'd':
            omniVX += 0.1;
            omniVY =  0.0;
            break;
        /* 停止 */
        default:
            omniVX = 0.0;
            omniVY = 0.0;
            break;
    }
    /* 速度が範囲内の場合 */
    if(omniVX<vmax && omniVX>-vmax && omniVY<vmax && omniVY>-vmax)
    {
        ControlOmni(omniVX, omniVY, 0);
    }
    /* 速度が限界値を超えた場合 */
    else if(omniVX>0)
    {
        ControlOmni(vmax, 0, 0);
    }
    else if(omniVX<0)
    {
        ControlOmni(-vmax, 0, 0);
    }
    else if(omniVY>0)
    {
        ControlOmni(0, vmax, 0);
    }
    else
    {
        ControlOmni(0, -vmax, 0);
    }
}


/* ------------------------
* スタート関数
------------------------ */
static void start()
{
    float xyz[3] = {   0.2, 2.5, 4.00};     /* カメラ座標 */
    float hpr[3] = { 90.0, -90.0, 0.0};     /* カメラ方向 */
    dsSetViewpoint(xyz,hpr);
    dsSetSphereQuality(3);
}
 
void SetCamera(double x, double y, double z, double u, double v, double w)
{
    float xyz[3] = {x, y, z};   /* カメラ座標 */
    float hpr[3] = {u, v, w};   /* カメラ方向 */
    dsSetViewpoint(xyz,hpr);
}
 
/* ------------------------
* シミュレーションループ
------------------------ */
static void simLoop(int pause)
{
    /* ローカル変数の定義 */
    double vx, vy, vx1, vy1, vx2, vy2, x, y;
	double sup_x[2], sup_y[2];
    t2 = clock();
    dSpaceCollide(space, 0, &nearCallback);
    GetOmniPotision(&x, &y, 0);
    printf("Position(x, y) = (%lf, %lf) \r", x, y);
    VectorRoute(x, y, routeX, routeY, lineRoute-2, &vx1, &vy1);
    VectorObst(x, y, obstX, obstY, lineObst-1, 1.0, 1.0, 0.0, &vx2, &vy2);
    vx = vx1+vx2;
    vy = vy1+vy2;
    /* 停留点にはまった場合(障害物回避ベクトルを切る) */
    if(abs(vx)<0.2 || abs(vy)<0.2)
    {
        vx = vx1*2;
        vy = vy1*2;
    }
    //ControlOmni(vx, vy, 0);
    dWorldStep(world, 0.01);
    dJointGroupEmpty(contactgroup);
    DrawOmni();
	/* 経路の表示 */
    //DrawLine(routeX, routeY, 0.01, lineRoute, 1.0, 0.2, 0.2);
    DrawBox();
    //SetCamera(x, y-0.15, 0.45, 90.0, 0.0, 0.0);
    dt = t2-t1;
    /* 1000[ms]毎にロボットが移動した経路を更新 */
    if(dt >1000)
    {
        t1 = clock();
        omniX[line] = x;
        omniY[line] = y;
        line++;
        /* ロボットの移動経路をtxtファイルに記録 */
        SaveTxt("omni.txt", omniX, omniY, omniZ, line);
    }
    /* 自律制御による移動経路の表示 */
    //DrawLine(omniX, omniY, 0.01, line, 0.2, 0.8, 0.2);
    /* 補助線の表示 */
	sup_x[0] = x;
	sup_x[1] = x+vx1;
	sup_y[0] = y;
	sup_y[1] = y+vy1;
	DrawLine(sup_x, sup_y, 0.4, 3, 1.0, 0.1, 0.0);
}
      
/* ------------------------
* 描画関数の設定
------------------------ */
static void setDrawStuff()
{
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    fn.path_to_textures = "img";
}
      
/* ------------------------
* メイン関数
------------------------ */
int main(int argc, char *argv[])
{
    /* txtデータ読み込み */
    LoadTxt("route.txt", routeX, routeY, routeZ, &lineRoute);
    LoadTxt("obstacle.txt", obstX, obstY, obstZ, &lineObst);
    /* ODEの初期化 */
    dInitODE();
    /* 描画関数の設定 */
    setDrawStuff();
    /* ワールド, スペース, 接触点グループの生成 */
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    /* 地面, 重力の生成 */
    ground = dCreatePlane(space,0,0,1,0);
    dWorldSetGravity(world, 0.0, 0.0, -9.8);
    /* CFM, ERPの設定 */
    dWorldSetCFM(world,1e-3);
    dWorldSetERP(world,0.8);
    /* 全方向移動ロボットの生成 */
    t1 = clock();
    MakeBox();
    MakeOmni();
    /* シミュレーションループ */
    dsSimulationLoop(argc,argv,640,480,&fn);
    /* 接触点グループ, スペース, ワールドの破壊, ODEの終了 */
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
