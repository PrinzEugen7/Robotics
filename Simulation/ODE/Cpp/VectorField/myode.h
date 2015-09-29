#ifdef _MSC_VER
#pragma warning(disable:4244 4305)
#endif
    
static dWorldID    world;
static dSpaceID    space;
static dGeomID     ground;
static dJointGroupID contactgroup;
static dsFunctions fn;
dJointID fixed;

// 箱
class Box{
private:
    double lx, ly, lz;
    typedef struct{
        dBodyID  body;
        dGeomID  geom;
        dJointID joint;
    } MyObject;
    MyObject box;
 
public:
    Box(){
    }
     
    void init(double x, double y, double z, double blx, double bly, double blz, double m, int flag){
     
        dMass mass;
        lx = blx; ly = bly; lz = blz;
         
        box.body = dBodyCreate(world);              /* ボディを生成 */
        dMassSetZero(&mass);
        dMassSetBoxTotal(&mass, m, lx, ly, lz);     /* 質量を設定 */
        dBodySetMass(box.body, &mass);
        box.geom = dCreateBox(space, lx, ly, lz);   /* ジオメトリ生成 */
        dGeomSetBody(box.geom, box.body);           /* ジオメトリをセット */
        dBodySetPosition(box.body, x, y, z);
        if(flag == 1){
            fixed = dJointCreateFixed(world, 0);        /* 箱と地面の結合 */
            dJointAttach(fixed, box.body, 0);
            dJointSetFixed(fixed);
        }
    }
    void drow(double r, double g, double b, double a){
        double box_size[3] = {lx ,ly, lz};
        dsSetColorAlpha(r, g, b, a);
        dsSetTexture(DS_WOOD);
        dsDrawBoxD(dBodyGetPosition(box.body),dBodyGetRotation(box.body), box_size);
    }
};

class Cylinder{
private:
    double l, r;
    typedef struct{
        dBodyID  body;
        dGeomID  geom;
        dJointID joint;
    } MyObject;
    MyObject cylinder;
public:
    Cylinder(){
    }
    // 円柱の生成
    void init(double x, double y, double z, double cr, double cl, double m, int flag){
        dMass mass;                                         // シリンダの重さ
        l = cr;
        r = cl;
        cylinder.body = dBodyCreate(world);         // ボディを生成
        dMassSetZero(&mass);
        dMassSetCylinderTotal(&mass, m, 3, r, l); // 質量を設定
        dBodySetMass(cylinder.body, &mass);
        cylinder.geom = dCreateCylinder(space, r, l);   // ジオメトリ生成
        dGeomSetBody(cylinder.geom, cylinder.body);   // ジオメトリをセット
        dBodySetPosition(cylinder.body, x, y, z);
        if(flag == 1){
            fixed = dJointCreateFixed(world, 0);        /* 箱と地面の結合 */
            dJointAttach(fixed, cylinder.body, 0);
            dJointSetFixed(fixed);
        }
    }
    // 円柱の描画
    void drow(double red, double green, double blue, double alpha){
        dsSetColorAlpha(red, green, blue, alpha);
        dsDrawCylinderD(dBodyGetPosition(cylinder.body),dBodyGetRotation(cylinder.body), l, r);
    }
    // 円柱の回転
    void rotate(double ax, double ay, double az, double angle){
        dMatrix3 R;
        dRFromAxisAndAngle(R, ax, ay, az, angle);
        dBodySetRotation(cylinder.body, R);
    }
};

class Line{
private:
public:
    Line(){
    }
    // 描画
    void drow(double x0, double y0, double z0, double x, double y, double z, double r, double g, double b){
		double p0[3]={x0, y0, z0};
		double p[3]={x, y, z};
		dsSetColor(r, g, b);
		dsDrawLineD(p0, p);
    }
    // 複数描画
    void drows(double x[], double y[], double z, int n, double r, double g, double b){
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
};
    
class Omni{
private:

public:
    typedef struct{
        dBodyID  body;
        dGeomID  geom;
        dJointID joint;
    } MyObject;
    MyObject wheel[4], base;
     
    Omni(){
    }
    /* 全方向ロボットの生成 */
    void init(double r, double w, double d, double m_w, double x, double y, double z, double lx, double ly, double lz, double m_b)
    {
        dMass mass;
        /* 土台の生成 */
        base.body  = dBodyCreate(world);
        dMassSetZero(&mass);
        dMassSetBoxTotal(&mass, m_b, lx, ly, lz);
        dBodySetMass(base.body,&mass);
        base.geom = dCreateBox(space, lx, ly, lz);
        dGeomSetBody(base.geom, base.body);
        dBodySetPosition(base.body, x, y, z); /* 座標 */
        /* 車輪の生成 */
        double wx[4] = {x+lx/2+w/2+d, x- (lx/2+w/2+d), x, x};
        double wy[4] = {y, y, y+ly/2+w/2+d, y-(ly/2+w/2+d)};
        double wz[4] = {z, z, z, z};
        double jx[4] = {x+lx/2, x-lx/2, x, x};
        double jy[4] = {y, y, y+ly/2, y-ly/2};
        double jz[4] = {z, z, z, z};
        for (int j = 0; j < 4; j++)
        {
            wheel[j].body = dBodyCreate(world);
            dMatrix3 R;
            if (j >= 2)
            {
                dRFromAxisAndAngle(R,1,0,0,M_PI/2.0);
                dBodySetRotation(wheel[j].body,R);
            }
            else
            {
                dRFromAxisAndAngle(R, 0, 1, 0, M_PI/2.0);
                dBodySetRotation(wheel[j].body, R);
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
            dBodySetMass(wheel[j].body,&mass);
            wheel[j].geom = dCreateCylinder(space, r, w);
            dGeomSetBody(wheel[j].geom, wheel[j].body);
            dBodySetPosition(wheel[j].body, wx[j], wy[j], wz[j]);
            wheel[j].joint = dJointCreateHinge(world, 0);
            dJointAttach(wheel[j].joint, base.body, wheel[j].body);
            if (j < 2)
            {
                dJointSetHingeAxis(wheel[j].joint, 1, 0, 0);
            }
            else
            {
                dJointSetHingeAxis(wheel[j].joint, 0, -1, 0);
            }
            dJointSetHingeAnchor(wheel[j].joint, jx[j], jy[j], jz[j]);
        }
    }
    void drow(double r_w, double g_w, double b_w, double r_b, double g_b, double b_b){
        dReal r, l;
        dVector3 sides; 
        dsSetColor(r_w, g_w, b_w);
        /* 車輪の描画 */
        for (int j=0; j < 4; j++)
        {
            dGeomCylinderGetParams(wheel[j].geom, &r, &l);
            dsDrawCylinderD(dGeomGetPosition(wheel[j].geom), dGeomGetRotation(wheel[j].geom),l, r);
        }
        /* 土台の描画 */
        dGeomBoxGetLengths(base.geom,sides); 
        dsSetColor(r_b, g_b, b_b);
        dsDrawBoxD(dBodyGetPosition(base.body),dBodyGetRotation(base.body), sides);
    }
 
    void control(double vx, double vy, int n)
    {
        double wheel_v[4] = {vy, vy, vx, vx};  /* 角速度 */
        for (int j=0; j<4; j++)
        {
            /* モータの目標速度，目標角速度 */
            dJointSetHingeParam(wheel[j].joint, dParamVel , wheel_v[j]);
            /* 目標速度を達成するために発揮可能なトルクの最大値 */
            dJointSetHingeParam(wheel[j].joint, dParamFMax, 0.1);
            /* 1ステップでモータに過剰なトルクがかかるのを防ぐ */
            dJointSetHingeParam(wheel[j].joint, dParamFudgeFactor, 0.1);
        }
    }
     
    // ロボットの座標取得
    void getPosition(double *x, double *y)
    {
        const dReal *pb;
        pb  = dBodyGetPosition(base.body);   /* 台車の絶対座標 */
        *x = pb[0];
        *y = pb[1];
    }
};

class WheelRobot{
private:
public:
	typedef struct { 
		dBodyID body; 
		dGeomID geom;
		double  l,r,m; 
	} MyObject;

	MyObject wheel[4], base;
	dJointID joint[4];

	void init(double r0, double r1, double w, double m_w0, double m_w1, double x, double y, double z, double lx, double ly, double lz, double m_b)
	{
		dMatrix3 R;
		dMass mass, mass2, mass3;
		base.body  = dBodyCreate(world);
		dMassSetZero(&mass);
		dMassSetBoxTotal(&mass,m_b,lx,ly,lz);
		dBodySetMass(base.body,&mass);
		base.geom = dCreateBox(space,lx,ly,lz);
		dGeomSetBody(base.geom,base.body);
		dBodySetPosition(base.body,x,y,z);

		dRFromAxisAndAngle(R,0,1,0,M_PI/2.0);
		for (int i = 0; i < 4; i++) {
			wheel[i].body = dBodyCreate(world);

			dMassSetZero(&mass);
			if ((i % 2) == 0) {
				dMassSetCylinderTotal(&mass,m_w0, 2, r0, w);
				dBodySetMass(wheel[i].body,&mass);
				wheel[i].geom = dCreateCylinder(space, r0, w);
			}
			else {
				dMassSetCylinderTotal(&mass,m_w1, 2, r1, w);
				dBodySetMass(wheel[i].body,&mass);
				wheel[i].geom = dCreateCylinder(space, r1, w);
			}		
			dGeomSetBody(wheel[i].geom, wheel[i].body);
			dBodySetRotation(wheel[i].body,R);
		}

		dReal w1x = 0.5*(ly+w);
		dReal w1z = z - 0.5*lz;
		dReal w0y = 0.5*ly - r0;
		dReal w0z = z - 0.5*lz - r0;

		dBodySetPosition(wheel[0].body,  x,  w0y+y, w0z);
		dBodySetPosition(wheel[1].body,  x-w1x,  y, w1z);
		dBodySetPosition(wheel[2].body,  x, y-w0y, w0z);
		dBodySetPosition(wheel[3].body,  x+w1x,  y, w1z);

		for (int i = 0; i < 4; i++) {
			joint[i] = dJointCreateHinge(world,0);
			dJointAttach(joint[i], base.body, wheel[i].body);
		}
		dJointSetHingeAxis(joint[0],1, 0, 0);
		dJointSetHingeAxis(joint[1],1, 0, 0);
		dJointSetHingeAxis(joint[2],1, 0, 0);
		dJointSetHingeAxis(joint[3],1, 0, 0);
		dJointSetHingeAnchor(joint[0],    x,  w0y+y, w0z);
		dJointSetHingeAnchor(joint[1], x-w1x,  y, w1z);
		dJointSetHingeAnchor(joint[2], x, y-w0y, w0z);
		dJointSetHingeAnchor(joint[3], x+w1x,  y, w1z);
	}
	void drow(double rw, double gw, double bw, double rb, double gb, double bb){
		dVector3 sides; 
		dReal r, l;
		dsSetColor(rw, gw, bw);
		for (int i=0; i< 4; i++) {
				dGeomCylinderGetParams(wheel[i].geom, &r, &l);
				dsDrawCylinderD(dBodyGetPosition(wheel[i].body),dBodyGetRotation(wheel[i].body),l,r);
		}
        dGeomBoxGetLengths(base.geom,sides);
        dsSetColor(rb, gb, bb);
        dsDrawBoxD(dBodyGetPosition(base.body),dBodyGetRotation(base.body), sides);
	}

	void control(double v_left, double v_right)
	{
		dJointSetHingeParam(joint[1], dParamVel , v_left);
		dJointSetHingeParam(joint[3], dParamVel , v_right);
		dJointSetHingeParam(joint[1], dParamFMax, 100);
		dJointSetHingeParam(joint[3], dParamFMax, 100);
		dJointSetHingeParam(joint[1], dParamFudgeFactor, 0.1);
		dJointSetHingeParam(joint[3], dParamFudgeFactor, 0.1);
	}

	void getPosition(double *x, double *y)
	{
		const dReal *pb;
		pb  = dBodyGetPosition(base.body); 
		*x = pb[0];
		*y = pb[1];
	}
};
