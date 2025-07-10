
#include "stdafx.h"
#include "math.h"
#include "CVector.h"
#include "CMatrix.h"
#include <ctime>
#include <vector>
#include <algorithm> 
#include "CEuler.h"
#include <string>      // std::string
#include <sstream>     // std::stringstream, std::istringstream
#include <iomanip>     // std::setprecision, std::fixed
#include <atlimage.h>
#include "glew.h"
#include  "glut.h"
GLuint texture[10] = {0,1,2,3,4,5,6,7,8,9};
bool keyState[256] = { false }; // 初始化所有键的状态为未按下
bool isAstronautView = false; // 当前是否为太空人视角
int lastCollisionPlanet = -1;
// 飞船半径估计值（用于碰撞）
const float spaceshipRadius = 0.5f;
CVector lastCollisionPoint = { 0, 0, 0 };
float lastCollisionTime = -1.0f;
// 视点位置和方向
float mx = 0, my = 5, mz = 10, rx = -0.25, ry = 0, rz = 0; // 平移和旋转
float godView_mx = 0, godView_my = 5, godView_mz = 10;
float godView_rx = -0.25, godView_ry = 0, godView_rz = 0;
float sx = 1, sy = 1, sz = 1; // 缩放
float mspeed = 0.003, rspeed = 0.02;
float g_IEyeMat[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }, g_EyeMat[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
int mode = 1; // 0: 矩阵模式，1: 直接模式
bool isInterpolating = false; // 是否正在进行插值
int interpolationFrame = 0; // 当前插值帧数
const int interpolationTotalFrames = 300; // 总插值帧数
CVector initialPosition, targetPosition; // 初始和目标视点位置
float initialRx, initialRy, initialRz; // 初始旋转角度
float targetRx, targetRy, targetRz; // 目标旋转角度
CQuaternion initialQuat, targetQuat; // 初始和目标四元数
CQuaternion currentQuat;            // 当前四元数

template <typename T>
T clamp(T value, T min, T max) {
	if (value < min) {
		return min;
	}
	else if (value > max) {
		return max;
	}
	else {
		return value;
	}
}


struct Astronaut {
	CVector position; // 太空人位置
	float angle; // 太空人的旋转角度
	float step; // 太空人的行走步数
};
Astronaut astronaut = { CVector(0, 0, 0), 0, 0 };
CVector v; // 球的速度向量
CVector stars[100]; // 星星的位置
CVector points[30][60];
CVector pos;//球的位置
float starBrightness[100]; // 星星的亮度
float t = 0; // 时间变量，用于控制星星亮度变化
void myDisplay(void);
void DrawStars();
void myKeyboard();
void DrawPlanets();
void DrawShip();
void DrawText(const char* text, float x, float y, float z, void* font);
void rotateConeToDirection(float vx, float vy, float vz);
void DrawTexturedSphere(float radius, int slices, int stacks);
float seta = 0;//自转速度
float sp = 0.02;//速度大小
struct Planet {
    CVector position; // 行星位置
    float radius; // 行星半径
    float orbitRadius; // 公转轨道半径
    float orbitSpeed; // 公转速度
    float rotationSpeed; // 自转速度
    float angle; // 当前角度
    CVector color; // 行星颜色
};

struct Spaceship {
    CVector position; // 飞船位置
    float angleX, angleY; // 飞船的旋转角度
    float speed; // 飞船速度
    CVector direction; // 飞船的移动方向
    int targetPlanet; // 目标行星的索引
};

Spaceship spaceship = { CVector(0, 0, 0), 0, 0, 0.02, CVector(0, 0, 0), -1 };

// 全局变量
std::vector<Planet> planets = {
	{CVector(0, 0, 0), 0.5 * 7, 0, 0, 0, 0, CVector(1.0, 1.0, 0.0)}, // 太阳（黄色）
	{CVector(0, 0, 0), 0.2 * 7, 1.5 * 8, 0.01, 0.02, 0, CVector(0.8, 0.8, 0.8)}, // 水星（灰色）
	{CVector(0, 0, 0), 0.3 * 7, 2.0 * 8, 0.008, 0.015, 0, CVector(0.7, 0.6, 0.2)}, // 金星（黄褐色）
	{CVector(0, 0, 0), 0.4 * 7, 2.5 * 8, 0.006, 0.01, 0, CVector(0.2, 0.2, 0.9)}, // 地球（蓝色）
	{CVector(0, 0, 0), 0.1 * 7, 2.6 *2, 0.007, 0.012, 0, CVector(0.8, 0.8, 0.8)}, // 月球（灰色）
	{CVector(0, 0, 0), 0.35 * 7, 3.0 * 8, 0.005, 0.008, 0, CVector(0.9, 0.2, 0.2)}, // 火星（红色）
	{CVector(0, 0, 0), 0.6 * 7, 4.0 * 8, 0.004, 0.006, 0, CVector(0.9, 0.9, 0.2)}, // 木星（浅黄色）
	{CVector(0, 0, 0), 0.55 * 7, 5.0 * 8, 0.003, 0.005, 0, CVector(0.8, 0.6, 0.2)}, // 土星（黄褐色）
};
int selectedPlanet = -1; // 当前选中的行星
bool wireframeMode = false; // 是否为线框模式

void myTimerFunc(int val)
{
	t += 0.033f;
	seta += 0.1f;

	// 更新每颗行星的公转角度
	for (size_t i = 1; i < planets.size(); ++i) {
		planets[i].angle += planets[i].orbitSpeed;
	}

	// 飞船移动逻辑
	if (spaceship.targetPlanet != -1) {
		float distance = (planets[spaceship.targetPlanet].position - spaceship.position).len();
		if (distance > spaceship.speed) {
			spaceship.position = spaceship.position + spaceship.direction * spaceship.speed;
		}
		else {
			spaceship.position = planets[spaceship.targetPlanet].position;
			spaceship.targetPlanet = -1;
		}
	}
	else {
		CVector direction = CVector(
			sin(spaceship.angleY * 3.1415 / 180.0),
			0,
			cos(spaceship.angleY * 3.1415 / 180.0)
		);
		direction.Normalize();
		spaceship.position = spaceship.position + direction * spaceship.speed;
	}

	// 宇航员视角更新
	if (isAstronautView) {
		mx = spaceship.position.x;
		my = spaceship.position.y + 0.5f;
		mz = spaceship.position.z - 1.0f;
		ry = spaceship.angleY;
	}

	for (size_t i = 0; i < planets.size(); ++i) {
		float dist = (spaceship.position - planets[i].position).len();
		if (dist <= planets[i].radius + spaceshipRadius) {
			lastCollisionPlanet = i;
			lastCollisionTime = t;
			lastCollisionPoint = planets[i].position;
			break;  // 只记录第一个碰撞（本帧只处理一次）
		}
	}

	// 刷新画面
	myDisplay();
	glutTimerFunc(33, myTimerFunc, 0);  // 大约30帧/秒
}
void DrawText(float x, float y, const std::string& text) {
	glRasterPos2f(x, y);
	for (char c : text)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
}
void Font2D(char* str, double x, double y, int type)
{
	//设置投影方式：平行投影
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(-1.0f, 1.0f, -1.0f, 1.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	//输出字符串
	int len = (int)strlen(str);
	glRasterPos2f(x, y);
	for (int i = 0; i < len; ++i)
	{
		switch (type) {
		case 1:
			glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
			break;
		case 2:
			glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[i]);
			break;
		case 3:
			glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, str[i]);
			break;
		case 4:
			glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str[i]);
			break;
		case 5:
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, str[i]);
			break;
		case 6:
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, str[i]);
			break;
		case 7:
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, str[i]);
			break;
		}
	}
	//恢复投影方式
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

int LoadGLTextures(const char* filename, int i) {
	CImage img;
	HRESULT hResult = img.Load(filename);
	if (FAILED(hResult))
	{
		return 0;
	}
	glGenTextures(1, &texture[i]);                 // 创建纹理ID
	glBindTexture(GL_TEXTURE_2D, texture[i]);      // 绑定纹理

	int pitch = img.GetPitch();
	if (pitch < 0)
		glTexImage2D(GL_TEXTURE_2D, 0, img.GetBPP() / 8,
			img.GetWidth(), img.GetHeight(), 0,
			GL_BGR, GL_UNSIGNED_BYTE,
			img.GetPixelAddress(0, img.GetHeight() - 1));
	else
		glTexImage2D(GL_TEXTURE_2D, 0, img.GetBPP() / 8,
			img.GetWidth(), img.GetHeight(), 0,
			GL_BGR, GL_UNSIGNED_BYTE,
			img.GetBits());

	// 设置纹理参数
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	return 1;
}
void InitLighting() {
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);  // 自动单位化法向量（避免缩放影响）

	glEnable(GL_COLOR_MATERIAL); // 使用 glColor 影响材质
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	glEnable(GL_LIGHT0);  // 太阳光
	glEnable(GL_LIGHT1);  // 飞船聚光灯
	glEnable(GL_LIGHT2);  // 飞船内部点光源
}
void SetupSunLight() {
	GLfloat light_pos[] = { planets[0].position.x,planets[0].position.y,planets[0].position.z, 1.0f}; // 点光源
	GLfloat ambient[] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat diffuse[] = { 1.0f, 1.0f, 0.9f, 1.0f };
	GLfloat specular[] = { 1.0f, 1.0f, 0.9f, 1.0f };

	glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);

	// 距离衰减（可选）
	glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1.0f);
	glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.0005f);
	glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.00001f);
}
void SetupShipSpotlight() {
	// 飞船当前位置作为光源位置
	GLfloat light_pos[] = {
		spaceship.position.x,
		spaceship.position.y,
		spaceship.position.z,
		1.0f // 位置光源
	};

	// 飞船前进方向（假设已单位化）
	GLfloat spot_dir[] = {
		spaceship.direction.x,
		spaceship.direction.y,
		spaceship.direction.z
	};

	// 光的颜色（微蓝偏冷）
	GLfloat diffuse[] = { 0.8f, 0.8f, 1.0f, 1.0f };

	// 设置光源属性
	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_POSITION, light_pos);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, spot_dir);

	// 聚光灯参数
	glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 45.0f);       // 截止角（45°）
	glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 10.0f);     // 聚焦程度（越大越集中）

	// 距离衰减参数
	glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 1.0f);      // 常数项
	glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.01f);       // 线性项
	glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.005f);   // 二次项
}
void SetupInternalLight() {
	GLfloat light_pos[] = {
		spaceship.position.x,
		spaceship.position.y,
		spaceship.position.z, 1.0f
	};
	GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat diffuse[] = { 1.0f, 0.7f, 0.6f, 1.0f };

	glLightfv(GL_LIGHT2, GL_POSITION, light_pos);
	glLightfv(GL_LIGHT2, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse);

	glLightf(GL_LIGHT2, GL_CONSTANT_ATTENUATION, 1.0f);
	glLightf(GL_LIGHT2, GL_LINEAR_ATTENUATION, 0.5f);
	glLightf(GL_LIGHT2, GL_QUADRATIC_ATTENUATION, 0.3f);
}

void SetRC()
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glFrontFace(GL_CCW);
	
	glEnable(GL_DEPTH_TEST);
	glPolygonMode(GL_BACK, GL_LINE);
	LoadGLTextures("1.jpg", 0);
	LoadGLTextures("2.jpg", 1);
	LoadGLTextures("3.jpg", 2);
	LoadGLTextures("4.jpg", 3);
	LoadGLTextures("5.jpg", 4);
	LoadGLTextures("6.jpg", 5);
	LoadGLTextures("7.jpg", 6);
	LoadGLTextures("8.jpg", 7);
	LoadGLTextures("9.jpg", 8);
}
#define A2R(x) (x/180.0*3.14159)


bool rayIntersectsSphere(const CVector& rayOrigin, const CVector& rayDirection, const CVector& sphereCenter, float sphereRadius, float& t) {
    CVector L = sphereCenter - rayOrigin;
    float tca = -L.dotMul(rayDirection);
    //printf("tca=%f\n", tca);
    if (tca < 0) return false; // 射线方向与球心方向相反，无交点

    float d2 = L.dotMul(L) - tca * tca;
    float r2 = sphereRadius * sphereRadius;
    //printf("d2=%f,r2=%f\n",d2, r2);
    if (d2 > r2) return false; // 射线与球体不相交

    float thc = sqrt(r2 - d2);
    t = tca - thc; // 最近的交点
    return true;
}
void SetView() {
	if (isAstronautView) {
		// 太空人视角
		glTranslatef(-mx, -my, -mz);
		glRotatef(-rz, 0, 0, 1);
		glRotatef(-rx, 1, 0, 0);
		glRotatef(-ry, 0, 1, 0);
	}
	else {
		// 上帝视角
		if (mode == 0) {
			glLoadMatrixf(g_EyeMat);
		}
		else {
			glRotatef(-rz, 0, 0, 1);
			glRotatef(-rx, 1, 0, 0);
			glRotatef(-ry, 0, 1, 0);
			glTranslatef(-mx, -my, -mz);
		}
	}
}
void glutSolidCylinder(float radius, float height, int slices, int stacks) {
	float step = 2 * PI / slices; // 每个切片的角度
	float halfHeight = height / 2.0f; // 圆柱体的一半高度

	// 绘制圆柱体的侧面
	for (int i = 0; i < slices; ++i) {
		glBegin(GL_QUADS);
		{
			// 上圆
			glVertex3f(radius * cos(i * step), halfHeight, radius * sin(i * step));
			glVertex3f(radius * cos((i + 1) * step), halfHeight, radius * sin((i + 1) * step));
			// 下圆
			glVertex3f(radius * cos((i + 1) * step), -halfHeight, radius * sin((i + 1) * step));
			glVertex3f(radius * cos(i * step), -halfHeight, radius * sin(i * step));
		}
		glEnd();
	}

	// 绘制圆柱体的顶部
	glBegin(GL_POLYGON);
	for (int i = 0; i < slices; ++i) {
		glVertex3f(radius * cos(i * step), halfHeight, radius * sin(i * step));
	}
	glEnd();

	// 绘制圆柱体的底部
	glBegin(GL_POLYGON);
	for (int i = 0; i < slices; ++i) {
		glVertex3f(radius * cos(i * step), -halfHeight, radius * sin(i * step));
	}
	glEnd();
}
void mouse(int button, int state, int x, int y) {
    /*if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        // 获取鼠标点击位置的屏幕坐标
        int viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        int winWidth = viewport[2];
        int winHeight = viewport[3];
        // 将鼠标坐标转换为 NDC 坐标
        float ndcX = (2.0f * x) / winWidth - 1.0f;
        float ndcY = 1.0f - (2.0f * y) / winHeight;
        // 从深度缓冲区读取 Z 值
        float z, h;
        glReadPixels(x, winHeight - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);
        glReadPixels(7.0f, winHeight - 7.0f, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &h);
        // 将 NDC 坐标转换为世界坐标
        CVector rayOrigin(7 * 2 / winWidth - 1.0f, 1 - (2.0f * 7) / winHeight, h); // 摄像机位置
        CVector rayDirection(ndcX, ndcY, z);
        rayDirection.Normalize();

        // 检测射线与每个行星的交点
        float closestT = std::numeric_limits<float>::max();
        int closestPlanet = -1;
        for (size_t i = 0; i < planets.size(); ++i) {
            float t;
            if (rayIntersectsSphere(rayOrigin, rayDirection, planets[i].position, planets[i].radius, t)) {
                if (t < closestT) {
                    closestT = t;
                    closestPlanet = i;
                }
            }
        }

        // 更新选中的行星
        selectedPlanet = closestPlanet;
    }
    glutPostRedisplay();*/
}
void DrawTexturedSphere(float radius, int slices, int stacks)
{
	for (int i = 0; i < stacks; ++i) {
		float lat0 = PI * (-0.5f + (float)i / stacks);
		float z0 = radius * sin(lat0);
		float zr0 = radius * cos(lat0);

		float lat1 = PI * (-0.5f + (float)(i + 1) / stacks);
		float z1 = radius * sin(lat1);
		float zr1 = radius * cos(lat1);

		glBegin(GL_QUAD_STRIP);
		for (int j = 0; j <= slices; ++j) {
			float lng = 2.0f * PI * (float)j / slices;
			float x = cos(lng);
			float y = sin(lng);

			glTexCoord2f((float)j / slices, (float)i / stacks);
			glNormal3f(x * zr0, y * zr0, z0);
			glVertex3f(x * zr0, y * zr0, z0);

			glTexCoord2f((float)j / slices, (float)(i + 1) / stacks);
			glNormal3f(x * zr1, y * zr1, z1);
			glVertex3f(x * zr1, y * zr1, z1);
		}
		glEnd();
	}
}


void DrawAstronaut() {
	glPushMatrix();
	glTranslatef(spaceship.position.x, spaceship.position.y, spaceship.position.z); // 随飞船移动
	glTranslatef(astronaut.position.x, astronaut.position.y, astronaut.position.z); // 相对位置

	// 头部（黄色）
	glPushMatrix();
	glColor3f(1.0, 1.0, 0.0); // 黄色
	glTranslatef(0, 0.5, 0);
	glutSolidSphere(0.2, 20, 20);
	glPopMatrix();

	// 身体（红色）
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0); // 红色
	glTranslatef(0, 0, 0);
	glutSolidCube(0.5);
	glPopMatrix();

	// 左腿（蓝色）
	glPushMatrix();
	glColor3f(0.0, 0.0, 1.0); // 蓝色
	glTranslatef(-0.1, -0.25, 0);
	glRotatef(astronaut.step, 1, 0, 0);
	glutSolidCylinder(0.1, 0.5, 20, 20);
	glPopMatrix();

	// 右腿（蓝色）
	glPushMatrix();
	glColor3f(0.0, 0.0, 1.0); // 蓝色
	glTranslatef(0.1, -0.25, 0);
	glRotatef(-astronaut.step, 1, 0, 0);
	glutSolidCylinder(0.1, 0.5, 20, 20);
	glPopMatrix();

	// 左臂（绿色）
	glPushMatrix();
	glColor3f(0.0, 1.0, 0.0); // 绿色
	glTranslatef(-0.25, 0.25, 0);
	glutSolidCylinder(0.1, 0.5, 20, 20);
	glPopMatrix();

	// 右臂（绿色）
	glPushMatrix();
	glColor3f(0.0, 1.0, 0.0); // 绿色
	glTranslatef(0.25, 0.25, 0);
	glutSolidCylinder(0.1, 0.5, 20, 20);
	glPopMatrix();

	glPopMatrix();
}
void DrawShip() {
	float scale = 1.0f;       // 基础比例，内部部件尺寸相关
	float shipScale = 2.0f;   // 飞船整体缩放因子，调节大小

	GLUquadric* quad = gluNewQuadric();

	glPushMatrix();

	// 飞船整体位置和旋转
	glTranslatef(spaceship.position.x, spaceship.position.y, spaceship.position.z);
	glRotatef(spaceship.angleX, 1, 0, 0);
	glRotatef(spaceship.angleY, 0, 1, 0);

	// 整体缩放
	glScalef(shipScale, shipScale, shipScale);

	// 1. 机身圆柱体（沿Z+方向）
	glPushMatrix();
	glColor3f(0.6f, 0.6f, 0.6f); // 灰色机身
	gluCylinder(quad, 0.4f * scale, 0.4f * scale, 2.0f * scale, 30, 30);
	// 前盖
	glPushMatrix();
	glTranslatef(0, 0, 2.0f * scale);
	gluDisk(quad, 0.0, 0.4f * scale, 30, 1);
	glPopMatrix();
	// 后盖
	gluDisk(quad, 0.0, 0.4f * scale, 30, 1);
	glPopMatrix();

	// 2. 锥形机头（机身前端）
	glPushMatrix();
	glColor3f(1.0f, 0.0f, 0.0f); // 红色
	glTranslatef(0, 0, 2.0f * scale);
	glutSolidCone(0.3f * scale, 0.7f * scale, 20, 20);
	glPopMatrix();

	// 3. 舷窗（黑色小球）
	glPushMatrix();
	glColor3f(0.0f, 0.0f, 0.0f);
	glTranslatef(0, 0.2f * scale, 1.0f * scale);
	glutSolidSphere(0.1f * scale, 10, 10);
	glPopMatrix();

	// 4. 左右机翼（三角形）
	for (int i = -1; i <= 1; i += 2) {
		glPushMatrix();
		glColor3f(1.0f, 1.0f, 0.0f); // 黄色机翼
		glTranslatef(i * 0.5f * scale, 0, 1.0f * scale);
		glBegin(GL_TRIANGLES);
		glVertex3f(0, 0.0f, 0.0f);
		glVertex3f(i * 1.0f * scale, 0.3f * scale, 0.0f);
		glVertex3f(i * 1.0f * scale, -0.3f * scale, 0.0f);
		glEnd();
		glPopMatrix();
	}

	// 5. 后部推进器（圆柱，紧贴机舱尾部，两侧）
	for (int i = -1; i <= 1; i += 2) {
		glPushMatrix();
		glColor3f(0.2f, 0.4f, 1.0f); // 蓝色推进器
		glTranslatef(i * 0.5f * scale, -0.1f * scale, 0.0f);
		glScalef(1.0f, 1.0f, -1.0f); // 反转Z方向，使推进器朝Z负方向
		gluCylinder(quad, 0.1f * scale, 0.1f * scale, 0.6f * scale, 20, 20);
		glPopMatrix();
	}

	glPopMatrix();

	gluDeleteQuadric(quad);
}



void DrawPlanets()
{
	for (size_t i = 0; i < planets.size(); ++i) {
		// 绘制轨道
		glPushMatrix();
		glColor3f(0.5f, 0.5f, 0.5f);  // 轨道颜色灰色

		glBegin(GL_LINE_LOOP);
		if (i == 4) { // 月球轨道绕地球
			for (float angle = 0; angle < 2 * PI; angle += 0.01f) {
				glVertex3f(
					planets[3].position.x + planets[i].orbitRadius * cos(angle),
					0,
					planets[3].position.z + planets[i].orbitRadius * sin(angle)
				);
			}
		}
		else { // 其他轨道绕太阳原点
			for (float angle = 0; angle < 2 * PI; angle += 0.01f) {
				glVertex3f(planets[i].orbitRadius * cos(angle), 0, planets[i].orbitRadius * sin(angle));
			}
		}
		glEnd();
		glPopMatrix();

		// 计算位置
		if (i == 4) { // 月球相对地球位置
			planets[i].position = CVector(
				planets[3].position.x + planets[i].orbitRadius * cos(planets[i].angle),
				0,
				planets[3].position.z + planets[i].orbitRadius * sin(planets[i].angle)
			);
		}
		else { // 其他行星绕太阳
			planets[i].position = CVector(
				planets[i].orbitRadius * cos(planets[i].angle),
				0,
				planets[i].orbitRadius * sin(planets[i].angle)
			);
		}
		// 绘制行星
		glPushMatrix();
		glTranslatef(planets[i].position.x, planets[i].position.y, planets[i].position.z);
		glRotatef(planets[i].angle * 180.0f / PI, 0, 1, 0);

		if (wireframeMode) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glColor3fv(planets[i].color);
			glutSolidSphere(planets[i].radius, 20, 20);
		}
		else {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			// 绑定纹理
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, texture[i]);

			// 设置颜色为白色保证纹理色彩正常显示
			glColor3f(1.0f, 1.0f, 1.0f);

			DrawTexturedSphere(planets[i].radius, 40, 40);

			glDisable(GL_TEXTURE_2D);
		}

		glPopMatrix();
	}

	// 土星环（假设为第7个行星）绘制，不带纹理，保持不变
	if (!wireframeMode) {
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, texture[8]);

		glPushMatrix();
		glTranslatef(planets[7].position.x, planets[7].position.y, planets[7].position.z);
		glColor3f(1.0f, 1.0f, 1.0f); // 使用白色避免影响纹理颜色

		// 绘制带纹理的环面（用 quad strip 更精细可以替换 glutSolidTorus）
		GLUquadric* ring = gluNewQuadric();
		gluQuadricTexture(ring, GL_TRUE);   // 允许纹理贴图
		gluQuadricNormals(ring, GLU_SMOOTH);
		gluDisk(ring, 0.2f * 7, 1.0f * 7, 60, 1); // 使用圆环模拟土星环
		gluDeleteQuadric(ring);

		glPopMatrix();
		glDisable(GL_TEXTURE_2D);
	}

	// 选中行星线框绘制（如果需要）
	if (selectedPlanet != -1) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glPushMatrix();
		glTranslatef(planets[selectedPlanet].position.x, planets[selectedPlanet].position.y, planets[selectedPlanet].position.z);
		glColor3f(1.0f, 1.0f, 1.0f);
		glutSolidSphere(planets[selectedPlanet].radius, 20, 20);
		glPopMatrix();
		glPolygonMode(GL_FRONT_AND_BACK, wireframeMode ? GL_LINE : GL_FILL);
	}
}

void draw()
{
    glPushMatrix();
    glRotatef(seta, 1, 1, 0);
    DrawStars();
    glPopMatrix();

    // 绘制行星
    glPushMatrix();
    DrawPlanets();
    glPopMatrix();

    // 绘制飞船
    glPushMatrix();
    DrawShip();
    glPopMatrix();

	// 新增绘制太空人
	glPushMatrix();
	DrawAstronaut();
	glPopMatrix();

	
}
void myDisplay(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();  // 重置模型视图矩阵
    /*gluLookAt(7.0, 7.0, 7.0,  // 摄像机位置
        0.0, 0.0, 0.0,  // 观察目标点
        0.0, 1.0, 0.0); // 上方向*/
	if (isInterpolating) {
		// 计算插值进度
		float t = (float)interpolationFrame / interpolationTotalFrames;

		// 插值计算视点位置
		mx = initialPosition.x + (targetPosition.x - initialPosition.x) * t;
		my = initialPosition.y + (targetPosition.y - initialPosition.y) * t;
		mz = initialPosition.z + (targetPosition.z - initialPosition.z) * t;

		// 使用四元数插值计算当前旋转
		currentQuat = initialQuat.Slerp(targetQuat, t);

		// 更新插值帧数
		interpolationFrame++;
		if (interpolationFrame >= interpolationTotalFrames) {
			// 插值完成
			isInterpolating = false;
			isAstronautView = !isAstronautView; // 切换视点模式
		}
	}

	bool bChange = false;
	if (keyState['w'])
	{
		if (isAstronautView)
		{
			//运动
			astronaut.position.z += cos(astronaut.angle * 3.1415 / 180.0) * mspeed;
			astronaut.position.x += sin(astronaut.angle * 3.1415 / 180.0) * mspeed;
			astronaut.position.x = clamp(astronaut.position.x, -0.75f, 0.75f);
			astronaut.position.y = clamp(astronaut.position.y, -0.75f, 0.75f);
			astronaut.position.z = clamp(astronaut.position.z, -0.75f, 0.75f);
		}
		else if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0, -mspeed, 0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx += g_IEyeMat[4] * mspeed;
			my += g_IEyeMat[5] * mspeed;
			mz += g_IEyeMat[6] * mspeed;
		}
	}
	if (keyState['s'])
	{
		if (isAstronautView)
		{
			//运动
			astronaut.position.z -= cos(astronaut.angle * 3.1415 / 180.0) * mspeed;
			astronaut.position.x -= sin(astronaut.angle * 3.1415 / 180.0) * mspeed;
			astronaut.position.x = clamp(astronaut.position.x, -0.75f, 0.75f);
			astronaut.position.y = clamp(astronaut.position.y, -0.75f, 0.75f);
			astronaut.position.z = clamp(astronaut.position.z, -0.75f, 0.75f);
		}
		else if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0, mspeed, 0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx -= g_IEyeMat[4] * mspeed;
			my -= g_IEyeMat[5] * mspeed;
			mz -= g_IEyeMat[6] * mspeed;
		}
	}
	if (keyState['a'])
	{
		if (isAstronautView)
		{
			//运动
			astronaut.angle += rspeed;
			astronaut.position.x = clamp(astronaut.position.x, -0.75f, 0.75f);
			astronaut.position.y = clamp(astronaut.position.y, -0.75f, 0.75f);
			astronaut.position.z = clamp(astronaut.position.z, -0.75f, 0.75f);
		}
		else if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(mspeed, 0, 0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx -= g_IEyeMat[0] * mspeed;
			my -= g_IEyeMat[1] * mspeed;
			mz -= g_IEyeMat[2] * mspeed;
		}
	}
	if (keyState['d'])
	{
		if (isAstronautView)
		{
			//运动
			astronaut.angle -= rspeed;
			astronaut.position.x = clamp(astronaut.position.x, -0.75f, 0.75f);
			astronaut.position.y = clamp(astronaut.position.y, -0.75f, 0.75f);
			astronaut.position.z = clamp(astronaut.position.z, -0.75f, 0.75f);
		}
		else if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(-mspeed, 0, 0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx += g_IEyeMat[0] * mspeed;
			my += g_IEyeMat[1] * mspeed;
			mz += g_IEyeMat[2] * mspeed;
		}

	}
	if (keyState['q'])
	{
		if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0, 0, mspeed);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx -= g_IEyeMat[8] * mspeed;
			my -= g_IEyeMat[9] * mspeed;
			mz -= g_IEyeMat[10] * mspeed;
		}
	}
	if (keyState['e'])
	{
		if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0, 0, -mspeed);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx += g_IEyeMat[8] * mspeed;
			my += g_IEyeMat[9] * mspeed;
			mz += g_IEyeMat[10] * mspeed;
		}
	}
	if (keyState['i'])
	{
		if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(-rspeed, 1, 0, 0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rx += rspeed;
			bChange = true;
		}
	}
	if (keyState['k'])
	{
		if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(rspeed, 1, 0, 0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rx -= rspeed;
			bChange = true;
		}
	}
	if (keyState['j'])
	{
		if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(-rspeed, 0, 1, 0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			ry += rspeed;
			bChange = true;
		}
	}
	if (keyState['l'])
	{
		if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(rspeed, 0, 1, 0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			ry -= rspeed;
			bChange = true;
		}
	}
	if (keyState['u'])
	{
		if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(rspeed, 0, 0, 1);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rz += rspeed;
			bChange = true;
		}
	}
	if (keyState['o'])
	{
		if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(-rspeed, 0, 0, 1);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rz -= rspeed;
			bChange = true;
		}
	}
	if (keyState['0'])
	{
		mode = 1 - mode;
		if (mode == 0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(-rz, 0, 0, 1);
			glRotatef(-rx, 1, 0, 0);
			glRotatef(-ry, 0, 1, 0);
			glTranslatef(-mx, -my, -mz);
			glGetFloatv(GL_MODELVIEW_MATRIX, g_EyeMat);
			glPopMatrix();
		}
		printf("mode:%d\n", mode);
	}
	if (bChange)//计算视点矩阵的逆矩阵
	{
		glPushMatrix();
		glLoadIdentity();
		glRotatef(ry, 0, 1, 0);
		glRotatef(rx, 1, 0, 0);
		glRotatef(rz, 0, 0, 1);
		glGetFloatv(GL_MODELVIEW_MATRIX, g_IEyeMat);
		glPopMatrix();
	}
	glutPostRedisplay(); // 重新绘制窗口
    glPushMatrix();
	
	SetView();
	SetupSunLight();
	SetupShipSpotlight();
	SetupInternalLight();
  
	GLfloat amb[4] = { 0.4f,0.4f,0.4f,1 };
	GLfloat dif[4] = { 1,1,1,1 };
	GLfloat spe[4] = { 0.1f,0.1f,0.1f,1 };


	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, dif);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spe);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    draw();

    glPopMatrix();
    glutSwapBuffers();

	

}
void DrawStars() {
    glColor3f(1.0f, 1.0f, 1.0f); // 设置星星颜色为白色
    glPointSize(2.0f); // 设置星星大小
    glBegin(GL_POINTS);
    for (int i = 0; i < 100; ++i) {
        float brightness = sin(t + i) * 0.5f + 0.5f; // 计算星星亮度
        glColor3f(brightness, brightness, brightness); // 设置星星亮度
        glVertex3fv(stars[i]); // 绘制星星
    }
    glEnd();
}
void myReshape(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, GLfloat(w) / h, 1, 1000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
void myKeyboard(unsigned char key, int x, int y) {
	keyState[key] = true; // 设置按键状态为按下
	if (key == 'c' || key == 'C') { // 按下 C 键切换视点模式
		if (!isInterpolating) {
			isInterpolating = true;
			interpolationFrame = 0;

			// 存储初始状态
			initialPosition = CVector(mx, my, mz);
			initialQuat = CQuaternion(cos(rx * PI / 360),
				sin(rx * PI / 360) * sin(ry * PI / 360),
				sin(rx * PI / 360) * cos(ry * PI / 360),
				0); // 初始化四元数

			if (isAstronautView) {
				// 存储目标状态为上帝视角
				targetPosition = CVector(godView_mx, godView_my, godView_mz);
				targetQuat = CQuaternion(cos(godView_rx * PI / 360),
					sin(godView_rx * PI / 360) * sin(godView_ry * PI / 360),
					sin(godView_rx * PI / 360) * cos(godView_ry * PI / 360),
					0);
			}
			else {
				// 存储目标状态为宇航员视角
				targetPosition = CVector(spaceship.position.x, spaceship.position.y + 0.5f, spaceship.position.z - 1.0f);
				// 使用四元数表示朝向飞船的旋转
				targetQuat = CQuaternion(cos(-90 * PI / 360),
					sin(-90 * PI / 360) * sin(spaceship.angleY * PI / 360),
					sin(-90 * PI / 360) * cos(spaceship.angleY * PI / 360),
					0);
			}
		}
	}

    glutPostRedisplay(); // 请求重新绘制窗口
   
}
void mySpecialKeyboard(int key, int x, int y) {
    if (key == GLUT_KEY_F1) { // 按下 F1 键
        // 切换填充模式和线框模式
        wireframeMode = !wireframeMode;
    }
    glutPostRedisplay(); // 重新绘制窗口
}
void KeyUp(unsigned char key, int x, int y) {
	keyState[key] = false; // 设置按键状态为未按下
	glutPostRedisplay(); // 请求重新绘制窗口
}
int main(int argc, char* argv[])
{
    Calculate2();
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    srand((unsigned int)time(NULL)); // 初始化随机数种子

    for (int i = 0; i < 100; ++i) {
        stars[i].Set(rand() % 40 - 20, rand() % 40 - 20, rand() % 40 - 20, 1);
        starBrightness[i] = rand() % 100 / 100.0f;
    }// 初始化星星的位置
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(1024, 768);
	InitLighting();
    glutCreateWindow("图形学作业5");
    glutDisplayFunc(&myDisplay);
    glutTimerFunc(33, myTimerFunc, 0);
    glutReshapeFunc(&myReshape);
    glutKeyboardFunc(&myKeyboard);
    glutMouseFunc(&mouse);
    glutSpecialFunc(mySpecialKeyboard); // 注册特殊键回调函数
    glutKeyboardUpFunc(&KeyUp);
    SetRC();
    glutMainLoop();
    return 0;
}