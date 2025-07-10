// 001.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "stdio.h"
#include "glut.h"
#include "math.h"
//模型旋转控制
float seta = 0;
int axis=0;
//////////////////////////////////////////////////////////////////////////
//视点位置和方向
float mx=0,my=500,mz=1000,rx=-25,ry=0,rz=0;//平移和旋转
float sx=1,sy=1,sz=1;//缩放
float mspeed=5,rspeed=1;
float g_IEyeMat[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1},g_EyeMat[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
int mode=1;
//////////////////////////////////////////////////////////////////////////
void myDisplay(void);

void myTimerFunc(int val)
{	
	seta+=1;
	myDisplay();
	glutTimerFunc(1,myTimerFunc,0);
}

void myKeyboardUpFunc(unsigned char key,int x, int y)
{
	printf("%c Up\n",key);
}
void myKeyboardFunc(unsigned char key,int x, int y)
{
	printf("%c Down\n",key);
	bool bChange=false;
	switch(key)
	{
	case 'w':
		//my+=mspeed;
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0,-mspeed,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx+=g_IEyeMat[4]*mspeed;
			my+=g_IEyeMat[5]*mspeed;
			mz+=g_IEyeMat[6]*mspeed;
		}
		break;
	case 's':
		//my-=mspeed;	
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0,mspeed,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx-=g_IEyeMat[4]*mspeed;
			my-=g_IEyeMat[5]*mspeed;
			mz-=g_IEyeMat[6]*mspeed;
		}
		
		break;
	case 'a':
		//mx-=mspeed;
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(mspeed,0,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx-=g_IEyeMat[0]*mspeed;
			my-=g_IEyeMat[1]*mspeed;
			mz-=g_IEyeMat[2]*mspeed;
		}
		
		break;
	case 'd':
		//mx+=mspeed;
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(-mspeed,0,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx+=g_IEyeMat[0]*mspeed;
			my+=g_IEyeMat[1]*mspeed;
			mz+=g_IEyeMat[2]*mspeed;
		}
		
		break;
	case 'q':
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0,0,mspeed);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx-=g_IEyeMat[8]*mspeed;
			my-=g_IEyeMat[9]*mspeed;
			mz-=g_IEyeMat[10]*mspeed;
		}
		//mz-=mspeed;
		
		break;
	case 'e':
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glTranslatef(0,0,-mspeed);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			mx+=g_IEyeMat[8]*mspeed;
			my+=g_IEyeMat[9]*mspeed;
			mz+=g_IEyeMat[10]*mspeed;
		}
		//mz+=mspeed;
		
		break;
	case 'i':
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(-rspeed,1,0,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rx+=rspeed;
			bChange = true;
		}
		break;
	case 'k':
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(rspeed,1,0,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rx-=rspeed;
			bChange = true;
		}
		
		break;
	case 'j':
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(-rspeed,0,1,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			ry+=rspeed;
			bChange = true;
		}
		
		break;
	case 'l':
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(rspeed,0,1,0);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			ry-=rspeed;
			bChange = true;
		}
		
		break;
	case 'u':
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(rspeed,0,0,1);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rz+=rspeed;
			bChange = true;
		}
		
		break;
	case 'o':
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();
			glRotatef(-rspeed,0,0,1);
			glMultMatrixf(g_EyeMat);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		else
		{
			rz-=rspeed;
			bChange = true;
		}
		
		break;
	case '1':
		sx+=0.1;
		break;
	case '2':
		sx-=0.1;
		break;
	case '3':
		sy+=0.1;
		break;
	case '4':
		sy-=0.1;
		break;
	case '5':
		sz+=0.1;
		break;
	case '6':
		sz-=0.1;
		break;
	case '0':
		mode = 1-mode;
		if(mode==0)
		{
			glPushMatrix();
			glLoadIdentity();		
			glRotatef(-rz,0,0,1);
			glRotatef(-rx,1,0,0);
			glRotatef(-ry,0,1,0);
			glTranslatef(-mx,-my,-mz);
			glGetFloatv(GL_MODELVIEW_MATRIX,g_EyeMat);
			glPopMatrix();
		}
		printf("mode:%d\n",mode);
		break;
	case '=':
		mspeed*=1.1;
		printf("mspeed:%.1f\n",mspeed);
		break;
	case '-':
		mspeed*=0.9;
		printf("mspeed:%.1f\n",mspeed);
		break;
	case ' ':
		seta=0;
		if(axis<2)
			axis++;
		else axis=0;
		break;
	}
	if(bChange)//计算视点矩阵的逆矩阵
	{
		glPushMatrix();
		glLoadIdentity();		
		glRotatef(ry,0,1,0);
		glRotatef(rx,1,0,0);
		glRotatef(rz,0,0,1);
		glGetFloatv(GL_MODELVIEW_MATRIX,g_IEyeMat);
		glPopMatrix();
	}
}
void SetRC()
{
	glShadeModel(GL_FLAT);
	glFrontFace(GL_CW);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glPolygonMode(GL_BACK,GL_LINE);
	//
}
void SetView()
{
	if(mode==0)
	{
		glLoadMatrixf(g_EyeMat);
	}
	else
	{
		glRotatef(-rz,0,0,1);	
		glRotatef(-rx,1,0,0);
		glRotatef(-ry,0,1,0);
		glTranslatef(-mx,-my,-mz);
	}
}
void RenderWorld()
{
	//绘制坐标轴
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3f(1,0,0);
	glVertex3f(0,0,0);
	glVertex3f(1000,0,0);
	glColor3f(0,1,0);
	glVertex3f(0,0,0);
	glVertex3f(0,1000,0);
	glColor3f(0,0,1);
	glVertex3f(0,0,0);
	glVertex3f(0,0,1000);
	glEnd();
	//////////////////////////////////////////////////////////////////////////
	//绘制一个地球
	glEnable(GL_LIGHTING);
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT,GL_LINE);
	float col1[3]={2,2,2};
	glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,col1);
	glPushMatrix();
	float r=5000;
	
	glTranslatef(0,-r/100,0);
	glScalef(1,0.01,1);
	glutSolidSphere(r,5000,100);	//绘制模型
	glPopMatrix();
	glPopAttrib();
	//绘制一个旋转的茶壶
	if(axis==0)
		glRotatef(seta,1,0,0);
	else if(axis==1)
		glRotatef(seta,0,1,0);
	else
		glRotatef(seta,0,0,1);
	float col2[3]={1,1,0};
	glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,col2);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glutSolidTeapot(100);
}
void myDisplay(void)
{
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	SetView();
	RenderWorld();
	glPopMatrix();
	glutSwapBuffers();
}

void myReshape(int w,int h)
{	
	GLfloat nRange = 100.0f;
	glViewport(0,0,w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60,GLfloat(w)/h,1,1000000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(400, 400);
    glutCreateWindow("第一个OpenGL程序");
    glutDisplayFunc(&myDisplay);
	glutTimerFunc(1,myTimerFunc,0);
	glutReshapeFunc(&myReshape);
	glutKeyboardUpFunc(&myKeyboardUpFunc);
	glutKeyboardFunc(&myKeyboardFunc);
	SetRC();
    glutMainLoop();
    return 0;
}