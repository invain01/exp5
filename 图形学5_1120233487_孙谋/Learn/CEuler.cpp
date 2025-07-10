
#include "stdafx.h"
#include "string.h"
#include "math.h"
#include "CEuler.h"
float c1[16];

CQuaternion CEuler::ToQuaternion() const {

	float cy = cos(h * 0.5f * (3.1415926f / 180.0f));
	float sy = sin(h * 0.5f * (3.1415926f / 180.0f));
	float cp = cos(p * 0.5f * (3.1415926f / 180.0f));
	float sp = sin(p * 0.5f * (3.1415926f / 180.0f));
	float cr = cos(b * 0.5f * (3.1415926f / 180.0f));
	float sr = sin(b * 0.5f * (3.1415926f / 180.0f));

	CQuaternion q;
	q.w = cy * cp * cr + sy * sp * sr;
	q.z = cy * cp * sr - sy * sp * cr;
	q.x = sy * cp * sr + cy * sp * cr;
	q.y = sy * cp * cr - cy * sp * sr;
	return q;

};
CEuler CVector::ToEuler() const{
	float pitch = atan2(fabs(y), sqrt(x * x + z * z)) * (180.0f / PI);

	// 计算偏航角（Yaw）
	float yaw = atan2(x, fabs(z)) * (180.0f / PI);

	// 翻滚角（Roll）通常为0，因为向量通常只表示方向
	float roll = 0.0f;

	return CEuler(yaw, pitch, roll);
}
CEuler CMatrix::ToEuler() const {
	CEuler euler;


	float sp = -m12; 
	if (fabs(sp) > 0.9999f) { // 万向锁情况（俯仰角接近 ±90 度）
		euler.p = asin(sp);
		euler.h = atan2(-m20, m00); // 偏航角（Yaw）
		euler.b = 0.0f; // 翻滚角（Roll）为0
	}
	else {
		euler.p = asin(sp);
		euler.h = atan2(m02, m22); // 偏航角（Yaw）
		euler.b = atan2(m10, m11); // 翻滚角（Roll）
	}


	// 将弧度转换为度
	euler.h = euler.h * (180.0f / PI);
	euler.p = euler.p * (180.0f / PI);
	euler.b = euler.b * (180.0f / PI);

	// 规范化欧拉角
	euler.Normalize();
	return euler;
}
CQuaternion CMatrix::ToQuaternion() const {
	CQuaternion q;
	float trace = m00 + m11 + m22;

	if (trace > 0.0f) {
		float s = 0.5f / sqrt(trace + 1.0f);
		q.w = 0.25f / s;
		q.x = (m21 - m12) * s;
		q.y = (m02 - m20) * s;
		q.z = (m10 - m01) * s;
	}
	else {
		if (m00 > m11 && m00 > m22) {
			float s = 2.0f * sqrt(1.0f + m00 - m11 - m22);
			q.w = (m21 - m12) / s;
			q.x = 0.25f * s;
			q.y = (m01 + m10) / s;
			q.z = (m02 + m20) / s;
		}
		else if (m11 > m22) {
			float s = 2.0f * sqrt(1.0f + m11 - m00 - m22);
			q.w = (m02 - m20) / s;
			q.x = (m01 + m10) / s;
			q.y = 0.25f * s;
			q.z = (m12 + m21) / s;
		}
		else {
			float s = 2.0f * sqrt(1.0f + m22 - m00 - m11);
			q.w = (m10 - m01) / s;
			q.x = (m02 + m20) / s;
			q.y = (m12 + m21) / s;
			q.z = 0.25f * s;
		}
	}

	return q;
}
bool Calculate2()
{
	FILE* fp, * fpOut;
	char str[1024];

	fopen_s(&fp, "test.txt", "rt");
	fopen_s(&fpOut, "out.txt", "wt");
	while (!feof(fp))
	{
		fscanf_s(fp, "%s\n", str, 1024);

		if (strcmp(str, "四元数点乘") == 0)
		{
			CQuaternion q1,q2;
			float qout;
			fprintf_s(fpOut, "%s\n", str);
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,",&c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\t");
			q1.Set(c1[0], c1[1], c1[2], c1[3]);
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			q2.Set(c1[0], c1[1], c1[2], c1[3]);
			qout = q1.dotMul(q2);
			fprintf_s(fpOut, "%f\n",qout);
			continue;
		}
		if (strcmp(str, "向量转换欧拉角") == 0) {
			fprintf_s(fpOut, "%s\n", str);
			CVector v1;
			CEuler e1;
			for (int i = 0;i < 3;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 2) fprintf_s(fpOut, ",");
			}
			fprintf(fpOut, "\t");
			v1.Set(c1[0], c1[1], c1[2], 1.0f);
			e1 = v1.ToEuler();
			fprintf_s(fpOut, "%f,%f,%f\n", e1.h, e1.p, e1.b);
			continue;
		}
		if (strcmp(str, "欧拉角转换四元数") == 0) {
			fprintf_s(fpOut, "%s\n", str);
			CQuaternion q1;
			CEuler e1;
			for (int i = 0;i < 3;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 2) fprintf_s(fpOut, ",");
			}
			fprintf(fpOut, "\t");
			e1.Set(c1[0], c1[1], c1[2]);
			q1 = e1.ToQuaternion();
			fprintf_s(fpOut, "%f,%f,%f,%f\n", q1.w, q1.x, q1.y, q1.z);
			continue;
		}
		if (strcmp(str, "欧拉角转换向量") == 0) {
			fprintf_s(fpOut, "%s\n", str);
			CVector v1;
			CEuler e1;
			for (int i = 0;i < 3;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 2) fprintf_s(fpOut, ",");
			}
			fprintf(fpOut, "\t");
			e1.Set(c1[0], c1[1], c1[2]);
			v1 = e1.ToVector3();
			fprintf_s(fpOut, "%f,%f,%f\n",  v1.x, v1.y, v1.z);
			continue;

		}
		if (strcmp(str, "欧拉角标准化") == 0) {
			fprintf_s(fpOut, "%s\n", str);
			CEuler e1;
			for (int i = 0;i < 3;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 2) fprintf_s(fpOut, ",");
			}
			fprintf(fpOut, "\t");
			e1.Set(c1[0], c1[1], c1[2]);
			e1.Normalize();
			fprintf_s(fpOut, "%f,%f,%f\n", e1.h, e1.p, e1.b);
			continue;

		}
		if (strcmp(str, "四元数求角度和旋转轴") == 0) {
			fprintf_s(fpOut, "%s\n", str);
			CQuaternion q1;
			float angle;
			CVector v1;
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			fprintf(fpOut, "\t");
			q1.Set(c1[0], c1[1], c1[2], c1[3]);
			q1.GetAngle(angle, v1);
			fprintf_s(fpOut,"%f \t%f,%f,%f\n",angle,v1.x,v1.y,v1.z);
			continue;

		}
		if (strcmp(str, "四元数转换矩阵") == 0) {
			
			fprintf_s(fpOut, "%s\n", str);
			CQuaternion q1;
			CMatrix m1;
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			fprintf(fpOut, "\t");
			q1.Set(c1[0], c1[1], c1[2], c1[3]);
			m1 = q1.ToMatrix();
			float* arr = m1;
			for (int i = 0;i < 16;i++)
			{
				fprintf_s(fpOut, "%f", arr[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			continue;

		}
		if (strcmp(str, "四元数转换欧拉角") == 0) {

			fprintf_s(fpOut, "%s\n", str);
			CQuaternion q1;
			CEuler e1;
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			fprintf(fpOut, "\t");
			q1.Set(c1[0], c1[1], c1[2], c1[3]);
			e1 = q1.ToEuler();
			fprintf_s(fpOut, "%f,%f,%f\n", e1.h, e1.p, e1.b);
			continue;

		}
		if (strcmp(str, "欧拉角转换矩阵") == 0) {

			fprintf_s(fpOut, "%s\n", str);
			CEuler e1;
			CMatrix m1;
			CQuaternion q1;
			for (int i = 0;i < 3;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 2) fprintf_s(fpOut, ",");
			}
			fprintf(fpOut, "\t");
			e1.Set(c1[0], c1[1], c1[2]);
			m1 = e1.ToMatrix();
			float* arr = m1;
			for (int i = 0;i < 16;i++)
			{
				fprintf_s(fpOut, "%f", arr[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			continue;

		}
		if (strcmp(str, "四元数插值") == 0) {

			fprintf_s(fpOut, "%s\n", str);
			CQuaternion q1,q2,qout;
			float t;
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			q1.Set(c1[0], c1[1], c1[2], c1[3]);
			fprintf(fpOut, "\t");
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			fprintf(fpOut, "\n");
			q2.Set(c1[0], c1[1], c1[2], c1[3]);
			fscanf_s(fp, "%f", &t);
			fprintf_s(fpOut, "%f\n", t);
			qout = q1.Slerp(q2, t);
			fprintf_s(fpOut, "%f,%f,%f,%f\n", qout.w, qout.x, qout.y, qout.z);
			continue;

		}
		if (strcmp(str, "矩阵转换欧拉角") == 0) {

			fprintf_s(fpOut, "%s\n", str);
			CEuler e1;
			CMatrix m1;
			for (int i = 0;i < 16;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "	");
			m1.Set(c1);
			fprintf(fpOut, "\t");
			e1 = m1.ToEuler();
			fprintf_s(fpOut, "%f,%f,%f\n", e1.h, e1.p, e1.b);
			continue;

		}
		if (strcmp(str, "四元数单位化") == 0) {

			fprintf_s(fpOut, "%s\n", str);
			CQuaternion q1 ,qout;
			float t;
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			q1.Set(c1[0], c1[1], c1[2], c1[3]);
			fprintf(fpOut, "\t");
			qout = q1.Normalize();
			fprintf_s(fpOut, "%f,%f,%f,%f\n", qout.w, qout.x, qout.y, qout.z);
			continue;

		}
		if (strcmp(str, "矩阵转换四元数") == 0) {

			fprintf_s(fpOut, "%s\n", str);
			CQuaternion qout;
			CMatrix m1;
			for (int i = 0;i < 16;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%.2f", c1[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "	");
			m1.Set(c1);
			qout = m1.ToQuaternion();
			fprintf_s(fpOut, "%f,%f,%f,%f\n", qout.w, qout.x, qout.y, qout.z);
			continue;

		}
		if (strcmp(str, "矩阵正交化") == 0) {

			fprintf_s(fpOut, "%s\n", str);
			CMatrix m1;
			for (int i = 0;i < 16;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%.2f", c1[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "	");
			m1.Set(c1);
			m1.Orthogonalize();
			float* arr = m1;
			for (int i = 0;i < 16;i++)
			{
				fprintf_s(fpOut, "%f", arr[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			continue;

		}
		if (strcmp(str, "四元数求逆") == 0)
		{
			CQuaternion q1, qout;
			fprintf_s(fpOut, "%s\n", str);
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\t");
			q1.Set(c1[0], c1[1], c1[2], c1[3]);
			qout=q1.GetInverse();
			fprintf_s(fpOut, "%f,%f,%f,%f\n", qout.w, qout.x, qout.y, qout.z);
			continue;
		}
		if (strcmp(str, "四元数求差") == 0)
		{
			CQuaternion q1, q2,qout;
			fprintf_s(fpOut, "%s\n", str);
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\t");
			q1.Set(c1[0], c1[1], c1[2], c1[3]);
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			q2.Set(c1[0], c1[1], c1[2], c1[3]);
			qout = q2.Div(q1);
			fprintf_s(fpOut, "%f,%f,%f,%f\n", qout.w, qout.x, qout.y, qout.z);
			continue;
		}
		if (strcmp(str, "四元数相乘") == 0)
		{
			CQuaternion q1, q2, qout;
			fprintf_s(fpOut, "%s\n", str);
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\t");
			q1.Set(c1[0], c1[1], c1[2], c1[3]);
			for (int i = 0;i < 4;i++)
			{
				fscanf_s(fp, "%f,", &c1[i]);
				fprintf_s(fpOut, "%f", c1[i]);
				if (i < 3) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			q2.Set(c1[0], c1[1], c1[2], c1[3]);
			qout = q1*q2;
			fprintf_s(fpOut, "%f,%f,%f,%f\n", qout.w, qout.x, qout.y, qout.z);
			continue;
		}
	}
	fclose(fp);
	fclose(fpOut);
	return false;
}