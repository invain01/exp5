
#include "stdafx.h"
#include "string.h"
#include "math.h"
#include "CMatrix.h"
float c[16];

bool Calculate1()
{
	FILE* fp,*fpOut;
	char str[1024];
	
	fopen_s(&fp, "test.txt", "rt");
	fopen_s(&fpOut, "out.txt", "wt");	
	while (!feof(fp))
	{
		fscanf_s(fp, "%s\n", str, 1024);
		
		if (strcmp(str, "æÿ’Ûœ‡≥À") == 0)
		{
			fprintf_s(fpOut, "%s\n", str);
			CMatrix m1,m2,mout;
			for (int i = 0;i < 16;i++)
			{
				fscanf_s(fp, "%f,", &c[i]);
				fprintf_s(fpOut, "%.2f", c[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			m1.Set(c);
			for (int i = 0;i < 16;i++)
			{
				fscanf_s(fp, "%f,", &c[i]);
				fprintf_s(fpOut, "%.2f", c[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			m2.Set(c);
			mout = m1 * m2;
			float* arr = mout;
			for (int i = 0;i < 16;i++)
			{
				fprintf_s(fpOut, "%.2f", arr[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
            continue;
		}
		if (strcmp(str, "æÿ’Û≥ÀœÚ¡ø" )==0) {
			CMatrix m1;
			CVector v1;
			CVector vout;
			fprintf_s(fpOut, "%s\n", str);
			for (int i = 0;i < 16;i++)
			{
				fscanf_s(fp, "%f,", &c[i]);
				fprintf_s(fpOut, "%.2f", c[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "	");
			m1.Set(c);
			fscanf_s(fp,"%f,%f,%f", &v1.x, &v1.y, &v1.z);
			fprintf_s(fpOut,"%.2f,%.2f,%.2f\n",v1.x,v1.y,v1.z);
			vout = m1.vecMul(v1);
			fprintf_s(fpOut, "%.2f,%.2f,%.2f\n", vout.x, vout.y, vout.z);
			continue;
		}
		if (strcmp(str, "æÿ’Û≥ÀŒª÷√") == 0) {
			CMatrix m1;
			CVector v1;
			CVector vout;
			fprintf_s(fpOut, "%s\n", str);
			for (int i = 0;i < 16;i++)
			{
				fscanf_s(fp, "%f,", &c[i]);
				fprintf_s(fpOut, "%.2f", c[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "	");
			m1.Set(c);
			fscanf_s(fp, "%f,%f,%f", &v1.x, &v1.y, &v1.z);
			fprintf_s(fpOut, "%.2f,%.2f,%.2f\n", v1.x, v1.y, v1.z);
			vout = m1.posMul(v1);
			fprintf_s(fpOut, "%.2f,%.2f,%.2f\n", vout.x, vout.y, vout.z);
			continue;
		}
		if (strcmp(str, "æÿ’Û…Ë÷√–˝◊™") == 0) {
			float x;
			CVector v1;
			CMatrix mout;
			fprintf_s(fpOut, "%s\n", str);
			fscanf_s(fp, "%f", &x);
			fprintf_s(fpOut, "%.2f\t", x);
			fscanf_s(fp, "%f,%f,%f", &v1.x, &v1.y, &v1.z);
			fprintf_s(fpOut, "%.2f,%.2f,%.2f\n", v1.x, v1.y, v1.z);
			mout.SetRotate(x, v1);
			float* arr = mout;
			for (int i = 0;i < 16;i++)
			{
				fprintf_s(fpOut, "%.2f", arr[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			continue;

		}
		if (strcmp(str, "æÿ’Û…Ë÷√∆Ω“∆") == 0) {
			CVector v1;
			CMatrix mout;
			fprintf_s(fpOut, "%s\n", str);
			fscanf_s(fp, "%f,%f,%f", &v1.x, &v1.y, &v1.z);
			fprintf_s(fpOut, "%.2f,%.2f,%.2f\n", v1.x, v1.y, v1.z);
			mout.SetTrans( v1);
			float* arr = mout;
			for (int i = 0;i < 16;i++)
			{
				fprintf_s(fpOut, "%.2f", arr[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			continue;

		}
		if (strcmp(str, "æÿ’Û…Ë÷√Àı∑≈") == 0) {
			CVector v1;
			CMatrix mout;
			fprintf_s(fpOut, "%s\n", str);
			fscanf_s(fp, "%f,%f,%f", &v1.x, &v1.y, &v1.z);
			fprintf_s(fpOut, "%.2f,%.2f,%.2f\n", v1.x, v1.y, v1.z);
			mout.SetScale(v1);
			float* arr = mout;
			for (int i = 0;i < 16;i++)
			{
				fprintf_s(fpOut, "%.2f", arr[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			continue;

		}
		if (strcmp(str, "æÿ’Û«ÛƒÊ") == 0) {
			CMatrix m1;
			CMatrix mout;
			fprintf_s(fpOut, "%s\n", str);
			for (int i = 0;i < 16;i++)
			{
				fscanf_s(fp, "%f,", &c[i]);
				fprintf_s(fpOut, "%.2f", c[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			m1.Set(c);
			mout=m1.GetInverse();
			float* arr = mout;
			for (int i = 0;i < 16;i++)
			{
				fprintf_s(fpOut, "%.2f", arr[i]);
				if (i != 15) fprintf_s(fpOut, ",");
			}
			fprintf_s(fpOut, "\n");
			continue;

		}
	}	
	fclose(fp);
	fclose(fpOut);
	return false;
}