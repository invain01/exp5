
#include "stdafx.h"
#include "string.h"
#include "math.h"
#include "CVector.h"


bool Calculate()
{
    FILE* fp, * fpOut;
    char str[1024];
    fopen_s(&fp, "test.txt", "rt");
    fopen_s(&fpOut, "out.txt", "wt");
    while (!feof(fp))
    {
        fscanf_s(fp, "%s\n", str, 1024);
        if (strcmp(str, "向量相加") == 0)
        {
            CVector v1, v2, vout;
            fscanf_s(fp, "%f,%f,%f %f,%f,%f", &v1.x, &v1.y, &v1.z, &v2.x, &v2.y, &v2.z);
            vout = v1 + v2;
            fprintf(fpOut, "%s\n%g,%g,%g \t%g,%g,%g \t%g,%g,%g\n", str, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, vout.x, vout.y, vout.z);
            continue;
        }
        if (strcmp(str, "向量点乘") == 0)
        {
            CVector v1, v2;
            float out;
            fscanf_s(fp, "%f,%f,%f %f,%f,%f", &v1.x, &v1.y, &v1.z, &v2.x, &v2.y, &v2.z);
            out = v1.dotMul(v2);
            fprintf(fpOut, "%s\n%g,%g,%g \t%g,%g,%g \t%g\n", str, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, out);
            continue;
        }
        if (strcmp(str, "向量叉乘") == 0)
        {
            CVector v1, v2, vout;
            fscanf_s(fp, "%f,%f,%f %f,%f,%f", &v1.x, &v1.y, &v1.z, &v2.x, &v2.y, &v2.z);
            vout = v1.crossMul(v2);
            fprintf(fpOut, "%s\n%g,%g,%g \t%g,%g,%g \t%g,%g,%g\n", str, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, vout.x, vout.y, vout.z);
            continue;
        }
        if (strcmp(str, "向量标准化") == 0)
        {
            CVector v1, vout;
            fscanf_s(fp, "%f,%f,%f %f,%f,%f", &v1.x, &v1.y, &v1.z);
            vout = v1.Normalize();
            fprintf(fpOut, "%s\n%g,%g,%g \t%g,%g,%g\n", str, v1.x, v1.y, v1.z, vout.x, vout.y, vout.z);
            continue;
        }
        if (strcmp(str, "向量求模") == 0)
        {
            CVector v1;
            float out;
            fscanf_s(fp, "%f,%f,%f", &v1.x, &v1.y, &v1.z);
            out = v1.len();
            fprintf(fpOut, "%s\n%g,%g,%g \t%g\n", str, v1.x, v1.y, v1.z, out);
            continue;
        }
        if (strcmp(str, "向量投影") == 0)
        {
            CVector v1, v2, vout;
            fscanf_s(fp, "%f,%f,%f %f,%f,%f", &v1.x, &v1.y, &v1.z, &v2.x, &v2.y, &v2.z);
            vout = v1.project(v2);
            fprintf(fpOut, "%s\n%g,%g,%g \t%g,%g,%g \t%g,%g,%g\n", str, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, vout.x, vout.y, vout.z);
            continue;
        }
    }
    fclose(fp);
    fclose(fpOut);
    return false;
}