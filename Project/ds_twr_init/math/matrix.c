#include "matrix.h"

// TODO: fix this dependency
#include "kalman/kalman.h"

#include <stm32f10x.h>
#include <math.h>

void MatrixAdd(float *a, float *b, float *c, unsigned char m, unsigned char n)
{
  unsigned char i;

  for (i=0; i<m*n; i++)
  {
    c[i] = a[i] + b[i];
  }
}

void MatrixMinus(double *a, float *b, float *c, unsigned char m, unsigned char n)
{
  unsigned char i;

  for (i=0; i<m*n; i++)
  {
    c[i] = (float)a[i] - b[i];
  }
}

void MatrixMul(float *a, float *b, float *c, unsigned char m, unsigned char p, unsigned char n)
{
  unsigned char i,j,k;

  for (i=0; i<m; i++)
  {
    for (j=0; j<n; j++)
    {
      c[i*n+j] = 0.0;
      for (k=0; k<p; k++)
      {
        c[i*n+j] += a[i*p+k] * b[k*n+j];
      }
    }
  }
}

void MatrixTrans(float *a, float *c, unsigned char m, unsigned char n)
{
  unsigned char i,j;

  for (i=0; i<n; i++)
  {
    for (j=0; j<m; j++)
    {
      c[i*m+j] = a[j*n+i];
    }
  }
}

float MatrixDet1(float *a, unsigned char m, unsigned char n)
{
  signed char i, j, k, p, r;
  float Temp=1, Temp1=1, S=0, S1=0;
  float X;

  if (n==2)
  {
    for(i=0; i<m; i++)
    {
      for(j=0 ;j<n; j++)
      {
        if((i+j)%2)
        {
          Temp1 *= a[i*n+j];
        }
        else
        {
          Temp  *= a[i*n+j];
        }
      }
    }
    X=Temp-Temp1;
  }
  else
  {
    for (k=0; k<n; k++)
    {
      for (i=0,j=k; i<m&&j<n; i++,j++)
      {
        Temp *= a[i*n+j];
      }
      if (m-i)
      {
        for (p=m-i,r=m-1; p>0; p--,r--)
        {
          Temp  *= a[r*n+p-1];
        }
      }
      S += Temp;
      Temp = 1;
    }


    for (k=n-1; k>=0; k--)
    {
      for(i=0,j=k; i<m&&j>=0; i++,j--)
      {
        Temp1 *= a[i*n+j];
      }
      if (m-i)
      {
        for(p=m-1,r=i; r<m; p--,r++)
        {
          Temp1 *= a[r*n+p];
        }
      }
      S1 += Temp1;
      Temp1 = 1;
    }
    X=S-S1;
  }

  return   X;
}

void MatrixInv1(float *a, float *c, unsigned char m, unsigned char n)
{
  unsigned char i,j,k,x,y;
  float AB[LENGTH], SP[LENGTH], B[LENGTH];
  float X;

  X = MatrixDet1(a, m, n);
  X = 1/X;

  for (i=0; i<m; i++)
  {
    for (j=0; j<n; j++)
    {
      for (k=0; k<m*n; k++)
      {
        B[k] = a[k];
      }
      for(x=0; x<n; x++)
      {
        B[i*n+x] = 0;
      }
      for(y=0; y<m; y++)
      {
        B[m*y+j] = 0;
      }
      B[i*n+j] = 1;
      SP[i*n+j] = MatrixDet1(B,m,n);
      AB[i*n+j] = X*SP[i*n+j];
    }
  }
  MatrixTrans(AB, c, m, n);
}

unsigned char Gauss_Jordan(float *a, unsigned char n)
{
    signed char i,j,k,l,u,v;
    signed char is[ORDER];
    signed char js[ORDER];
    float d,p;

    for (k=0; k<=n-1; k++)
      { d=0.0;
        for (i=k; i<=n-1; i++)
        for (j=k; j<=n-1; j++)
          { l=i*n+j; p=fabs(a[l]);
            if (p>d) { d=p; is[k]=i; js[k]=j;}
          }
        if (d+1.0==1.0)
          {
            return(0);
          }
        if (is[k]!=k)
          for (j=0; j<=n-1; j++)
            { u=k*n+j; v=is[k]*n+j;
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        if (js[k]!=k)
          for (i=0; i<=n-1; i++)
            { u=i*n+k; v=i*n+js[k];
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        l=k*n+k;
        a[l]=1.0/a[l];
        for (j=0; j<=n-1; j++)
          if (j!=k)
            { u=k*n+j; a[u]=a[u]*a[l];}
        for (i=0; i<=n-1; i++)
          if (i!=k)
            for (j=0; j<=n-1; j++)
              if (j!=k)
                { u=i*n+j;
                  a[u]=a[u]-a[i*n+k]*a[k*n+j];
                }
        for (i=0; i<=n-1; i++)
          if (i!=k)
            { u=i*n+k; a[u]=-a[u]*a[l];}
      }
    for (k=n-1; k>=0; k--)
      { if (js[k]!=k)
          for (j=0; j<=n-1; j++)
            { u=k*n+j; v=js[k]*n+j;
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        if (is[k]!=k)
          for (i=0; i<=n-1; i++)
            { u=i*n+k; v=i*n+is[k];
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
      }

    return(1);
}

void MatrixCal(float *a, float *b, float *c, unsigned char n)
{
  float Temp1[LENGTH] = {0};
  float Temp2[LENGTH] = {0};

  MatrixMul(a, b, Temp1, n, n, n);
  MatrixTrans(a, Temp2, n, n);
  MatrixMul(Temp1, Temp2, c, n, n, n);
}
