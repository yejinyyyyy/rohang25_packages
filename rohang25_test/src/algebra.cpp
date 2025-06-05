#include "algebra.h"

double norm(std::vector<double> x1)
{  
  // 벡터 크기 계산
  double temp = 0;
  for (const auto &i : x1)
  {
    temp += i*i;
  }
  return sqrt(temp);
}

double dot(std::vector<double> v0, std::vector<double> v1)
{
  // 스칼라곱 계산
  double out = 0;
  for (int i = 0; i < v0.size(); i++)
  {
    out += v0[i] * v1[i];
  }
  return out;
}

std::vector<double> emult(std::vector<double> v0, std::vector<double> v1)
{
  // 두 벡터의 원소별 곱 계산
  std::vector<double> out;
  for (int i = 0; i < v0.size(); i++)
  {
    out.push_back(v0[i] * v1[i]);
  }
  return out;
}

std::vector<double> mult_const(std::vector<double> v0, double v1)
{
  // 벡터의 상수배 계산
  std::vector<double> out;
  for (const auto &elem : v0)
  {
    out.push_back(elem * v1);
  }
  return out;
}

std::vector<double> eplus(std::vector<double> v0, std::vector<double> v1)
{
  // 두 벡터의 합 계산
  std::vector<double> out;
  for (int i = 0; i < v0.size(); i++)
  {
    out.push_back(v0[i] + v1[i]);
  }
  return out;
}

std::vector<double> eminus(std::vector<double> v0, std::vector<double> v1)
{
  // 두 벡터의 차 계산 : v0 - v1
  std::vector<double> out;
  for (int i = 0; i < v0.size(); i++)
  {
    out.push_back(v0[i] - v1[i]);
  }
  return out;
}

std::vector<std::vector<double>> Multiply(std::vector<std::vector<double>> aMat,
                                         std::vector<std::vector<double>> bMat)
{
  //
  std::vector<std::vector<double>> out;
  if (aMat[0].size() == bMat.size())
  {
    for (int col = 0; col < aMat.size(); col++)
    {
      for (int row = 0; row < bMat[0].size(); row++)
      {
        for (int inner = 0; inner < bMat.size(); inner++)
        {
          out[col][row] += aMat[col][inner] * bMat[inner][row];
        }
      }
    }
    return out;
  }
  return out;
}

// double point2line_dist(std::vector<double> P1, std::vector<double> P2, std::vector<double> N1)
// {
//   // 점 P1, P2를 잇는 직선과 점 N1 사이의 거리
//   // Input
//   // P1(x,y) ,  P2(x,y) ,  N1(x,y)
//   std::vector<double> Vs = eminus(P2, P1);
//   double nVs = norm(Vs);
//   std::vector<double> vec = mult_const(Vs, 1 / nVs);

//   double L = dot(vec, eminus(N1, P1));

//   std::vector<double> temp1 = mult_const(vec, L);
//   std::vector<double> temp2 = eminus(P1, N1);
//   std::vector<double> temp3 = eplus(temp1, temp2);

//   return norm(temp3);
// }

// double point2point_dist(std::vector<double> P1, geometry_msgs::PoseStamped P2){
//   // 벡터 형식 점 P1과 mavros 위치 메시지형식의 점 P2 사이의 거리 계산
//   std::vector<double> temp1 {0,0}, temp2;
//   double dist;
//   temp1[0]=P1[0]-P2.pose.position.x;
//   temp1[1]=P1[1]-P2.pose.position.y;
//   dist = norm(temp1);
//   return dist;
// }
