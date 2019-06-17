#include <vector>
#include <list>
#include <cmath>
#include <iostream>
using namespace std;

#define N 3

// dis表示从起点到goals的最小距离
double dis[8][N+1];

int getpow2(int n)
{
  int p = 1;
  while (n > 0)
  {
    p = p << 1;
    n--;
  }
  return p;
}

// 计算goals中点的个数
bool count1(int s)
{
  int count = 0;
  while (s > 0)
  {
    s = s & (s-1);
    count++;
  }
  return count == 1;
}

// goals中只有一个点时，得到那个点的位置
int locate(int s)
{
  int loc = 0;
  while (s > 0)
  {
    if (s & 1) break;
    s = s >> 1;
    loc++;
  }
  return loc;
}


// current 表示当前起点，初始点为N
// goals用一个数表示剩余的点数，二进制1表示有, goals < 1024，最多10个点
int getShortestPath(int current, int goals, double costs[N][N])
{
  // 如果已经计算了距离，直接返回
  if (dis[goals][current] != 0) return dis[goals][current];
  // 如果只剩一个点，返回当前点到其距离
  if (count1(goals)) 
  {
    return dis[goals][current] = costs[current][locate(goals)];
  }
  int mincost = 100000;
 
  // 遍历所有的点，从1开始到N-1
  for (int i=0;i<N;i++)
  {
    // 在goals中
    if (goals&(1<<i))
	  {
	    // 递归计算最短距离
	    double nextcost = getShortestPath(i, goals&(~(1<<i)), costs);
	    // 初始点特殊处理
      double curcost = current == N ? costs[i][i] : costs[current][i];
      if (nextcost + curcost < mincost)
	    {
	      mincost = nextcost + curcost;
	    }
	  }
  }
  return dis[goals][current] = mincost;
}

vector<int> getPath(int s, int cost, double costs[N][N])
{
  int current = N;
  vector<int> path;
  for (int i=0; i < N; i++)
  {
    for (int j = 0;j < N; j++)
    {
      if (s&(1<<j))
      {
        int curcost;
        if (current == N)
          curcost = costs[j][j];
        else
          curcost = costs[current][j];
        if (curcost + dis[s&(~(1<<j))][j] == cost)
        {
          cost = cost - curcost;
          s = s&(~(1<<j));
          current = j;
          path.push_back(current);
          break;
        }
      }
    }
  }
  return path;
}

int main(int argc, char** argv)
{
	double costs[N][N];
	int s = getpow2(N)-1;
	for (int i =0;i<N;i++)
	{
		costs[i][i] = i + 1;
		for (int j=i+1;j<N;j++)
		{
			costs[i][j] = 5 - j;
			costs[j][i] = costs[i][j];
		}
	}
	for (int i=0;i<N;i++)
	{
		cout << costs[i][0] << costs[i][1] << costs[i][2] << endl;
	}
	int cost = getShortestPath(N, s, costs);
	cout << cost << endl;
	vector<int> path = getPath(s, cost, costs);
	for (auto p : path)
	  cout << p << endl;
	return 0;
}

