#pragma once
#include "Mesh.h"
#include <set>
#include <vector>
#include <map>

//simplified spring for gpu
struct s_spring      
{
	int end;  //一端的点的索引
	float original;   //弹簧原长
};

class Springs
{
public:
	~Springs();
	Springs(Mesh* spring_obj);  //创建两级邻域，boundary and cloth弹簧劲度系数一致
	void draw(Mesh* spring_obj);
	void CSR_structure_spring(Mesh* spring_obj, vector<unsigned int>& CSR_R, vector<s_spring>& CSR_C);
	void CSR_bend_spring(Mesh* spring_obj, vector<unsigned int>& CSR_R, vector<s_spring>& CSR_C);

private:
	vector<pair<unsigned int,unsigned int>> cloth_boundary_springs;   //只包含pair(1,2)
	vector<pair<unsigned int,unsigned int>> boundary_boundary_springs;   //应该已经包含pair(1,2) && pair(2,1)
	set<pair<unsigned int,unsigned int>> boundary;
	vector<vector<unsigned int>> neigh1;   //存储每个点的所有一级邻域信息(存储点的索引),即 structure spring
	vector<vector<unsigned int>> neigh2;   //存储每个点的所有二级邻域信息(存储点的索引),即 bend spring
	map<pair<unsigned int, unsigned int>,float> bend_spring_length;  //存储两个共边三角形对角顶点索引+共面后的距离

private:
	void create_neigh(Mesh* spring_obj);
	bool exist(const vector<unsigned int>& array, const unsigned int val);
	void get_cloth_boundary_spring(Mesh* spring_obj);
	void get_boundary_boundary_spring(Mesh* spring_obj);
	void CSR_spring(const Mesh* spring_obj, const vector<vector<unsigned int>>& neigh, vector<unsigned int>& CSR_R, vector<s_spring>& CSR_C);
};