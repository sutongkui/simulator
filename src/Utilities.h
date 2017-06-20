#pragma once
#include <vector>

void getFPS();


template<class Key>
class SortAssistant
{
public:
	SortAssistant(const std::vector<Key> &keys) : _keys(keys) { }

	bool operator()(const unsigned int &lhs, const unsigned int &rhs)
	{
		return _keys[lhs] < _keys[rhs];
	}
private:
	const std::vector<Key> &_keys;
};

// 容器必须是vector等支持随机访问的容器
template<class Containers>
void indices_sort(const Containers &keys, std::vector<unsigned int> &indices)
{
	indices.resize(keys.size());
	for (int i = 0; i < keys.size(); ++i)
	{
		indices[i] = i;
	}
	std::sort(indices.begin(), indices.end(), SortAssistant<typename Containers::value_type>(keys));

}

// 为支持更通用的情况，由用户确保indices的索引不超过keys的大小
template<class Containers>
void remove_redundant(const Containers &keys, std::vector<unsigned int> &indices)
{
	if (indices.empty())
	{
		return;
	}

	unsigned int n = 1;
	typename Containers::value_type current = keys[indices[0]];
	for (int i = 1; i < indices.size(); ++i)
	{
		if (keys[indices[i]] != current)
		{
			indices[n++] = indices[i];
			current = keys[indices[i]];
		}
	}

	indices.resize(n);
}

// 为支持更通用的情况，由用户确保indices的索引不超过keys的大小
template<class Containers>
void filter(const Containers &input, const std::vector<unsigned int> &indices, Containers &ouput)
{
	ouput.resize(indices.size());
	for (int i = 0; i < indices.size(); ++i)
	{
		unsigned int index = indices[i];
		ouput[i] = input[indices[i]];
	}
}
