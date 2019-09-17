#ifndef KDTREE_H
#define KDTREE_H

#include "object.h"

namespace Raytracer{

class KdTreeNode
{
private:
	int _axis;					//切分轴向
	double _split;				//切分位置
	KdTreeNode* _left;			//左孩子	
	KdTreeNode* _right;			//右孩子
	ListNode* _obList;
public:
	KdTreeNode(): _left(0), _right(0), _obList(0) {}
	void setAxis( int a_Axis ) { _axis = a_Axis;  }
	int getAxis() { return _axis; }
	void setSplitPos( double a_Pos ) { _split = a_Pos; }
	double getSplitPos() { return _split; }
	void setLeft( KdTreeNode* a_Left ) { _left = a_Left; }
	void setRight( KdTreeNode* a_Right ) { _right = a_Right; }
	KdTreeNode* getLeft() { return _left; }
	KdTreeNode* getRight() { return _right; }
	bool isLeaf() { return (_left == 0); }
	ListNode* getList() { return _obList; }
	void setList( ListNode* a_List ) {  _obList = a_List; }
};

class Scene;

class KdTree
{
private:
	KdTreeNode* _root;			//根节点
	KdTreeNode* _KdTreeNodeBuffer;   //一次开够节点内存
	int _buffer_k;
	ListNode* _ListNodeBuffer;
	int _buffer_l;
	int _buffer_s;
public:
	KdTree() 
	{
		_buffer_k = 0;
		_KdTreeNodeBuffer = new KdTreeNode[2000000];
		_buffer_l = 0;
		_ListNodeBuffer = new ListNode[10000000];
	}
 	~KdTree()
	{
		delete[] _KdTreeNodeBuffer;
		delete[] _ListNodeBuffer;
		//delete[] _splitPool;
	}
	void build( Scene* a_Scene );
	KdTreeNode* getRoot() { return _root; }
	void setRoot( KdTreeNode* a_Root ) { _root = a_Root; }
	void insertSplitPos( double a_SplitPos );
	void subdivide( KdTreeNode* a_Node, Box& a_Box, int a_Depth, int obs );
	void add(KdTreeNode* node, Object* ob);
};

struct KdStack
{
	KdTreeNode* node;
	double t;
	Vec3 pb;
	int prev;
};

};
#endif