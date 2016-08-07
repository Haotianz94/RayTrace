#include "KdTree.h"
#include<fstream>
#include<algorithm>

namespace Raytracer{

void KdTree::build( Scene* a_Scene )
{
	_root = &_KdTreeNodeBuffer[_buffer_k++];
	for ( int p = 0; p < a_Scene->getObjects(); p++ )
		add(_root, a_Scene->getObject( p ) );
	int obs = a_Scene->getObjects();
	Box sbox = a_Scene->getExtend();
	subdivide( _root, sbox, 0, obs );					//�Ը��ڵ����仮��

	std::cout<<"KdTreeNode:  "<<_buffer_k<<std::endl;
	std::cout<<"ListNode:   "<<_buffer_l<<std::endl;
}

void KdTree::subdivide( KdTreeNode* a_Node, Box& a_Box, int depth, int obs )
{
	//���������������Ϊ�и�����
	Vec3 s = a_Box.getSize();
	if ((s.x >= s.y) && (s.x >= s.z))
		a_Node->setAxis( 0 );
	else if ((s.y >= s.x) && (s.y >= s.z))
		a_Node->setAxis( 1 );
	else
		a_Node->setAxis( 2 );
	int axis = a_Node->getAxis();

	double* splitPos_ = new double[obs*3+100];
	double* splitPos = new double[obs*3+100];
	int split = 0;
	int split_ = 0; 

	double* llist = new double[obs];
	double* rlist = new double[obs];
	ListNode* list = a_Node->getList();
	for( int i = 0; i < obs; i++)
	{
		llist[i] = list->getObject()->getMin(axis);
		rlist[i] = list->getObject()->getMax(axis);
		Object* ob = list->getObject();
		if( ob->getType() == Object::TRIANGLE)
		{
			for(int j = 0;j < 3;j++)
				splitPos_[split_++] = ( (Triangle*)ob )->getVertex(j)->getPos().cell[axis];//  �ָ��߿��ܳ������
		}
		else
		{
			splitPos_[split_++] = ob->getMin(axis);
			splitPos_[split_++] = ob->getMax(axis);
		}
		list = list->getNext();
	}

	std::sort(llist, llist+obs);
	std::sort(rlist, rlist+obs);
	std::sort(splitPos_, splitPos_ + split_);
	double lastsplit = -10000;
	for(int i = 0; i < split_; i++)
	{
		if(lastsplit != splitPos_[i])
		{
			splitPos[split++] = splitPos_[i];
			lastsplit = splitPos_[i];
		}
	}

	// ��ǰ�ڵ����������ĵ���
	double SAV = 0.5 / (a_Box.w() * a_Box.d() + a_Box.w() * a_Box.h() + a_Box.d() * a_Box.h());
	// ��ǰ�ڵ��cost
	double Cleaf = obs * 1.0;

	double lowcost = 10000000;
	double bestpos = 0;
	int n1, n2;
	int L = 0, R = 0;

	double pos1 = a_Box.getPos().cell[axis];
	double pos2 = a_Box.getSize().cell[axis] + a_Box.getPos().cell[axis];
	Box b1, b2, b3 = a_Box, b4 = a_Box;
	for( int i = 0; i < split; i++)
	{
		n1 = 0;
		n2 = 0;
		while(llist[L] <= splitPos[i] && L < obs) L++;
		while(rlist[R] < splitPos[i] && R < obs) R++;
		if( L ==0 )
			continue;
		else
			n1 = L;
		if( R > obs)
			continue;
		else
			n2 = obs - R; 

			// b3Ϊ��У�b4Ϊ�Һ�
			b4.getPos().cell[axis] = splitPos[i];
			b4.getSize().cell[axis] = pos2 - splitPos[i];
			b3.getSize().cell[axis] = splitPos[i] - pos1;
			// ����ָ���cost
			double SA1 = 2 * (b3.w() * b3.d() + b3.w() * b3.h() + b3.d() * b3.h());		//����ӱ����
			double SA2 = 2 * (b4.w() * b4.d() + b4.w() * b4.h() + b4.d() * b4.h());		//�Һ��ӱ����
			double splitcost = 0.3 + 1.0 * (SA1 * SAV * n1 + SA2 * SAV * n2 );
			// �������cost
			if (splitcost < lowcost)
			{
				lowcost = splitcost;
				bestpos = splitPos[i];
				b1 = b3, b2 = b4;
			}
	}	
	if (lowcost > Cleaf)	//���ָ����ָ�ǰ��costС��������ָ�
		return;

	a_Node->setSplitPos( bestpos );
	// �������Һ��ӽڵ�
	KdTreeNode* left = &_KdTreeNodeBuffer[_buffer_k++];
	KdTreeNode* right = &_KdTreeNodeBuffer[_buffer_k++];
	// ���������������
	double b1p1 = b1.getPos().cell[axis];
	double b2p2 = b2.getPos().cell[axis] + b2.getSize().cell[axis];
	double b1p2 = b1.getPos().cell[axis] + b1.getSize().cell[axis];
	double b2p1 = b2.getPos().cell[axis];
	double eleft, eright;
	int n1count = 0, n2count =0;
	list = a_Node->getList();
	while(list)
	{
		Object* p = list->getObject();
		eleft = p->getMin(axis);
		eright = p->getMax(axis);
		if( eleft <= bestpos)
		//if ((eleft <= b1p2) && (eright >= b1p1))
			//if (p->intersectBox( b1 )) 
			{
				add(left, p);
				n1count++;
			}
		if( eright >= bestpos )
		//if ((eleft <= b2p2) && (eright >= b2p1))
			//if (p->intersectBox( b2 )) 
			{
				add(right, p );
				n2count++;
			}
		list = list->getNext();
	}
	
	delete[] llist;
	delete[] rlist;
	delete[] splitPos;
	delete[] splitPos_;
	a_Node->setLeft( left );
	a_Node->setRight( right );
	if(_buffer_k > 1900000 || _buffer_l > 9900000)
		return ;
	if (depth < MAXTREEDEPTH)
	{
		if (n1count > 2) subdivide( left, b1, depth + 1, n1count );
		if (n2count > 2) subdivide( right, b2, depth + 1, n2count );
	}
}

void KdTree::add( KdTreeNode* kdnode, Object* ob )
{
	//std::cout<<_buffer_l<<std::endl;
	ListNode* node = &_ListNodeBuffer[_buffer_l++];
	node->setObject( ob );
	node->setNext( kdnode->getList() );
	kdnode->setList( node );
}

};