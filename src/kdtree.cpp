
//头文件  

#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <math.h>  
#include "kdtree.h"  

#if defined(WIN32) || defined(__WIN32__)  
#include <malloc.h>  
#endif  
  
#ifdef USE_LIST_NODE_ALLOCATOR  
  
#ifndef NO_PTHREADS  
#include <pthread.h>  
#else  
  
#ifndef I_WANT_THREA BUGS  
#error "You are compiling with the fast list node allocator, with pthreads disabled! This WILL break if used from multiple threads."  
#endif  /* I want thread bugs */  
  
#endif  /* pthread support */  
#endif  /* use list node allocator */  
  
  
//超平面的结构体  
//包括一个属性的维数和每维坐标的最大和最小值构成的数组  
struct kdhyperrect {  
    int dim;  
    double *min, *max;              /* minimum/maximum coords */  
};  
  
//节点的结构体，也就是事例的结构体  
struct kdnode {  
    double *pos;  
    int dir;  
    void *data;  
  
    struct kdnode *left, *right;    /* negative/positive side */  
};  
  
//返回结果节点， 包括树的节点,距离值, 是一个单链表的形式  
struct res_node {  
    struct kdnode *item;  
    double dist_sq;  
    struct res_node *next;  
};  
  
//树有几个属性，一是维数，一是树根节点，一是超平面，一是销毁data的函数  
struct kdtree {  
    int dim;  
    struct kdnode *root;  
    struct kdhyperrect *rect;  
    void (*destr)(void*);  
};  
  
//kdtree的返回结果，包括kdtree，这是一个双链表的形式  
struct kdres {  
    struct kdtree *tree;  
    struct res_node *rlist, *riter;  //双链表?  
    int size;  
};  
  
//计算平方的宏定义,相当于函数  
#define SQ(x)           ((x) * (x))  
  
  
static void clear_rec(struct kdnode *node, void (*destr)(void*));  
static int insert_rec(struct kdnode **node, const double *pos, void *data, int dir, int dim);  
static int rlist_insert(struct res_node *list, struct kdnode *item, double dist_sq);  
static void clear_results(struct kdres *set);  
  
static struct kdhyperrect* hyperrect_create(int dim, const double *min, const double *max);  
static void hyperrect_free(struct kdhyperrect *d_rect);
static struct kdhyperrect* hyperrect_duplicate(const struct kdhyperrect *rect);  
static void hyperrect_extend(struct kdhyperrect *rect, const double *pos);  
static double hyperrect_dist_sq(struct kdhyperrect *rect, const double *pos);  
  
#ifdef USE_LIST_NODE_ALLOCATOR  
static struct res_node *alloc_resnode(void);  
static void free_resnode(struct res_node*);  
#else  
#define alloc_resnode()     malloc(sizeof(struct res_node))  
#define free_resnode(n)     free(n)  
#endif  
  
  
//创建一个kdtree  
struct kdtree *kd_create(int k)
{  
    struct kdtree *tree;  
  
    if(!(tree = (kdtree*)malloc(sizeof *tree))) {  
        return 0;  
    }  
  
    tree->dim = k;  
    tree->root = 0;  
    tree->destr = 0;  
    tree->rect = 0;  
  
    return tree;  
}  
  
//释放掉kdtree  
void kd_free(struct kdtree *tree)
{  
    if(tree) {  
        kd_clear(tree);
        free(tree);  
    }  
}  
  
//清除掉超平面,是按节点递归地进行的  
static void clear_rec(struct kdnode *node, void (*destr)(void*))  
{  
    if(!node) return;   //一个节点对应一个超平面  
  
    //递归函数，递归地清除掉二叉树左分支的超平面和二叉树右分支的超平面  
    clear_rec(node->left, destr);  
    clear_rec(node->right, destr);  
      
    //如果data清楚函数不为空,就释放掉data  
    if(destr)   
    {  
        destr(node->data);  
    }  
    //释放节点的坐标数组  
    free(node->pos);  
    //释放节点  
    free(node);  
}  
  
//kdtree清除  
void kd_clear(struct kdtree *tree)
{  
    //清除树中每个节点的超平面,释放树中的各个节点  
    clear_rec(tree->root, tree->destr);  
    tree->root = 0;  
  
    //如果树的超平面指针不为空,对其进行释放  
    if (tree->rect)   
    {  
        hyperrect_free(tree->rect);  
        tree->rect = 0;  
    }  
}  
  
//数据销毁，用一个外来的函数来进行data的销毁  
void kd_data_destructor(struct kdtree *tree, void (*destr)(void*))
{  
    //用外来的函数来执行kdtree的销毁函数  
    tree->destr = destr;  
}  
  
  
//在一个树节点位置处插入超矩形  
static int insert_rec(struct kdnode **nptr, const double *pos, void *data, int dir, int dim)  
{  
    int new_dir;  
    struct kdnode *node;  
  
    //如果这个节点是不存在的  
    if(!*nptr)   
    {  
        //分配一个结点  
        if(!(node = (kdnode *)malloc(sizeof *node)))   
        {  
            return -1;  
        }  
        if(!(node->pos = (double*)malloc(dim * sizeof *node->pos))) {  
            free(node);  
            return -1;  
        }  
        memcpy(node->pos, pos, dim * sizeof *node->pos);  
        node->data = data;  
        node->dir = dir;  
        node->left = node->right = 0;  
        *nptr = node;  
        return 0;  
    }  
  
    node = *nptr;  
    new_dir = (node->dir + 1) % dim;  
    if(pos[node->dir] < node->pos[node->dir]) {  
        return insert_rec(&(*nptr)->left, pos, data, new_dir, dim);  
    }  
    return insert_rec(&(*nptr)->right, pos, data, new_dir, dim);  
}  
  
//节点插入操作  
//参数为:要进行插入操作的kdtree,要插入的节点坐标,要插入的节点的数据  
int kd_insert(struct kdtree *tree, const double *pos, void *data)
{  
    //插入超矩形  
    if (insert_rec(&tree->root, pos, data, 0, tree->dim))   
    {  
        return -1;  
    }  
    //如果树还没有超矩形,就创建一个超矩形  
    //如果已经有了超矩形,就扩展原有的超矩形  
    if (tree->rect == 0)   
    {  
        tree->rect = hyperrect_create(tree->dim, pos, pos);  
    }   
    else   
    {  
        hyperrect_extend(tree->rect, pos);  
    }  
  
    return 0;  
}  
  
//插入float型坐标的节点  
//参数为:要进行插入操作的kdtree,要插入的节点坐标,要插入的节点的数据  
//将float型的坐标赋值给double型的缓冲区,经过这个类型转化后进行插入  
//本质上是一种类型转化  
int kd_insertf(struct kdtree *tree, const float *pos, void *data)
{  
    static double sbuf[16];  
    double *bptr, *buf = 0;  
    int res, dim = tree->dim;  
  
    //如果kdtree的维数大于16, 分配dim维double类型的数组  
    if(dim > 16)   
    {  
#ifndef NO_ALLOCA  
        if(dim <= 256)  
            bptr = buf = (double*)alloca(dim * sizeof *bptr);  
        else  
#endif  
            if(!(bptr = buf = (double*)malloc(dim * sizeof *bptr)))   
            {  
                return -1;  
            }  
    }   
    //如果kdtree的维数小于16, 直接将指针指向已分配的内存  
    else   
    {  
        bptr = buf = sbuf;  
    }  
  
    //将要插入点的位置坐标赋值给分配的数组  
    while(dim-- > 0)   
    {  
        *bptr++ = *pos++;  
    }  
  
    //调用节点插入函数k insert  
    res = kd_insert(tree, buf, data);
#ifndef NO_ALLOCA  
    if(tree->dim > 256)  
#else  
    if(tree->dim > 16)  
#endif  
        //释放缓存  
        free(buf);  
    return res;  
}  
  
//给出三维坐标值的三维kdtree插入  
int kd_insert3(struct kdtree *tree, double x, double y, double z, void *data)
{  
    double buf[3];  
    buf[0] = x;  
    buf[1] = y;  
    buf[2] = z;  
    return kd_insert(tree, buf, data);
}  
  
//给出三维float型坐标值的三维kdtree插入  
int kd_insert3f(struct kdtree *tree, float x, float y, float z, void *data)
{  
    double buf[3];  
    buf[0] = x;  
    buf[1] = y;  
    buf[2] = z;  
    return kd_insert(tree, buf, data);
}  
  
//找到最近邻的点  
//参数为:树节点指针, 位置坐标, 阈值, 返回结果的节点, bool型排序,维度  
static int find_nearest(struct kdnode *node, const double *pos, double range, struct res_node *list, int ordered, int dim)
{  
    double dist_sq, dx;  
    int i, ret, added_res = 0;
  
    if(!node) return 0;  //注意这个地方,当节点为空的时候,表明已经查找到最终的叶子结点,返回值为零  
  
    dist_sq = 0;  
    //计算两个节点间的平方和  
    for(i=0; i<dim; i++)   
    {  
        dist_sq += SQ(node->pos[i] - pos[i]);  
    }  
    //如果距离在阈值范围内,就将其插入到返回结果链表中  
    if(dist_sq <= SQ(range))   
    {         
        if(rlist_insert(list, node, ordered ? dist_sq : -1.0) == -1)   
        {  
            return -1;  
        }  
        added_res = 1;
    }  
  
    //在这个节点的划分方向上,求两者之间的差值  
    dx = pos[node->dir] - node->pos[node->dir];  
  
    //根据这个差值的符号, 选择进行递归查找的分支方向  
    ret = find_nearest(dx <= 0.0 ? node->left : node->right, pos, range, list, ordered, dim);
    //如果返回的值大于等于零,表明在这个分支中有满足条件的节点,则返回结果的个数进行累加,并在节点的另一个方向进行查找最近的节点  
    if(ret >= 0 && fabs(dx) < range)   
    {  
        added_res += ret;
        ret = find_nearest(dx <= 0.0 ? node->right : node->left, pos, range, list, ordered, dim);
    }  
    if(ret == -1)   
    {  
        return -1;  
    }  
    added_res += ret;
  
    return added_res;
}  
  
  
//找到最近邻的n个节点  
#if 0  
static int fin nearest_n(struct kdnode *node, const double *pos, double range, int num, struct rheap *heap, int dim)  
{  
    double dist_sq, dx;  
    int i, ret, adde res = 0;  
  
    if(!node) return 0;  
      
    /* if the photon is close enough, add it to the result heap */  
    //如果足够近就将其加入到结果堆中  
    dist_sq = 0;  
    //计算两者间的欧式距离  
    for(i=0; i<dim; i++)   
    {  
        dist_sq += SQ(node->pos[i] - pos[i]);  
    }  
    //如果计算所得距离小于阈值  
    if(dist_sq <= range_sq) {  
    //如果堆的大小大于num,也就是大于总的要找的节点数  
        if(heap->size >= num)  
        {  
            /* get furthest element */  
            //得到最远的节点  
            struct res_node *maxelem = rheap_get_max(heap);  
  
            /* and check if the new one is closer than that */  
            //测试这个节点是不是比最远的节点要近  
            if(maxelem->dist_sq > dist_sq)   
            {  
            //如果是的话,就移除最远的节点  
                rheap_remove_max(heap);  
                //并将此节点插入堆中  
                if(rheap_insert(heap, node, dist_sq) == -1)   
                {  
                    return -1;  
                }  
                adde res = 1;  
  
                range_sq = dist_sq;  
            }  
        }   
        //如果堆的大小小于num,直接将此节点插入堆中  
        else   
        {  
            if(rheap_insert(heap, node, dist_sq) == -1)   
            {  
                return =1;  
            }  
            adde res = 1;  
        }  
    }  
  
  
    /* find signed distance from the splitting plane */  
    dx = pos[node->dir] - node->pos[node->dir];  
  
    ret = fin nearest_n(dx <= 0.0 ? node->left : node->right, pos, range, num, heap, dim);  
    if(ret >= 0 && fabs(dx) < range) {  
        adde res += ret;  
        ret = fin nearest_n(dx <= 0.0 ? node->right : node->left, pos, range, num, heap, dim);  
    }  
}  
#endif  
  
  
static void kd_nearest_i(struct kdnode *node, const double *pos, struct kdnode **result, double *result_dist_sq, struct kdhyperrect* rect)
{  
    int dir = node->dir;  
    int i;  
    double dummy, dist_sq;  
    struct kdnode *nearer_subtree, *farther_subtree;  
    double *nearer_hyperrect_coord, *farther_hyperrect_coord;  
  
    /* Decide whether to go left or right in the tree */  
    //在二叉树中,决定向左走还是向右走  
    dummy = pos[dir] - node->pos[dir];  
    if (dummy <= 0)   
    {  
        nearer_subtree = node->left;  
        farther_subtree = node->right;  
        nearer_hyperrect_coord = rect->max + dir;  
        farther_hyperrect_coord = rect->min + dir;  
    }   
    else   
    {  
        nearer_subtree = node->right;  
        farther_subtree = node->left;  
        nearer_hyperrect_coord = rect->min + dir;  
        farther_hyperrect_coord = rect->max + dir;  
    }  
  
    if (nearer_subtree) {  
        /* Slice the hyperrect to get the hyperrect of the nearer subtree */  
        dummy = *nearer_hyperrect_coord;  
        *nearer_hyperrect_coord = node->pos[dir];  
        /* Recurse down into nearer subtree */  
        kd_nearest_i(nearer_subtree, pos, result, result_dist_sq, rect);
        /* Undo the slice */  
        *nearer_hyperrect_coord = dummy;  
    }  
  
    /* Check the distance of the point at the current node, compare it 
     * with our best so far */  
    dist_sq = 0;  
    for(i=0; i < rect->dim; i++)   
    {  
        dist_sq += SQ(node->pos[i] - pos[i]);  
    }  
    if (dist_sq < *result_dist_sq)   
    {  
        *result = node;  
        *result_dist_sq = dist_sq;  
    }  
  
    if (farther_subtree) {  
        /* Get the hyperrect of the farther subtree */  
        dummy = *farther_hyperrect_coord;  
        *farther_hyperrect_coord = node->pos[dir];  
        /* Check if we have to recurse down by calculating the closest 
         * point of the hyperrect and see if it's closer than our 
         * minimum distance in result_dist_sq. */  
        if (hyperrect_dist_sq(rect, pos) < *result_dist_sq) {  
            /* Recurse down into farther subtree */  
            kd_nearest_i(farther_subtree, pos, result, result_dist_sq, rect);
        }  
        /* Undo the slice on the hyperrect */  
        *farther_hyperrect_coord = dummy;  
    }  
}  
  
//求kdtree中与点pos最近邻的值  
struct kdres *kd_nearest(struct kdtree *kd, const double *pos)
{  
    struct kdhyperrect *rect;  
    struct kdnode *result;  
    struct kdres *rset;  
    double dist_sq;  
    int i;  
  
    //如果kd不存在,或者其超平面不存在的话,则就不会有结果  
    if (!kd) return 0;  
    if (!kd->rect) return 0;  
  
    /* Allocate result set */  
    //为返回结果集合分配空间  
    if(!(rset = (kdres*)malloc(sizeof *rset)))   
    {  
        return 0;  
    }  
    if(!(rset->rlist = (res_node*)alloc_resnode())) {  
        free(rset);  
        return 0;  
    }  
    rset->rlist->next = 0;  
    rset->tree = kd;  
  
    /* Duplicate the bounding hyperrectangle, we will work on the copy */  
    //复制边界超平面  
    if (!(rect = hyperrect_duplicate(kd->rect)))   
    {  
        kd_res_free(rset);
        return 0;  
    }  
  
    /* Our first guesstimate is the root node */  
    result = kd->root;  
    dist_sq = 0;  
    for (i = 0; i < kd->dim; i++)  
        dist_sq += SQ(result->pos[i] - pos[i]);  
  
    /* Search for the nearest neighbour recursively */  
    //递归地查找最近邻的邻居  
    kd_nearest_i(kd->root, pos, &result, &dist_sq, rect);
  
    /* Free the copy of the hyperrect */  
    //释放超矩形  
    hyperrect_free(rect);  
  
    /* Store the result */  
    //存储结果  
    if (result)   
    {  
        if (rlist_insert(rset->rlist, result, -1.0) == -1)   
        {  
            kd_res_free(rset);
            return 0;  
        }  
        rset->size = 1;  
        kd_res_rewind(rset);
        return rset;  
    }   
    else   
    {  
        kd_res_free(rset);
        return 0;  
    }  
}  
  
//k nearest的float特例  
struct kdres *kd_nearestf(struct kdtree *tree, const float *pos)
{  
    static double sbuf[16];  
    double *bptr, *buf = 0;  
    int dim = tree->dim;  
    struct kdres *res;  
  
    if(dim > 16) {  
#ifndef NO_ALLOCA  
        if(dim <= 256)  
            bptr = buf = (double*)alloca(dim * sizeof *bptr);  
        else  
#endif  
            if(!(bptr = buf = (double*)malloc(dim * sizeof *bptr))) {  
                return 0;  
            }  
    } else {  
        bptr = buf = sbuf;  
    }  
  
    while(dim-- > 0) {  
        *bptr++ = *pos++;  
    }  
  
    res = kd_nearest(tree, buf);
#ifndef NO_ALLOCA  
    if(tree->dim > 256)  
#else  
    if(tree->dim > 16)  
#endif  
        free(buf);  
    return res;  
}  
  
//k nearest的三坐标特例  
struct kdres *kd_nearest3(struct kdtree *tree, double x, double y, double z)
{  
    double pos[3];  
    pos[0] = x;  
    pos[1] = y;  
    pos[2] = z;  
    return kd_nearest(tree, pos);
}  
  
//k nearest的三坐标float特例  
struct kdres *kd_nearest3f(struct kdtree *tree, float x, float y, float z)
{  
    double pos[3];  
    pos[0] = x;  
    pos[1] = y;  
    pos[2] = z;  
    return kd_nearest(tree, pos);
}  
  
/* ---- nearest N search ---- */  
/* 
static kdres *k nearest_n(struct kdtree *kd, const double *pos, int num) 
{ 
    int ret; 
    struct kdres *rset; 
 
    if(!(rset = malloc(sizeof *rset))) { 
        return 0; 
    } 
    if(!(rset->rlist = alloc_resnode())) { 
        free(rset); 
        return 0; 
    } 
    rset->rlist->next = 0; 
    rset->tree = kd; 
 
    if((ret = fin nearest_n(kd->root, pos, range, num, rset->rlist, kd->dim)) == -1) { 
        k res_free(rset); 
        return 0; 
    } 
    rset->size = ret; 
    k res_rewind(rset); 
    return rset; 
}*/  
  
//找到满足距离小于range值的节点  
struct kdres *kd_nearest_range(struct kdtree *kd, const double *pos, double range)
{  
    int ret;  
    struct kdres *rset;  
  
    if(!(rset = (kdres*)malloc(sizeof *rset))) {  
        return 0;  
    }  
    if(!(rset->rlist = (res_node*)alloc_resnode())) {  
        free(rset);  
        return 0;  
    }  
    rset->rlist->next = 0;  
    rset->tree = kd;  
  
    if((ret = find_nearest(kd->root, pos, range, rset->rlist, 0, kd->dim)) == -1) {
        kd_res_free(rset);
        return 0;  
    }  
    rset->size = ret;  
    kd_res_rewind(rset);
    return rset;  
}  
  
//k nearest_range的float特例  
struct kdres *kd_nearest_rangef(struct kdtree *kd, const float *pos, float range)
{  
    static double sbuf[16];  
    double *bptr, *buf = 0;  
    int dim = kd->dim;  
    struct kdres *res;  
  
    if(dim > 16) {  
#ifndef NO_ALLOCA  
        if(dim <= 256)  
            bptr = buf = (double*)alloca(dim * sizeof *bptr);  
        else  
#endif  
            if(!(bptr = buf = (double*)malloc(dim * sizeof *bptr))) {  
                return 0;  
            }  
    } else {  
        bptr = buf = sbuf;  
    }  
  
    while(dim-- > 0) {  
        *bptr++ = *pos++;  
    }  
  
    res = kd_nearest_range(kd, buf, range);
#ifndef NO_ALLOCA  
    if(kd->dim > 256)  
#else  
    if(kd->dim > 16)  
#endif  
        free(buf);  
    return res;  
}  
  
//k nearest_range的三坐标特例  
struct kdres *kd_nearest_range3(struct kdtree *tree, double x, double y, double z, double range)
{  
    double buf[3];  
    buf[0] = x;  
    buf[1] = y;  
    buf[2] = z;  
    return kd_nearest_range(tree, buf, range);
}  
  
//k nearest_range的三坐标float特例  
struct kdres *kd_nearest_range3f(struct kdtree *tree, float x, float y, float z, float range)
{  
    double buf[3];  
    buf[0] = x;  
    buf[1] = y;  
    buf[2] = z;  
    return kd_nearest_range(tree, buf, range);
}  
  
//返回结果的释放  
void kd_res_free(struct kdres *rset)
{  
    clear_results(rset);  
    free_resnode(rset->rlist);  
    free(rset);  
}  
  
//获取返回结果集合的大小  
int kd_res_size(struct kdres *set)
{  
    return (set->size);  
}  
  
//再次回到这个节点本身的位置  
void kd_res_rewind(struct kdres *rset)
{  
    rset->riter = rset->rlist->next;  
}  
  
//找到返回结果中的最终节点  
int kd_res_end(struct kdres *rset)
{  
    return rset->riter == 0;  
}  
  
//返回结果列表中的下一个节点  
int kd_res_next(struct kdres *rset)
{  
    rset->riter = rset->riter->next;  
    return rset->riter != 0;  
}  
  
//将返回结果的节点的坐标和data抽取出来  
void *kd_res_item(struct kdres *rset, double *pos)
{  
    if(rset->riter) {  
        if(pos) {  
            memcpy(pos, rset->riter->item->pos, rset->tree->dim * sizeof *pos);  
        }  
        return rset->riter->item->data;  
    }  
    return 0;  
}  
  
//将返回结果的节点的坐标和data抽取出来,坐标为float型的值  
void *kd_res_itemf(struct kdres *rset, float *pos)
{  
    if(rset->riter) {  
        if(pos) {  
            int i;  
            for(i=0; i<rset->tree->dim; i++) {  
                pos[i] = rset->riter->item->pos[i];  
            }  
        }  
        return rset->riter->item->data;  
    }  
    return 0;  
}  
  
//将返回结果的节点的坐标和data抽取出来,坐标具体形式给出  
void *kd_res_item3(struct kdres *rset, double *x, double *y, double *z)
{  
    if(rset->riter) {  
        if(*x) *x = rset->riter->item->pos[0];  
        if(*y) *y = rset->riter->item->pos[1];  
        if(*z) *z = rset->riter->item->pos[2];  
    }  
    return 0;  
}  
  
//将返回结果的节点的坐标和data抽取出来,坐标为float型的值,坐标具体形式给出  
void *kd_res_item3f(struct kdres *rset, float *x, float *y, float *z)
{  
    if(rset->riter) {  
        if(*x) *x = rset->riter->item->pos[0];  
        if(*y) *y = rset->riter->item->pos[1];  
        if(*z) *z = rset->riter->item->pos[2];  
    }  
    return 0;  
}  
  
//获取data数据  
void *kd_res_item_data(struct kdres *set)
{  
    return kd_res_item(set, 0);
}  
  
/* ---- hyperrectangle helpers ---- */  
//创建超平面,包括三个参数:维度,每维的最小值和最大值数组  
static struct kdhyperrect* hyperrect_create(int dim, const double *min, const double *max)  
{  
    size_t size = dim * sizeof(double);  
    struct kdhyperrect* rect = 0;  
  
    if (!(rect = (kdhyperrect*)malloc(sizeof(struct kdhyperrect))))   
    {  
        return 0;  
    }  
  
    rect->dim = dim;  
    if (!(rect->min = (double*)malloc(size))) {  
        free(rect);  
        return 0;  
    }  
    if (!(rect->max = (double*)malloc(size))) {  
        free(rect->min);  
        free(rect);  
        return 0;  
    }  
    memcpy(rect->min, min, size);  
    memcpy(rect->max, max, size);  
  
    return rect;  
}  
  
//释放超平面结构体  
static void hyperrect_free(struct kdhyperrect *rect)  
{  
    free(rect->min);  
    free(rect->max);  
    free(rect);  
}  
  
//赋值超平面结构体  
static struct kdhyperrect* hyperrect_duplicate(const struct kdhyperrect *rect)  
{  
    return hyperrect_create(rect->dim, rect->min, rect->max);  
}  
  
//更新超平面结构体最大\最小值数组  
static void hyperrect_extend(struct kdhyperrect *rect, const double *pos)  
{  
    int i;  
  
    for (i=0; i < rect->dim; i++) {  
        if (pos[i] < rect->min[i]) {  
            rect->min[i] = pos[i];  
        }  
        if (pos[i] > rect->max[i]) {  
            rect->max[i] = pos[i];  
        }  
    }  
}  
  
//计算固定坐标点与超平面之间的距离  
static double hyperrect_dist_sq(struct kdhyperrect *rect, const double *pos)  
{  
    int i;  
    double result = 0;  
  
    for (i=0; i < rect->dim; i++)   
    {  
        if (pos[i] < rect->min[i])   
        {  
            result += SQ(rect->min[i] - pos[i]);  
        }   
        else if (pos[i] > rect->max[i])   
        {  
            result += SQ(rect->max[i] - pos[i]);  
        }  
    }  
    return result;  
}  
  
  
/* ---- static helpers ---- */  
#ifdef USE_LIST_NODE_ALLOCATOR  
/* special list node allocators. */  
static struct res_node *free_nodes;  
  
#ifndef NO_PTHREADS  
static pthrea mutex_t alloc_mutex = PTHREA MUTEX_INITIALIZER;  
#endif  
  
//创建返回结果节点  
static struct res_node *alloc_resnode(void)  
{  
    struct res_node *node;  
  
#ifndef NO_PTHREADS  
    pthrea mutex_lock(&alloc_mutex);  
#endif  
  
    if(!free_nodes) {  
        node = malloc(sizeof *node);  
    } else {  
        node = free_nodes;  
        free_nodes = free_nodes->next;  
        node->next = 0;  
    }  
  
#ifndef NO_PTHREADS  
    pthrea mutex_unlock(&alloc_mutex);  
#endif  
  
    return node;  
}  
  
//释放返回结果节点  
static void free_resnode(struct res_node *node)  
{  
#ifndef NO_PTHREADS  
    pthrea mutex_lock(&alloc_mutex);  
#endif  
  
    node->next = free_nodes;  
    free_nodes = node;  
  
#ifndef NO_PTHREADS  
    pthrea mutex_unlock(&alloc_mutex);  
#endif  
}  
#endif  /* list node allocator or not */  
  
  
/* inserts the item. if dist_sq is >= 0, then do an ordered insert */  
/* TODO make the ordering code use heapsort */  
//函数参数: 返回结果节点指针,树节点指针,距离函数  
//将一个结果节点插入到返回结果的列表中  
static int rlist_insert(struct res_node *list, struct kdnode *item, double dist_sq)  
{  
    struct res_node *rnode;  
  
    //创建一个返回结果的节点  
    if(!(rnode = (res_node*)alloc_resnode()))   
    {  
        return -1;  
    }  
    rnode->item = item;           //对应的树节点  
    rnode->dist_sq = dist_sq;     //对应的距离值  
  
    //当距离大于零的时候  
    if(dist_sq >= 0.0)   
    {  
        while(list->next && list->next->dist_sq < dist_sq)   
        {  
            list = list->next;  
        }  
    }  
    rnode->next = list->next;  
    list->next = rnode;  
    return 0;  
}  
  
//清除返回结果的集合  
//本质上是个双链表中单链表的清理  
static void clear_results(struct kdres *rset)  
{  
    struct res_node *tmp, *node = rset->rlist->next;  
  
    while(node)   
    {  
        tmp = node;  
        node = node->next;  
        free_resnode(tmp);  
    }  
  
    rset->rlist->next = 0;  
}  