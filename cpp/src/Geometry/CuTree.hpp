#ifndef CUTREE_HPP
#define CUTREE_HPP

#include "Vector3.h"
#include <vector>
//#include "Cuboid.h"

template <typename T>
class CuVec3{
public:
    __device__ __host__ CuVec3();
    __device__ __host__ const T& operator[](const int& d);
    T x, y, z;
};

template <typename T>
__device__ __host__ CuVec3<T>::CuVec3(){
    x = 0;
    y = 0;
    z = 0;
}

template <typename T>
__device__ __host__ const T& CuVec3<T>::operator[](const int& d){
    if(d == 0) return x;
    if(d == 1) return y;
    if(d == 2) return z;
    return 0;
}

template <typename T>
class CuCuboid{
public:
    __device__ __host__ CuCuboid();
    CuVec3<T> maxCoords, minCoords;
    __device__ void setMaxCoords(T x, T y, T z);
    __device__ void setMinCoords(T x, T y, T z);
};

template <typename T>
__device__ __host__ CuCuboid<T>::CuCuboid() : maxCoords(), minCoords() {
    
}

template <typename T>
__device__ void CuCuboid<T>::setMaxCoords(T x, T y, T z){
    this->maxCoords.x = x;
    this->maxCoords.y = y;
    this->maxCoords.z = z;
}

template <typename T>
__device__ void CuCuboid<T>::setMinCoords(T x, T y, T z){
    this->minCoords.x = x;
    this->minCoords.y = y;
    this->minCoords.z = z;
}

template <class T>
class CuTree{
//private:
    /*struct CuTreeNode{
        CuCuboid<T> bb;
    };*/
public:
    CuTree(unsigned int maxNodeSize = 4);
    ~CuTree();
    void preprocess(std::vector<T>* objects);
    void findNearestObject(Vector3<double> const & queryPoint, T const **nearestObject, double *minDistanceSqr) const;
    bool isEmpty() const;
    void PrintDataStructureStats() const;
    void printSearchStats() const;

//private:
    struct Node{
        CuCuboid<double> aabb;
        unsigned int objectBegin, objectsEnd;
    };

    const unsigned int maxNodeSize;
    unsigned int curSize;
    std::vector<Node> aabbTree;
    Node* d_aabbTree;
    std::vector<T> *sortedObjects;
    T* d_sortedObjects;
};

template <typename T>
CuTree<T>::CuTree(unsigned int maxNodeSize) : maxNodeSize(maxNodeSize){
    d_aabbTree = nullptr;
    d_sortedObjects = nullptr;
    curSize = 0;
}

template <typename T>
CuTree<T>::~CuTree(){
    if(d_aabbTree != nullptr){
        cudaFree(d_aabbTree);
    }
    if(d_sortedObjects != nullptr){
        cudaFree(d_sortedObjects);
    }
}

template <typename T>
__global__ void cuPreprocess(Node* tree, unsigned int nodes, T* objs, unsigned int numObjs){
    unsigned int i = threadIdx.x+blockIdx.x*blockDim.x, offset = gridDim.x*blockDim.x;
    short dim = 0;
    for(unsigned int j = 1; j < numObjs; j <<= 1){
        if(i < j){
            int range = nodes[j-1+i].objectsBegin - nodes[j-1+i].objectsEnd, obB = nodes[j-1+i].objectsBegin;
            for(int l = 0; l < range; ++l){
                double midl = objs[obB+l].maxCoords[dim]+objs[obB+l].minCoords[dim];
                for(int k = l+1; k < range; ++k){
                    double midk = objs[obB+k].maxCoords[dim]+objs[obB+k].minCoords[dim];
                    if(midk < midl){
                        T temp = objs[obB+k];
                        objs[obB+k] = objs[obB+l];
                        objs[obB+l] = temp;
                    }
                }
            }
        }
        dim = dim+1 == 3 ? 0 : dim+1;
    }
}

template <typename T>
void CuTree<T>::preprocess(vector<T>* objects){
    unsigned int mask = 0x80000000;
    if(objects->empty())
        return;

    while(!(mask & objects->size()))
        mask >>= 1;
    mask <<= 1;

    cudaMalloc((void**)&d_aabbTree, mask*sizeof(Node));
    cudaMalloc((void**)&d_sortedObjects, objects.size()*sizeof(T));
    cudaMemcpy(d_sortedObjects, objects->data(), objects->size()*sizeof(T));
    cuPreprocess<<<1, 512>>>(d_aabbTree, aabbTree.size(), d_sortedObjects, objects->size());
    cudaDeviceSynchronize();
}

#endif