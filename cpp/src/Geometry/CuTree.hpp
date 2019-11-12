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
    __device__ T dist(CuVec3<T> const& point) const;
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

template <typename T>
__device__ T CuCuboid<T>::dist(CuVec3<T> const& point) const{
    T d = 0;
    CuVec3<T> max;
    for(int i = 0; i < 3; ++i){
        max[i] = minCoords[i] - point[i] > point[i] - maxCoords[i] ? minCoords[i] - point[i] : point[i] - maxCoords[i];
        if(max[i] > 0){
            d += max[i]*max[i];
        }
    }
    return d;
}

template <class T>
class CuTree{
public:
    CuTree(unsigned int maxNodeSize = 4);
    ~CuTree();
    void preprocess(std::vector<T>* objects);
    void findNearestObject(Vector3<double> const & queryPoint, T const **nearestObject, double *minDistanceSqr) const;
    bool isEmpty() const;
    void PrintDataStructureStats() const;
    void printSearchStats() const;

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
            int range = tree[j-1+i].objectsBegin - tree[j-1+i].objectsEnd, obB = tree[j-1+i].objectsBegin, mid = obB+(range>>1);
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
            tree[(j<<1)+(i<<1)].objectsBegin = obB;
            tree[(j<<1)+(i<<1)].objectsEnd = mid;
            tree[(j<<1)+(i<<1)+1].objectsBegin = mid;
            tree[(j<<1)+(i<<1)+1].objectsEnd = tree[j-1+i].objectsEnd;
            tree[j-1+i].objectsBegin = 0;
            tree[j-1+i].objectsEnd = 0;
        }
        dim = dim+1 == 3 ? 0 : dim+1;
        __syncthreads();
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
    curSize = mask;
    cudaMalloc((void**)&d_aabbTree, mask*sizeof(Node));
    cudaMalloc((void**)&d_sortedObjects, objects.size()*sizeof(T));
    cudaMemcpy(d_sortedObjects, objects->data(), objects->size()*sizeof(T));
    cuPreprocess<<<1, 512>>>(d_aabbTree, mask, d_sortedObjects, objects->size());
    cudaDeviceSynchronize();
}



template <typename T>
__global__ void cuNearest(CuVec3<T>* points, unsigned int numPoints, Node* tree, unsigned int nodes, T* objs, unsigned int numObjs, T*** stack, unsigned int ss, T const ** nearestObject, double** minDistanceSqr){
    if(nodes == 0)
        return;
    extern __shared__ T** s[];
    int i = threadIdx.x+blockDim.x*blockIdx.x, offset = gridDim.x*blockDim.x;
    s[i] = stack[i];
    for(int j = i; j < numPoints; j += offset){
        //for each queryPoint, descend down tree, pushing nodes to stack for this thread and finding minD^2 for each node. Store to nearestObject[j] and store minD^2 to minDistanceSquared[j]
        s[0] = &tree[0];
        while(true){

        }
    }
}

template <typename T>
__global__ void initializeStacks(T*** stack, unsigned int numT, unsigned int stackSize){
    unsigned int i = threadIdx.x+blockIdx.x*blockDim.x, offset = gridDim.x*blockDim.x;
    for(int j = i; j < numT; j += offset){
        for(int k = 0; k < stackSize; ++k){
            stack[j][k] = 0;
        }
    }
}

template <typename T>
void CuTree<T>::findNearestObject(vector<Vector3<double>> const & queryPoints, T const ** nearestObject, double** minDistanceSqr) const{
    unsigned int ss = (curSize - 1) >> 1;
    T*** stacks, d_stacks;
    stacks = new T**[NUMTHREADS];
    for(int i = 0; i < NUMTHREADS; ++i){ //this stack configuration is extremely inefficient as far as access time, need to coalesce
        cudaMalloc((void**)&stacks[i], sizeof(T*)*ss);
    }
    cudaMalloc((void**)&d_stacks, sizeof(T**)*NUMTHREADS);
    cudaMemcpy(d_stacks, stacks, sizeof(T**)*NUMTHREADS, cudaMemcpyHostToDevice);
    initializeStacks<<<1, NUMTHREADS>>>(d_stacks, NUMTHREADS, ss);
    cudaDeviceSynchronize();
    cuNearest<<<1, NUMTHREADS, NUMTHREADS*sizeof(T***)>>>(d_qp, queryPoints.size(), d_aabbTree, curSize, d_stacks, ss, d_nO, d_minD);
}

#endif