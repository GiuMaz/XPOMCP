#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "memorypool.h"
#include <algorithm>
#include <iostream>


#define LargeInteger 1000000
#define Infinity 1e+10
#define Tiny 1e-10

#ifdef DEBUG
#define safe_cast dynamic_cast
#else
#define safe_cast static_cast
#endif

namespace UTILS
{

inline int Sign(int x)
{
    return (x > 0) - (x < 0);
}

inline int Random(int max)
{
    return rand() % max;
}

inline int Random(int min, int max)
{
    return rand() % (max - min) + min;
}

inline double RandomDouble(double min, double max)
{
    return (double) rand() / RAND_MAX * (max - min) + min;
}

inline void RandomSeed(int seed)
{
    srand(seed);
}

inline bool Bernoulli(double p)
{
    return rand() < p * RAND_MAX;
}

inline bool Near(double x, double y, double tol)
{
    return fabs(x - y) <= tol;
}

inline bool CheckFlag(int flags, int bit) { return (flags & (1 << bit)) != 0; }

inline void SetFlag(int& flags, int bit) { flags = (flags | (1 << bit)); }

template<class T>
inline bool Contains(std::vector<T>& vec, const T& item)
{
    return std::find(vec.begin(), vec.end(), item) != vec.end();
}

void UnitTest();

}

inline void visit(int nodeI, bool* markedNodes, int* nMarkedNodes, std::vector<double*>* stateVarRelationships, std::vector<int>* connComp){
    connComp->push_back(nodeI);
    (*nMarkedNodes)++;
    std::vector<int> adjNodes;
    for(std::vector<double*>::iterator it = stateVarRelationships->begin() ; it != stateVarRelationships->end(); ++it){
        if((*it)[0]==nodeI && markedNodes[(int)((*it)[1])]==0){
            adjNodes.push_back((int)((*it)[1]));
            markedNodes[(int)((*it)[1])]=1;
        }
        if((*it)[1]==nodeI && markedNodes[(int)((*it)[0])]==0){
            adjNodes.push_back((int)((*it)[0]));
            markedNodes[(int)((*it)[0])]=1;
        }
    }
    for(std::vector<int>::iterator it = adjNodes.begin() ; it != adjNodes.end(); ++it){
        visit((*it), markedNodes, nMarkedNodes, stateVarRelationships,connComp);
    }
}

inline std::vector<std::vector<int>*>* computeConnComp(std::vector<double*>* stateVarRelationships, int nNodes){
    int nConnComp=0;
    bool* markedNodes=new bool[nNodes];
    for(int i=0; i<nNodes; i++){
        markedNodes[i]=0;
    }
    std::vector<std::vector<int>*>* connComps = new std::vector<std::vector<int>*>();
    int nMarkedNodes=0;
    int nodeI;
    while(nMarkedNodes<nNodes){
        nConnComp++;
        connComps->push_back(new std::vector<int>());
        nodeI=0;
        while(markedNodes[nodeI]==1)
            nodeI++;
        markedNodes[nodeI]=1;
        visit(nodeI,markedNodes,&nMarkedNodes,stateVarRelationships,(*connComps)[nConnComp-1]);
    }
    return connComps;
}

inline void printConnectedComponents(std::vector<std::vector<int>*>* connComps){
    std::cout << "Printing connected components " << std::endl;
    int cn=0;
    for(std::vector<std::vector<int>*>::iterator it = connComps->begin() ; it != connComps->end(); ++it){
        std::cout << "Component: " << cn << std::endl;
        for(std::vector<int>::iterator it1 = (*it)->begin() ; it1 != (*it)->end(); ++it1){
            std::cout << (*it1) << " ";
        }
        cn++;
        std::cout << std::endl;
    }
}

#endif // UTILS_H
