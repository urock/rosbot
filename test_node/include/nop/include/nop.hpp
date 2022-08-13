#pragma once

#include "baseFunctions.hpp"
#include <map>
#include <string>
#include <vector>


// class network operator
class NetOper
{
public:
    NetOper();

    // RPCntrol
    void calcResult(const std::vector<float>& x_in, std::vector<float>& y_out);

    float getUnaryOperationResult(int operationNum, float input);
    float getBinaryOperationResult(int operationNum, float left, float right);
    
    
    // void setOutputsNum(size_t newNum);
    // size_t getOutputsNum();

    // TODO use move semantics and rvalues
    void setNodesForVars(const std::vector<int>& nodes);
    const std::vector<int>& getNodesForVars();

    void setNodesForParams(const std::vector<int>& nodes);
    const std::vector<int>& getNodesForParams();

    void setNodesForOutput(const std::vector<int>& nodes);
    const std::vector<int>& getNodesForOutput();

    void setCs(const std::vector<float>& newParams);
    const std::vector<float>& getCs();

    void setPsi(const std::vector<std::vector<int>>& newMatrix);
    const std::vector<std::vector<int>>& getPsi();

    
private:
    void initUnaryFunctionsMap();
    void initBinaryFunctionsMap();

private:
    // size_t m_matrixDimension;              // L
    size_t m_numOutputs;                   // Mout
    // std::vector<float> m_variables;        // Vs // probably useless
    std::vector<float> m_parameters;       // Cs
    std::vector<int> m_unaryOperations;    // O1s
    std::vector<int> m_binaryOperations;   // O2s
    std::vector<int> m_nodesForVars;       // Pnum
    std::vector<int> m_nodesForParams;     // Rnum
    std::vector<int> m_nodesForOutput;     // Dnum
    std::vector<float> z;                  // z
    std::vector<std::string> m_nodesExpr;  // zs

    std::vector<std::vector<int>> m_matrix; // Psi

    std::map<int, float(*)(float)> m_unaryFuncMap;
    std::map<int, float(*)(float, float)> m_binaryFuncMap;



};

extern const std::vector<std::vector<int>> NopPsiN;
extern std::vector<float> qc;

extern const std::vector<std::vector<int>> Psi;
