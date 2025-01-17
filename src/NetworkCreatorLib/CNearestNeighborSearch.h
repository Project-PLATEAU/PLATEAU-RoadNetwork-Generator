#pragma once
#include "Boost3DPointHash.h"

#pragma region rtree定義

// Boost3DPointHashのRTree定義
typedef bg::index::rtree<Boost3DPointHash, bg::index::quadratic<16>> BoostRTree;


#pragma endregion

/*!
 * @brief 近傍探索クラス
*/
class CNearestNeighborSearch
{
public:
    /*!
     * @brief コンストラクタ
     * @param pts 入力点群
    */
    CNearestNeighborSearch(const Boost3DMultiPointHashs &pts);
    ~CNearestNeighborSearch(void) {};  // デストラクタ

    /*!
     * @brief 近傍点探索(点数指定)
     * @param pt    注目頂点
     * @param n     探索点数
     * @return      近傍点群
    */
    Boost3DMultiPointHashs NNSearch(const Boost3DPointHash &pt, const int n);

    /*!
     * @brief 近傍点探索(半径指定)
     * @param pt    注目頂点
     * @param r     半径距離
     * @return      近傍点群
    */
    Boost3DMultiPointHashs RadiusSearch(const Boost3DPointHash &pt, const double r);

private:
    BoostRTree m_rtree;
};

