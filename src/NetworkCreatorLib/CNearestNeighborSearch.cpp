#include "CNearestNeighborSearch.h"
#include "CEpsUtil.h"

// コンストラクタ
CNearestNeighborSearch::CNearestNeighborSearch(const Boost3DMultiPointHashs &pts)
{
    m_rtree.clear();
    for (const auto &pt : pts)
    {
        m_rtree.insert(pt);
    }
}

// 近傍点探索(点数指定)
Boost3DMultiPointHashs CNearestNeighborSearch::NNSearch(const Boost3DPointHash &pt, const int n)
{
    Boost3DMultiPointHashs dst;

    if (m_rtree.size() > 0 && n > 0)
    {
        m_rtree.query(bg::index::nearest(pt, n), std::back_inserter(dst));
    }

    return dst;
}

 // 近傍点探索(半径指定)
Boost3DMultiPointHashs CNearestNeighborSearch::RadiusSearch(const Boost3DPointHash &pt, const double r)
{
    Boost3DMultiPointHashs dst;

    if (m_rtree.size() > 0 && r >= 0)
    {
        m_rtree.query(
            bg::index::satisfies([pt, r](const Boost3DPointHash &v)
                { return CEpsUtil::LessEqual(pt.RoundDistance(v), r); }),
            std::back_inserter(dst));
    }

    return dst;
}

