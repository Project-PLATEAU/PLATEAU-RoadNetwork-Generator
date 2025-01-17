#pragma once
#include "BoostCommon.h"
#include "Boost3DPointHash.h"

/*!
 * @brief 中心線データ
*/
class CCenterLineData
{
public:
    /*!
     * @brief コンストラクタ
    */
    CCenterLineData(void)
    {
        this->dMinWidth = 0;
        this->isIntersection = false;
    };

    /*!
     * @brief コンストラクタ
     * @param line 中心線
    */
    CCenterLineData(const Boost3DHashPolyline &line) : CCenterLineData()
    {
        this->centerLine = line;
    }

    /*!
     * @brief デストラクタ
    */
    ~CCenterLineData() {};

    /**
     * コピーコンストラクタ
    */
    CCenterLineData(const CCenterLineData &x) { *this = x; }

    /**
     * 代入演算子
    */
    CCenterLineData &operator = (const CCenterLineData &data)
    {
        if (this != &data)
        {
            this->centerLine = data.centerLine;
            this->dMinWidth = data.dMinWidth;
            this->minWidthPos = data.minWidthPos;
            this->isIntersection = data.isIntersection;
        }
        return *this;
    }

    Boost3DHashPolyline centerLine;     // 中心線
    double              dMinWidth;      // 最小幅員
    Boost3DPointHash    minWidthPos;    // 最小幅員地点
    bool                isIntersection; // 交差点の中心線か否か(車道用)

private:
};
