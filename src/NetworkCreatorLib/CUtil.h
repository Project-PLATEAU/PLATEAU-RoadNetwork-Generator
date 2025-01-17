#pragma once
#include "CEpsUtil.h"
#include "BoostCommon.h"


class CUtil
{
public:
    /*!
     * @brief 小数点以下n桁での四捨五入
     * @param[in] dValue    四捨五入対象
     * @param[in] nDigit    小数点以下の有効桁数(nDisit+1の桁を四捨五入する)
     * @return  四捨五入結果
    */
    static double RoundN(double dValue, int nDigit);

    /*!
     * @brief 小数点以下n桁での頂点座標の四捨五入
     * @param[in] Boost3DPoint    四捨五入対象
     * @param[in] nDigit    小数点以下の有効桁数(nDisit+1の桁を四捨五入する)
     * @return  四捨五入結果
    */
    static Boost3DPoint RoundNPoint(const Boost3DPoint &pt, int nDigit);

    /*!
     * @brief 文字列変換(Shift_JIS -> UTF-8)
     * @param[in]   strShiftJis 文字列(Shift_JIS)
     * @return      文字列(UTF-8)
    */
    static std::string ConvShiftJisToUtf8(const std::string strShiftJis);

};
