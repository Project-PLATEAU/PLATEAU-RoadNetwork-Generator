#include "CUtil.h"
#include <windows.h>
#include <string>
#include <vector>

/*!
 * @brief 小数点以下n桁での四捨五入
 * @param[in] dValue    四捨五入対象
 * @param[in] nDigit    小数点以下の有効桁数(nDisit+1の桁を四捨五入する)
 * @return  四捨五入結果
*/
double CUtil::RoundN(double dValue, int nDigit)
{
    double dPow = pow(10.0, static_cast<double>(nDigit));
    double dTmp = dValue * dPow;

    double dRet = dValue;
    if (CEpsUtil::Greater(dTmp, 0))
    {
        dRet = floor(dTmp + 0.5) / dPow;
    }
    else if (CEpsUtil::Less(dTmp, 0))
    {
        dRet = floor(abs(dTmp) + 0.5) / dPow * -1.0;
    }

    return dRet;
}

/*!
 *@brief 小数点以下n桁での頂点座標の四捨五入
 *@param[in] Boost3DPoint    四捨五入対象
 *@param[in] nDigit    小数点以下の有効桁数(nDisit + 1の桁を四捨五入する)
 *@return  四捨五入結果
 */
Boost3DPoint CUtil::RoundNPoint(const Boost3DPoint &pt, int nDigit)
{
    double dX = RoundN(pt.x(), nDigit);
    double dY = RoundN(pt.y(), nDigit);
    double dZ = RoundN(pt.z(), nDigit);
    return Boost3DPoint(dX, dY, dZ);
}

/*!
 * @brief 文字列変換(Shift_JIS -> UTF-8)
 * @param[in]   strShiftJis 文字列(Shift_JIS)
 * @return      文字列(UTF-8)
*/
std::string CUtil::ConvShiftJisToUtf8(const std::string strShiftJis)
{
    int nLenUniCode = MultiByteToWideChar(CP_ACP, 0, strShiftJis.data(), -1, NULL, 0);
    std::vector<wchar_t> buffUnicode(nLenUniCode, L'\0');
    MultiByteToWideChar(
        CP_ACP, 0, strShiftJis.c_str(), -1, buffUnicode.data(), buffUnicode.size());
    int nLenUtf8 = WideCharToMultiByte(
        CP_UTF8, 0, buffUnicode.data(), -1, nullptr, 0, nullptr, nullptr);
    std::vector<char> buffUtf8(nLenUtf8, L'\0');
    WideCharToMultiByte(
        CP_UTF8, 0, buffUnicode.data(), -1,
        buffUtf8.data(), buffUtf8.size(), nullptr, nullptr);
    buffUtf8.resize(std::char_traits<char>::length(buffUtf8.data()));
    buffUtf8.shrink_to_fit();
    std::string strUtf8(buffUtf8.begin(), buffUtf8.end());
    return strUtf8;
}