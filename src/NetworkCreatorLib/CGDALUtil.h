#pragma once
#include "gdal/gdal.h"
#include "gdal/gdal_priv.h"
#include "gdal/ogrsf_frmts.h"
#include "Boost3DPointHash.h"
#include "CommonDef.h"
#include "opencv2/opencv.hpp"

/*!
 * @brief GDAL用のユーティリティクラス(シングルトン)
*/
class CGDALUtil
{

private:
    static CGDALUtil m_instance;    // 自クラス唯一のインスタンス

    /*!
     * @brief コンストラクタ
    */
    CGDALUtil();

    /*!
     * @brief デストラクタ
    */
    virtual ~CGDALUtil();

public:

    /*!
     * @brief インスタンスの取得
     * @return 入力設定インスタンス
    */
    static CGDALUtil *GetInstance() { return &m_instance; }

    /*!
     * @brief ポリゴン融合
     * @param[in] srcPolygons 融合するポリゴン群
     * @param[in] isUseZ      z座標の使用可否
     * @param[in] isRound     丸め座標の使用可否
     * @param[in] nDigit      丸め座標使用時の小数点以下の桁数
     * @return 融合後のポリゴン群
     * @note   接しているポリゴンごとに融合する
    */
    static Boost3DHashMultiPolygon Dissolve(
        const Boost3DHashMultiPolygon &srcPolygons,
        const bool isUseZ = true,
        const bool isRound = false,
        const int nDigit = ncl_common_def::POINT_SIGNIFICANT_DIGITS_FOR_DISSOLVE);

    /*!
     * @brief ラスタライズ
     * @param[in]  nBaseX       基準地理x座標
     * @param[in]  nBaseY       基準地理y座標
     * @param[in]  dResoX       x座標の解像度
     * @param[in]  dResoY       y座標の解像度
     * @param[in]  polygon      ポリゴン
     * @param[in]  dResolution  入力解像度
     * @return  opencvの画像配列
    */
    static cv::Mat Rasterize(
        double &dBaseX,
        double &dBaseY,
        double &dResoX,
        double &dResoY,
        const Boost3DHashPolygon &polygon,
        const double dResolution = 0.1);

    /*!
     * @brief ラスタ画像サイズの算出
     * @param[out] nWidth       画像幅
     * @param[out] nHeight      画像高さ
     * @param[in]  polygon      ポリゴン
     * @param[in]  dResolution  解像度
    */
    static void GetRasterImgSize(
        int &nWidth,
        int &nHeight,
        const Boost3DHashPolygon &polygon,
        const double dResolution = 0.1);


    /*!
     * @brief ConvexHull
     * @param[in] srcPolygons ConvexHull対象ポリゴン群
     * @param[in] isUseZ      z座標の使用可否
     * @param[in] isRound     丸め座標の使用可否
     * @param[in] nDigit      丸め座標使用時の小数点以下の桁数
     * @return ConvexHull結果のポリゴン
    */
    static Boost3DHashPolygon ConvexHull(
        const Boost3DHashMultiPolygon &srcPolygons,
        const bool isUseZ = true,
        const bool isRound = false,
        const int nDigit = ncl_common_def::POINT_SIGNIFICANT_DIGITS_FOR_DISSOLVE);

};
