#pragma once
#include "CityGMLCommon.h"
#include "Boost3DPointHash.h"
#include "CFurnitureData.h"

/*!
 * @brief 横断歩道の回転基準ベクトル算出用のデータクラス
*/
class CRotateAngleVecData
{
public:
    int nCount;     // 頻出数
    CVector2D vec;  // 単位ベクトル
    CVector2D pos;  // ベクトルの始点
    double dAngle;  // 角度deg
    double dLength; // ベクトルの長さ

    /*!
     * @brief コンストラクタ
    */
    CRotateAngleVecData()
    {
        nCount = 0;
        dAngle = 0;
        dLength = 0;
        vec = CVector2D(0, 0);
        pos = CVector2D(0, 0);
    }
    /*!
     * @brief コンストラクタ
    */
    CRotateAngleVecData(
        const CVector2D &vec, const CVector2D &pt,
        const double dAngle, const double dLength, const int nCount)
    {
        this->nCount = nCount;
        this->dAngle = dAngle;
        this->dLength = dLength;
        this->vec = vec;
        this->pos = pt;
    }

    /*!
     * @brief コピーコンストラクタ
    */
    CRotateAngleVecData(const CRotateAngleVecData &data) { *this = data; }

    /*!
     * @brief 代入演算子
    */
    CRotateAngleVecData &operator =(const CRotateAngleVecData &data)
    {
        if (&data != this)
        {
            nCount = data.nCount;
            dAngle = data.dAngle;
            dLength = data.dLength;
            vec = data.vec;
            pos = data.pos;
        }
        return *this;
    }

    /*!
     * @brief 比較演算子(ソート用)
    */
    bool operator <(const CRotateAngleVecData &other) const
    {
        if (nCount == other.nCount)
        {
            return dLength > other.dLength; // 頻出数が同一の場合は、長さ確認
        }
        else
        {
            return nCount < other.nCount;
        }
    }

private:
};


/*!
 * @brief 横断歩道の回転基準ベクトル算出用のデータの管理クラス
*/
class CRotateAngleVecDataManager
{
public:
    std::vector<CRotateAngleVecData> data;  // データ
    double dEpsilon;    // 角度誤差deg

    /*!
     * @brief コンストラクタ
    */
    CRotateAngleVecDataManager() : dEpsilon(3.0) {};

    /*!
     * @brief コピーコンストラクタ
    */
    CRotateAngleVecDataManager(const CRotateAngleVecDataManager &mng) { *this = mng; }

    /*!
     * @brief 代入演算子
    */
    CRotateAngleVecDataManager &operator =(const CRotateAngleVecDataManager &mng)
    {
        if (&mng != this)
        {
            data = mng.data;
            dEpsilon = mng.dEpsilon;
        }
        return *this;
    }

    /*!
     * @brief 追加処理
     * @param datum 横断歩道の回転基準ベクトル算出用のデータ
    */
    void Add(const CRotateAngleVecData &datum);

    /*!
     * @brief 頻出数が昇順、長さは昇降順選択でソート
     * @param increasingOrder 長さ昇順ソートフラグ
    */
    void Sort(const bool increasingOrder = false)
    {
        if (increasingOrder)
        {
            std::sort(data.begin(), data.end(), [](const CRotateAngleVecData &a, const CRotateAngleVecData &b)
                {
                    if (a.nCount == b.nCount)
                        return a.dLength < b.dLength;   // 長さが昇順
                    else
                        return a.nCount < b.nCount;
                });
        }
        else
        {
            std::sort(data.begin(), data.end(), [](const CRotateAngleVecData &a, const CRotateAngleVecData &b)
                {
                    if (a.nCount == b.nCount)
                        return a.dLength > b.dLength;   // 長さが降順
                    else
                        return a.nCount < b.nCount;
                });
        }
    };

private:
};

/*!
 * @brief 都市設備データのユーティリティクラス
*/
class CFurnitureDataUtil
{
public:
    /*!
     * @brief 都市設備品質情報のlodType(LOD3の場合の詳細度)を取得する
     * @param pFurniture    CityGMLの都市設備データのポインタ
     * @param dLodType      LOD3の場合の詳細度
     * @return      データの有無
     * @retval      true    有り
     * @retval      false   無し
    */
    static bool GetFurnitureDataQualityAttributeLodType(
        const citygml::CityObject *pFurniture, double &dLodType);

    /*!
     * @brief CityObjectからfrn:functionを取得
     * @param pFurniture    CityGMLの都市設備データのポインタ
     * @param furnitureData 都市設備データクラス
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
     */
    static bool GetFurnitureFunction(
        const citygml::CityObject *pFurniture,
        CFurnitureData &furnitureData);

    /*!
     * @brief LOD1,2,3幾何情報の取得
     * @param pFurniture    CityGMLの都市設備データのポインタ
     * @param furnitureData 都市設備データクラス
     * @param nJPZone       平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @return 処理結果
     * @retval true     成功(何かしらの幾何情報が取得出来た場合)
     * @retval false    失敗
    */
    static bool GetGeometry(
        const citygml::CityObject *pFurniture,
        CFurnitureData &furnitureData,
        const int nJPZone);

    /*!
     * @brief 横断歩道と点字ブロックの場合CityObjectから都市設備データを取得する
     * @param pFurniture    CityGMLの都市設備データのポインタ
     * @param furnitureData 都市設備データクラス
     * @param nJPZone       平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    static bool GetPedestrianCrossingOrBrailleBlocks(
        const citygml::CityObject *pFurniture,
        CFurnitureData &furnitureData,
        const int nJPZone);

    /*!
     * @brief 横断歩道の中心線の作成
     * @param[in] pFurniture    都市設備(横断歩道)データ
     * @param[in] dParallelEpsilon  並行確認時の許容誤差
     */
    static void CreateCenterLineOfPedestrianCrossing(
        std::shared_ptr<CFurnitureData> &pFurniture,
        const double dParallelEpsilon = 3.0);

    /*!
     * @brief 横断歩道の重複確認
     * @param[in/out]   furnitures 横断歩道群
     * @param[in]       dDistTh    重複判断用の距離しきい値m
     * @note  歩行者横断歩道と自転車横断歩道が隣接している場合は自転車横断歩道を使用しない設定に変更する
    */
    static void DuplicateCheck(
        std::vector<std::shared_ptr<CFurnitureData>> &furnitures,
        const double dDistTh = 0.2);

private:

    /*!
     * @brief 横断歩道の中心線作成の前処理
     * @param[in]   srcPolygons         入力ポリゴン群
     * @param[out]  simplePolygons      入力ポリゴンの簡略化ポリゴン群
     * @param[out]  validPolygons       横断歩道の方向算出で使用するポリゴン群
     * @param[out]  mbrs                横断歩道の方向算出で使用するポリゴンのMBR
     * @param[out]  rotMngForCheckType  横断歩道の種別判定用データ
     * @param[out]  rotMngForStripe     縞タイプの回転方向決定用データ
     * @param[out]  dLengthTh           ジオメトリの長辺の長さしきい値
     * @param[out]  dAreaRateTh         面積比(ジオメトリ面積/MBR面積)のしきい値
    */
    static void preprocessForPedestrianCrossing(
        const Boost3DHashMultiPolygon &srcPolygons,
        BoostMultiPolygon &simplePolygons,
        BoostMultiPolygon &validPolygons,
        Boost3DHashMultiPolygon &mbrs,
        CRotateAngleVecDataManager &rotMngForCheckType,
        CRotateAngleVecDataManager &rotMngForStripe,
        const double dLengthTh = 1.0,
        const double dAreaRateTh = 0.8);

    /*!
     * @brief 平均、分散、標準偏差の算出
     * @param[in]   values  母集団
     * @param[out]  dAve    平均値
     * @param[out]  dVar    分散
     * @param[out]  dStd    標準偏差
    */
    static bool calcParam(
        const std::vector<double> values,
        double &dAve, double &dVar, double &dStd);

    /*!
     * @brief 縦線タイプの横断歩道判定用の矩形作成
     * @param[out]  dst         矩形群
     * @param[in]   src         入力ポリゴン群
     * @param[in]   baseVec     回転角算出用ベクトル
     * @param[in]   centerPt    回転中心座標
     * @param[in]   dLength     矩形幅
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    static bool makeRects(
        BoostMultiPolygon &dst,
        const BoostMultiPolygon &src,
        const CVector2D &baseVec,
        const CVector2D &centerPt,
        const double dRectWidth = 0.5);
};
