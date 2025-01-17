#pragma once
#include <vector>
#include <memory>
#include "CGeoUtil.h"
#include "CTranRoadData.h"
#include "CBridgeData.h"
#include "CFurnitureData.h"
#include "CSearchNeighbor.h"
#include "CCenterLineData.h"

#define MIN_ZERO_ANGLE 0
#define MAX_ZERO_ANGLE 10
#define MIN_PARALLEL 170
#define MAX_PARALLEL 180

/*!
 * @brief 交差点接続時の隣接道路情報
*/
class NeighborRoadInfo
{
public:
    // 車線数
    // 0: tranRoadDataはあるが中心線なし
    // 1: 車線数が1、または車線数2以上のうちの一つ
    // 2: 車線数が2以上（LOD2以降）
    int RoadCount;

    // RoadCount = 1 の場合
    // 交差点と接続する点、または親の隣接道路情報と接続する点
    // RoadCount = 2以上 の場合
    // 交差点と子の隣接道路情報を結ぶ点
    Boost3DPointHash ConnectPoint;

    // RoadCount = 1 の場合
    // 中心線の方向ベクトル
    // RoadCount = 2以上 の場合
    // 子の隣接道路情報全体の方向ベクトル
    CVector2D ConnectVector2D;

    // RoadCount = 1 の場合
    // 幅員
    // RoadCount = 2以上 の場合
    // 子の隣接道路情報全体の幅員
    double Width;

    // RoadCount = 1 の場合
    // 中心線のポインタ
    std::shared_ptr<CCenterLineData> CenterLinePtr;

    // RoadCount = 2以上 の場合
    // 子道路情報リスト
    std::vector<NeighborRoadInfo> ChildNeighborRoadInfo;

    // RoadCount = 1以上 の場合
    // 描画が完了しているかどうか
    bool IsConnectedIntersection;

public:
    /*!
     * @brief コンストラクタ
    */
    NeighborRoadInfo():
        RoadCount(0),
        ConnectPoint(Boost3DPointHash()),
        ConnectVector2D(CVector2D()),
        Width(0.0),
        CenterLinePtr(nullptr),
        ChildNeighborRoadInfo(std::vector<NeighborRoadInfo>()),
        IsConnectedIntersection(false)
    {

    }

    /*!
     * @brief 車線数1用コンストラクタ
     * @param connectPoint      交差点と接続する点
     * @param connectVector2D   中心線の方向ベクトル
     * @param width             幅員
     * @param tranRoadData      tranRoadDataのポインタ
    */
    NeighborRoadInfo(
        Boost3DPointHash connectPoint,
        CVector2D connectVector2D,
        double width,
        std::shared_ptr<CCenterLineData> centerLinePtr):
        RoadCount(1),
        ConnectPoint(connectPoint),
        ConnectVector2D(connectVector2D),
        Width(width),
        CenterLinePtr(centerLinePtr),
        ChildNeighborRoadInfo(std::vector<NeighborRoadInfo>()),
        IsConnectedIntersection(false)
    {
        connectVector2D.Normalize();
    }

    /*!
     * @brief 車線数2用コンストラクタ
     * @param connectPoint      交差点と接続する点
     * @param connectVector2D   中心線の方向ベクトル
     * @param width             幅員
     * @param tranRoadData      tranRoadDataのポインタ
    */
    NeighborRoadInfo(std::vector<NeighborRoadInfo> childNeighborRoadInfo) :
        RoadCount(childNeighborRoadInfo.size()),
        ConnectPoint(Boost3DPointHash()),
        ConnectVector2D(CVector2D()),
        Width(0.0),
        CenterLinePtr(nullptr),
        ChildNeighborRoadInfo(childNeighborRoadInfo),
        IsConnectedIntersection(false)
    {
        CalculateBasedOnInputedData();
    }

private:
    /*!
     * @brief 車線数2用データ整理
    */
    void CalculateBasedOnInputedData()
    {
        if (ChildNeighborRoadInfo.size() < 1)
        {
            return;
        }

        for (auto info : ChildNeighborRoadInfo)
        {
            ConnectPoint = Boost3DPointHash(
                ConnectPoint.x() + info.ConnectPoint.x(),
                ConnectPoint.y() + info.ConnectPoint.y(),
                ConnectPoint.z() + info.ConnectPoint.z());

            ConnectVector2D = CVector2D(
                ConnectVector2D.x + info.ConnectVector2D.x,
                ConnectVector2D.y + info.ConnectVector2D.y);

            Width += info.Width;
        }

        ConnectPoint = Boost3DPointHash(
            ConnectPoint.x() / ChildNeighborRoadInfo.size(),
            ConnectPoint.y() / ChildNeighborRoadInfo.size(),
            ConnectPoint.z() / ChildNeighborRoadInfo.size());

        ConnectVector2D = CVector2D(
            ConnectVector2D.x / ChildNeighborRoadInfo.size(),
            ConnectVector2D.y / ChildNeighborRoadInfo.size());
        ConnectVector2D.Normalize();
    }
};

class CNetworkCreator
{
public:
    CNetworkCreator(void)
    {
        m_iLod = 1;
    };

    CNetworkCreator(int lod)
    {
        m_iLod = lod;
    };

    ~CNetworkCreator()
    {

    };

    /*!
     * @brief CityObject配列を道路CityGMLに変換する
     * @para[in]  cityObjectList    CityGML配列
     * @para[in]  nJPZone           平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @para[out] dLod3Detail       LOD3の場合の詳細度
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool SetTranRoadData(std::vector<const citygml::CityObject*>& cityObjectList, int nJPZone, double &dLod3Detail);

    /*!
     * @brief CityObject配列を橋梁データに変換する
     * @param cityObjectList    CityGML配列
     * @param nJPZone           平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool SetBridgeData(std::vector<const citygml::CityObject *> &cityObjectList, int nJPZone);

    /*!
     * @brief CityObject配列を都市設備データに変換する
     * @param cityObjectList    CityGML配列
     * @param nJPZone           平面直角座標系の系番号(1-19以外の場合は経緯度座標(CityGMLの値そのまま)とする)
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool SetFurnitureData(std::vector<const citygml::CityObject *> &cityObjectList, int nJPZone);

    /*!
     * @brief エッジ検出
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool DetectEdge();

    /*!
     * @brief ポリゴンからエッジを抽出
     * @param targetPolygonList     エッジ抽出する道路
     * @param neighborPolygonList   隣接道路
     * @param pairEdge              抽出したエッジのペア
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool ExtractRoadEdgeFromPolygon(
        Boost3DHashPolygon& targetPolygon,
        Boost3DHashMultiPolygon& neighborPolygonList,
        std::pair<Boost3DHashPolyline, Boost3DHashPolyline>& pairEdge);

    /*!
     * @brief 誤検出したエッジの分割
     * @param tranRoadData      入力道路情報
     * @param targetPolygon     対象ポリゴン
     * @param isEdgeList        事前出力の重複リスト
     * @param edgePair          出力エッジペア
     * @return 分割後のエッジ本数
    */
    int SplitEdges(
        Boost3DHashPolygon& targetPolygon,
        Boost3DHashMultiPolygon& neighborPolygonList,
        std::pair<Boost3DHashPolyline, Boost3DHashPolyline>& edgePair);

    /*!
     * @brief 行き止まり道路のエッジ検出
     * @param tranRoadData      入力道路情報
     * @param edgePair          出力エッジペア
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool DetectEndOfRoadEdge(CTranRoadData& tranRoadData, std::vector<std::pair<Boost3DHashPolyline, Boost3DHashPolyline>>& edgePairList);

    /*!
     * @brief LOD2以上で島がある道路のエッジ検出
     * @param tranRoadData      入力道路情報
     * @param edgePair          出力エッジペア
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool DetectRoadWithIslandEdge(CTranRoadData& tranRoadData, std::vector<std::pair<Boost3DHashPolyline, Boost3DHashPolyline>>& edgePairList);

    /*!
     * @brief 道路中心線作成
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool CreateCenterLines();

    /*!
     * @brief 二つのポリラインの間のポリラインを求める
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool CreatePolylineBetweenTwoPolylines(
        std::pair<Boost3DHashPolyline, Boost3DHashPolyline>& inputLinePair,
        Boost3DHashPolyline& outputLine);

    /*!
     * @brief 二つのポリラインの間のポリラインを求める
     * @brief 両端の平行ではないポリラインは無視する
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool CreatePolylineBetweenTwoPolylinesUsingParallel(
        std::pair<Boost3DHashPolyline, Boost3DHashPolyline>& inputLinePair,
        Boost3DHashPolyline& outputLine);

    /*!
     * @brief 車道の幅員計測
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool MeasureRoadWidth();

    /*!
     * @brief 交差点接続
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool ConnectIntersection();

    /*!
     * @brief 交差点の隣接道路を取得
     * @param[in ] tranRoadData             対象道路
     * @param[out] neighborRoadList         対象道路（隣接交差部を含む）に隣接している車道部リスト
     * @param[out] intersectionRoadList     対象道路に隣接している交差部リスト
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool GetNeighborRoadOfIntersection(std::shared_ptr<CTranRoadData>& tranRoadData, std::set<std::shared_ptr<CTranRoadData>>& neighborRoadList, std::set<std::shared_ptr<CTranRoadData>>& intersectionRoadList);

    /*!
     * @brief 交差点の隣接道路の情報を取得する
     * @param[in ] neighborRoadPtrList      交差部に隣接している道路情報リスト
     * @param[in ] intersectionPolygonList  交差部の車道部ポリゴンリスト
     * @param[out] neighborRoadInfoList     隣接道路情報リスト
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool GetNeighborRoadInfoList(
        std::set<std::shared_ptr<CTranRoadData>>& neighborRoadPtrList,
        Boost3DHashMultiPolygon& intersectionPolygonList,
        std::vector<NeighborRoadInfo>& neighborRoadInfoList);

    /*!
     * @brief 交差点の主道路を接続する
     * @param[in ] intersectionPolygonList      交差部ポリゴンリスト
     * @param[in ] neighborRoadPolygonList      隣接道路の車道部ポリゴンリスト
     * @param[in ] neighborRoadInfoList         隣接道路情報リスト
     * @param[out] intersectionPolylines        交差点内中心線
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool ConnectMainCenterLine(
        Boost3DHashMultiPolygon& intersectionPolygonList,
        Boost3DHashMultiPolygon& neighborRoadPolygonList,
        std::vector<NeighborRoadInfo>& neighborRoadInfoList,
        Boost3DHashMultiLines& intersectionPolylines);

    /*!
     * @brief 交差点の従道路を接続する
     * @param[in    ] intersectionPolygonList               交差部ポリゴンリスト
     * @param[in    ] intersectionIslandOnlyPolygonList     交差部の島ポリゴンリスト
     * @param[in    ] neighborRoadPolygonList               隣接道路の車道部ポリゴンリスト
     * @param[in    ] neighborRoadInfoList                  隣接道路情報リスト
     * @param[in,out] intersectionPolylines                 交差点内中心線
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool ConnectSubCenterLine(
        Boost3DHashMultiPolygon& intersectionPolygonList,
        Boost3DHashMultiPolygon& intersectionIslandOnlyPolygonList,
        Boost3DHashMultiPolygon& neighborRoadPolygonList,
        std::vector<NeighborRoadInfo>& neighborRoadInfoList,
        Boost3DHashMultiLines& intersectionPolylines);

    /*!
     * @brief 隣接道路接続
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool ConnectNeighborRoad();

    /*!
     * @brief 車道ネットワークデータの出力
     * @param[in] strShpOutputFolder        道路SHP出力フォルダパス
     * @param[in] strGeoJsonOutputFolder    道路GeoJSON出力フォルダパス
     * @param[in] nJPZone                   平面直角座標系の系番号
     * @param[in] bCreateSHP                SHP出力有無
     * @param[in] bCreateGeoJSON            GeoJSON出力有無
     * @param[in] dLod3Detail               LOD3の場合の詳細度
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
     */
    bool OutputRoadwayNetwork(
        const std::string &strShpOutputFolder,
        const std::string &strGeoJsonOutputFolder,
        const int nJPZone,
        const bool bCreateSHP,
        const bool bCreateGeoJSON,
        const double dLod3Detail = 3.0);

    /*!
     * @brief 隣接道路探索
    */
    void SearchNeighborRoad();

    /*!
     * @brief 2つのポリゴンが重畳しているかどうか判別する
     * @param targetPolygon                     入力ポリゴン1
     * @param searchPolygon                     入力ポリゴン2
     * @param targetOverlapEdgeIndexPairList    重畳している辺の始点と終点のペアリスト
     * @return 処理結果
     * @retval true     重畳している
     * @retval false    重畳していない
    */
    bool IsOverlapPolygons(
        Boost3DHashPolygon& targetPolygon,
        Boost3DHashPolygon& searchPolygon,
        std::vector<std::pair<size_t, size_t>>& targetOverlapEdgeIndexPairList);

    /*!
     * @brief 2つのポリゴンが重畳しているかどうか判別する
     * @param targetPolygon                     入力ポリゴン1
     * @param searchPolygonList                 入力ポリゴン2
     * @param targetOverlapEdgeIndexPairList    重畳している辺の始点と終点のペアリスト
     * @return 処理結果
     * @retval true     重畳している
     * @retval false    重畳していない
    */
    bool IsOverlapPolygons(
        Boost3DHashPolygon& targetPolygon,
        Boost3DHashMultiPolygon& searchPolygonList,
        std::vector<std::pair<size_t, size_t>>& targetOverlapEdgeIndexPairList);

    /*!
     * @brief ポリラインのサンプリング
     * @param src           入力ポリライン
     * @param interval      サンプリング間隔
     * @return サンプリング後のポリライン
    */
    Boost3DHashPolyline sampling(Boost3DHashPolyline src, double interval);

    /*!
     * @brief マルチポリラインのサンプリング
     * @param src           入力マルチポリライン
     * @param interval      サンプリング間隔
     * @return サンプリング後のマルチポリライン
    */
    Boost3DHashMultiLines sampling(Boost3DHashMultiLines src, double interval);

    /*!
     * @brief ポリラインのサンプリング
     * @param src       入力ポリライン
     * @param num       サンプリング個数
     * @return サンプリング後のポリライン
    */
    Boost3DHashPolyline sampling(Boost3DHashPolyline src, int num);

    /*!
     * @brief マルチポリラインのサンプリング
     * @param src       入力マルチポリライン
     * @param num       サンプリング個数
     * @return サンプリング後のマルチポリライン
    */
    Boost3DHashMultiLines sampling(Boost3DHashMultiLines src, int num);

    /*!
     * @brief ポリラインの間引き処理
     * @param src       入力ポリライン
     * @return 間引き後のマルチポリライン
    */
    Boost3DHashPolyline ThinOutVerticesOfPolyline(Boost3DHashPolyline src, double interval = 10.0);

    /*!
     * @brief ポリラインの両端をポリゴンにぶつかるまで延長する
     * @param inputPolyline             入力ポリライン
     * @param collisionTargetPolygons   衝突対象ポリゴン群
     * @return 延長後のマルチポリライン
    */
    Boost3DHashPolyline ExtendPolylineUntilPolygon(Boost3DHashPolyline& inputPolyline, Boost3DHashMultiPolygon& collisionTargetPolygons);

    /*!
     * @brief ポリラインの両端がポリゴンからはみ出さないようにトリミングする
     * @param inputPolyline             入力ポリライン
     * @param collisionTargetPolygons   衝突対象ポリゴン群
     * @return トリミング後のマルチポリライン
    */
    Boost3DHashPolyline TrimPolylineUntilPolygon(Boost3DHashPolyline& inputPolyline, Boost3DHashMultiPolygon& collisionTargetPolygons);

    /*!
     * @brief ポリゴン内にポイントが入っているかどうかの確認
     * @brief 高さ情報を無視する
     * @param inputPoint        入力ポイント
     * @param inputPolygons     入力ポリゴン群
     * @return 処理結果
     * @retval true     入っている
     * @retval false    入っていない
    */
    bool CoveredByPolygonsIgnoreZ(
        Boost3DPointHash& inputPoint,
        Boost3DHashMultiPolygon& inputPolygons);

    /*!
     * @brief ポリゴン内にポイントが入っているかどうかの確認
     * @brief 高さ情報を無視する
     * @param inputPoint        入力ポイント
     * @param inputPolygons     入力ポリゴン群
     * @return 処理結果
     * @retval true     入っている
     * @retval false    入っていない
    */
    bool CoveredByPolygonsIgnoreZ(
        Boost3DMultiPointHashs& inputPoints,
        Boost3DHashMultiPolygon& inputPolygons);

    /*!
     * @brief ポリゴンからポリラインの途中がはみ出しているかどうかの確認
     * @param inputPolyline     入力ポリライン
     * @param inputPolygons     入力ポリゴン群
     * @return 処理結果
     * @retval true     はみ出している
     * @retval false    はみ出していない
    */
    bool CheckPolylineProtrudeFromPolygon(
        Boost3DHashPolyline& inputPolyline,
        Boost3DHashMultiPolygon& inputPolygons);

    /*!
     * @brief ポリゴンからポリラインの途中がはみ出しているかどうかの確認
     * @param inputPolylineList     入力ポリライン群
     * @param inputPolygons         入力ポリゴン群
     * @return 処理結果
     * @retval true     はみ出している
     * @retval false    はみ出していない
    */
    bool CheckPolylineProtrudeFromPolygon(
        Boost3DHashMultiLines& inputPolylineList,
        Boost3DHashMultiPolygon& inputPolygons);

    /*!
     * @brief 同数の複数点ペアリング
     * @param[in ] pointList1      複数点セット1
     * @param[in ] pointList2      複数点セット2
     * @param[out] pairingList     ペアリング結果（ペア同士のインデックス）
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool PairMultiplePoints(
        Boost3DMultiPointHashs& pointList1,
        Boost3DMultiPointHashs& pointList2,
        std::vector<std::pair<size_t, size_t>>& pairingList);

    /*!
     * @brief 異なる本数のポリラインを接続する
     * @param inputPolylinePtrList1     入力ポリライン群1
     * @param inputPolylinePtrList2     入力ポリライン群2
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
    */
    bool ConnectCenterLineOfDifferentCount(
        std::vector<Boost3DHashPolyline*> inputPolylinePtrList1,
        std::vector<Boost3DHashPolyline*> inputPolylinePtrList2);

    /*!
     * @brief 入力ベクトルの水平確認
     * @param vec1              入力ベクトル1
     * @param vec2              入力ベクトル2
     * @param checkDirection    確認する2ベクトルの方向
     * @retval 0    順方向か逆方向で水平か確認
     * @retval 正   順方向のみ水平か確認（逆方向は水平でもfalse）
     * @retval 負   逆方向のみ水平か確認（順方向は水平でもfalse）
     * @return 処理結果
     * @retval true         水平
     * @retval false        水平でない
    */
    static bool IsParallel(
        const CVector2D& vec1,
        const CVector2D& vec2,
        int checkDirection);

    /*!
     * @brief 横断歩道の中心線作成
    */
    void CreateCenterLineOfPedestrianCrossing();

    // 歩道のエッジ線検出
    bool DetectEdgeOfFootpath();

    /*!
     * @brief エッジ線の整形(歩道のカーブ部分の除去)
     * @param polyline          エッジ線
     * @param dTotalLengthTh    基準方向決定用の探索距離m
     * @param dAngleTh          基準方向との並行確認時の許容角度deg
    */
    void ShapingEdgeLine(
        Boost3DHashPolyline &polyline,
        const double dTotalLengthTh = 15.0,
        const double dAngleTh = 5.0);

    /*!
     * @brief   歩道の中心線の作成
     * @return  処理結果
    */
    bool CreateFootpathCenterLines();

    /*!
     * @brief 歩道の交差点接続
    */
    void FootpathConnectionByCrossing();

    /*!
     * @brief 横断歩道による歩道の接続
     * @param[in] dSamplingInterval 歩道中心線のサンプリング間隔m
     * @param[in] dNNFootpathDistTh 最近傍歩道を探索する際の距離しきい値m
     * @param[in] dNNPCDistTh       最近傍横断歩道を探索する際の距離しきい値m
     * @param[in] dSearchPCDistTh   横断歩道同士を接続する際の探索範囲m
    */
    void FootpathConnectionByPedestrianCrossing(
        const double dSamplingInterval = 1.0,
        const double dNNFootpathDistTh = 10.0,
        const double dNNPCDistTh = 20.0,
        const double dSearchPCDistTh = 10.0);

    /*!
     * @brief 横断歩道橋の中心線作成
     * @param[in] dInterval サンプリング間隔m(幅員計測用)
    */
    void CreateCenterLineOfPedestrianBridge(const double dInterval = 1.0);

    /*!
     * @brief 横断歩道橋による歩道の接続
     * @param[in] dSamplingInterval 歩道中心線のサンプリング間隔m
     * @param[in] dDistTh           横断歩道橋と歩道を繋ぐエッジの距離閾値m
    */
    void FootpathConnectionByPedestrianBridge(
        const double dSamplingInterval = 1.0,
        const double dDistTh = 10.0);

    /*!
     * @brief 車道ネットワークの標高値設定
    */
    void SetRoadwayHeight();

    /*!
     * @brief 歩道ネットワークの標高値設定
    */
    void SetFootpathHeight();

    /*!
     * @brief 歩道ネットワークデータの出力
     * @param[in] strShpOutputFolder        歩道SHP出力フォルダパス
     * @param[in] strGeoJsonOutputFolder    歩道GeoJSON出力フォルダパス
     * @param[in] nJPZone                   平面直角座標系の系番号
     * @param[in] bCreateSHP                SHP出力有無
     * @param[in] bCreateGeoJSON            GeoJSON出力有無
     * @param[in] dLod3Detail               LOD3の場合の詳細度
     * @return 処理結果
     * @retval true     成功
     * @retval false    失敗
     */
    bool OutputFootpathNetwork(
        const std::string &strShpOutputFolder,
        const std::string &strGeoJsonOutputFolder,
        const int nJPZone,
        const bool bCreateSHP,
        const bool bCreateGeoJSON,
        const double dLod3Detail = 3.0);
private:
    std::vector<std::shared_ptr<CTranRoadData>>     m_tranRoadData;     // 交通CityGML群
    int                                             m_iLod;             // 処理対象LOD
    double                                          m_dLodType;         // LODの詳細区分
    std::vector<std::shared_ptr<CBridgeData>>       m_bridgeData;       // 橋梁CityGML群
    std::vector<std::shared_ptr<CFurnitureData>>    m_pedestrianCrossingData;   // 都市設備(横断歩道)CityGML群
    std::vector<std::shared_ptr<CFurnitureData>>    m_brailleBlocksData;        // 都市設備(点字ブロック)CityGML群

    /*!
     * @brief 近傍歩道中心線の探索用データ
    */
    typedef std::tuple<
        Boost3DPointHash,
        std::shared_ptr<CTranRoadData>,
        std::shared_ptr<CCenterLineData>,
        Boost3DHashPolyline::iterator, bool> CenterLineTuple;

    /*!
     * @brief 近傍歩道中心線探索用RTree定義
    */
    typedef bg::index::rtree<CenterLineTuple, bg::index::quadratic<16>> SearchCenterLineRTree;

    /*!
     * @brief 近傍横断歩道の探索用データ
    */
    typedef std::tuple<Boost3DPointHash, std::shared_ptr<CFurnitureData>, bool> PCCenterLineTuple;
    /*!
     * @brief 近傍横断歩道の探索用RTree定義
    */
    typedef bg::index::rtree<PCCenterLineTuple, bg::index::quadratic<16>> SearchPCCenterLineRTree;

    /*!
     * @brief 最近傍歩道ポリゴンの探索
     * @param[in] roadPtr           注目道路ポインタ
     * @param[in/out] vec           最近傍探索対象のベクトル
     * @param[in/out] pt            ベクトルの始点
     * @param[in/out] dNNFrontDist  ベクトルの始点と最近傍歩道との距離
     * @param[out] nnFrontRoadPtr   ベクトルの始点と最近傍な歩道を含む道路のポインタ
     * @param[in/out] dNNBackDist   ベクトルの終点と最近傍歩道との距離
     * @param[out] nnBackRoadPtr    ベクトルの終点と最近傍な歩道を含む道路のポインタ
     * @return  処理結果
     * @retval  true    近傍歩道との距離と道路ポインタを更新した
     * @retval  false   近傍歩道との距離と道路ポインタを更新していない
    */
    bool searchNNFootpathPolygon(
        const std::shared_ptr<CTranRoadData> &roadPtr,
        CVector2D &vec,
        CVector2D &pt,
        double &dNNFrontDist,
        std::shared_ptr<CTranRoadData> &nnFrontRoadPtr,
        double &dNNBackDist,
        std::shared_ptr<CTranRoadData> &nnBackRoadPtr);

    /*!
     * @brief 近傍歩道中心線内の近傍点探索用RTreeの作成
     * @param[in] roadPtr           道路ポインタ
     * @param[in/out] rtree         rtree
     * @param[in] dSamplingInterval 中心線サンプリング間隔m
    */
    void updateFootpathCenterLineRTree(
        SearchCenterLineRTree &rtree,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const double dSamplingInterval = 1.0);

    /*!
     * @brief ポリゴン外にポリラインがはみ出ているか確認する
     * @param[in] polyline  ポリライン
     * @param[in] polygons  マルチポリゴン
     * @param[in] dLengthTh 有効はみだし部分とみなすポリラインの長さしきい値
     * @return はみ出し部分のポリライン群(はみ出し部分がない場合は空マルチポリライン)
    */
    Boost3DHashMultiLines checkOutsideOfPolygon(
        const Boost3DHashPolyline &polyline,
        const Boost3DHashMultiPolygon &polygons,
        const double dLengthTh = 0.01);

    /*!
     * @brief ポリゴン外にポリラインがはみ出ているか確認する(車道用)
     * @param[in] polyline  中心線
     * @param[in] roadPtr   道路情報ポインタ
     * @param[in] dLengthTh 有効はみだし部分とみなすポリラインの長さしきい値
     * @return はみ出し部分のポリライン群(はみ出し部分がない場合は空マルチポリライン)
    */
    Boost3DHashMultiLines checkOutsideOfPolygonForRoadway(
        const Boost3DHashPolyline &polyline,
        const std::shared_ptr<CTranRoadData> &roadPtr,
        const double dLengthTh = 0.01);
};
