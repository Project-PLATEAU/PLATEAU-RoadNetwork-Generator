#pragma once
#include "opencv2/opencv.hpp"
#include "Boost3DPointHash.h"
#include "boost/graph/adjacency_list.hpp"
#include <unordered_map>

/*!
 * @brief OpenCVを利用した画像処理のユーティリティクラス
*/
class COpenCVUtil
{
public:

    /*!
     * @brief 細線化処理
     * @param[in] polygon       ポリゴン
     * @param[in] dResolution   解像度
     * @param[in] dLineLengthTh 線の長さしきい値
     * @param[in] isRound       丸め座標の使用可否
     * @param[in] nDigit        丸め座標使用時の小数点以下の桁数
     * @return
    */
    static Boost3DHashMultiLines Thinning(
        const Boost3DHashPolygon &polygon,
        const double dResolution = 0.1,
        const bool isRound = false,
        const int nDigit = ncl_common_def::POINT_SIGNIFICANT_DIGITS);

    /*!
     * @brief 細線化処理(細線化後に直線化とひげ除去を行う)
     * @param[out] connections   接続状況
     * @param[in]  polygon       ポリゴン
     * @param[in]  dResolution   解像度
     * @param[in]  dDPTh         直線化時の線の長さしきい値
     * @param[in]  dLengthTh     ひげ除去用の長さしきい値
     * @param[in]  isRound       丸め座標の使用可否
     * @param[in]  nDigit        丸め座標使用時の小数点以下の桁数
     * @return
    */
    static std::vector<std::tuple<Boost3DHashPolyline, bool, bool>> Thinning(
        const Boost3DHashPolygon &polygon,
        const double dResolution = 0.1,
        const double dDPTh = 0.3,
        const double dLengthTh = 3.0,
        const bool isRound = false,
        const int nDigit = ncl_common_def::POINT_SIGNIFICANT_DIGITS);

    /*!
     * @brief ラスタ画像サイズの算出
     * @param[out] nWidth       画像幅
     * @param[out] nHeight      画像高さ
     * @param[out] dBaseX       基準地理x座標
     * @param[out] dBaseY       基準地理y座標
     * @param[out] dResoX       x座標の解像度
     * @param[out] dResoY       y座標の解像度
     * @param[in]  polygon      ポリゴン
     * @param[in]  dResolution  解像度
     * @param[in]  nMargin      余白サイズ
    */
    static void GetRasterImgSize(
        int &nWidth,
        int &nHeight,
        double &dBaseX,
        double &dBaseY,
        double &dResoX,
        double &dResoY,
        const Boost3DHashPolygon &polygon,
        const double dResolution = 0.1,
        const int nMargin = 1);

    /*!
     * @brief 画像座標->地理座標変換
     * @param[out] dX       地理x座標
     * @param[out] dX       地理y座標
     * @param[in]  nX       画像x座標
     * @param[in]  nY       画像y座標
     * @param[in]  dBaseX   基準地理x座標
     * @param[in]  dBaseY   基準地理y座標
     * @param[in]  dResoX   x座標の解像度
     * @param[in]  dResoY   y座標の解像度
     */
    static void GetWorldPos(
        double &dX,
        double &dY,
        const int nX,
        const int nY,
        const double dBaseX,
        const double dBaseY,
        const double dResoX,
        const double dResoY);

    /*!
     * @brief 地理座標->画像座標変換
     * @param[out] nX       画像x座標
     * @param[out] nY       画像y座標
     * @param[in]  dX       地理x座標
     * @param[in]  dY       地理y座標
     * @param[in]  nBaseX   基準地理x座標
     * @param[in]  nBaseY   基準地理y座標
     * @param[in]  dResoX   x座標の解像度
     * @param[in]  dResoY   y座標の解像度
    */
    static void GetImgPos(
        int &nX,
        int &nY,
        const double dX,
        const double dY,
        const double dBaseX,
        const double dBaseY,
        const double dResoX,
        const double dResoY);

private:
#pragma region ベクタ変換用グラフ定義
    struct VertexProperty; // 頂点プロパティ宣言
    struct EdgeProperty;   // エッジプロパティ宣言

    /*!
     * @brief   無向グラフ定義
     * @note    隣接構造のコンテナ定義(vecS = std::vector)
                頂点集合のコンテナ定義
                有向・無向指定
                頂点プロパティ指定
                エッジプロパティ指定
    */
    typedef boost::adjacency_list<
        boost::vecS, boost::vecS, boost::undirectedS,
        VertexProperty, EdgeProperty> Graph;

    typedef boost::graph_traits<Graph>::vertex_descriptor VertexDesc;   // 頂点ディスクリプタ定義
    typedef boost::graph_traits<Graph>::edge_descriptor EdgeDesc;       // エッジディスクリプタ定義

   /*!
    * @brief Boost3DPointHashデータをキーとするハッシュマップ(同一頂点の探索用)
    */
    typedef std::unordered_map<
        Boost3DPointHash, VertexDesc, Boost3DPointHash::HashFunc,
        Boost3DPointHash::RoundEqualFunc> VertexMap;

    /*!
     * @brief 頂点プロパティ定義
    */
    struct VertexProperty
    {
        VertexDesc desc;        // デスクリプター
        Boost3DPointHash pt;    // 座標
        bool isSearched;        // 探索済みフラグ

        /*!
         * @brief コンストラクタ
        */
        VertexProperty()
        {
            desc = 0;
            pt.x(0.0);
            pt.y(0.0);
            pt.z(0.0);
            isSearched = false;
        }
        /*!
         * @brief コンストラクタ
         * @param[in] p 座標
        */
        VertexProperty(const Boost3DPointHash &p)
        {
            desc = 0;
            pt = p;
            isSearched = false;
        }

        /*!
         * @brief コピーコンストラクタ
        */
        VertexProperty(const VertexProperty &p) { *this = p; }

        /*!
         * @brief 代入演算子
        */
        VertexProperty &operator =(const VertexProperty &p)
        {
            if (&p != this)
            {
                desc = p.desc;
                pt = p.pt;
                isSearched = p.isSearched;
            }
            return *this;
        }
    };

    /*!
     * @brief グラフのエッジプロパティ定義
    */
    struct EdgeProperty
    {
        VertexDesc vertexDesc1; // 頂点ディスクリプタ―1
        VertexDesc vertexDesc2; // 頂点ディスクリプタ―2
        double dLength;         // エッジの長さ(要手動更新)
        bool isSearched;        // 探索済みフラグ

        /*!
         * @brief コンストラクタ
        */
        EdgeProperty()
        {
            vertexDesc1 = 0;
            vertexDesc2 = 0;
            dLength = 0;
            isSearched = false;
        }

        /*!
         * @brief コンストラクタ
         * @param[in] desc1         頂点デスクリプター
         * @param[in] desc2         頂点デスクリプター
         * @param[in] length        エッジの長さ
        */
        EdgeProperty(
            VertexDesc desc1, VertexDesc desc2,
            double length = 0)
        {
            vertexDesc1 = desc1;
            vertexDesc2 = desc2;
            dLength = length;
            isSearched = false;
        }

        /*!
         * @brief コピーコンストラクタ
        */
        EdgeProperty(const EdgeProperty &e) { *this = e; }

        /*!
         * @brief 代入演算子
        */
        EdgeProperty &operator =(const EdgeProperty &e)
        {
            if (&e != this)
            {
                vertexDesc1 = e.vertexDesc1;
                vertexDesc2 = e.vertexDesc2;
                dLength = e.dLength;
                isSearched = e.isSearched;
            }
            return *this;
        }
    };
#pragma endregion

    /*!
     * @brief 線画像のベクタデータ変換
     * @param[in] img       画像
     * @param[in] nBaseX    基準地理x座標
     * @param[in] nBaseY    基準地理y座標
     * @param[in] dResoX    x座標の解像度
     * @param[in] dResoY    y座標の解像度
     * @return マルチポリライン
    */
    static Boost3DHashMultiLines toVectorLine(
        const cv::Mat &img,
        const double dBaseX,
        const double dBaseY,
        const double dResoX,
        const double dResoY);

    /*!
     * @brief グラフ内の端点と分岐点の探索
     * @param[in]  graph    グラフ
     * @param[out] endPts   端点群
     * @param[out] branch   分岐点群
    */
    static void searchVertex(
        const Graph &graph,
        std::set<VertexDesc> &endPts,
        std::unordered_map<VertexDesc, size_t> &branch);
    /*!
     * @brief グラフ内の分岐点のマージ
     * @param[in/out]  graph    グラフ
     * @param[in/out]  map      グラフ内頂点探索用マップ
     * @param[in]      dDistTh  経路長しきい値
    */
    static void mergeBranch(
        Graph &graph,
        VertexMap &map,
        const double dDistTh = 0.2);
    /*!
     * @brief 経路探索
     * @param[in]  graph    グラフ
     * @param[in]  v        注目頂点
     * @param[in]  ends     経路探索対象の終点群
     * @param[out] routes   経路(パスと経路長)
     * @param[out] vertices 経路が存在した終点
    */
    static void shortestPath(
        const Graph &graph,
        const VertexDesc &v,
        const std::set<VertexDesc> &ends,
        std::vector<std::pair<std::vector<VertexDesc>, double>> &routes,
        std::vector<VertexDesc> &vertices);
};

