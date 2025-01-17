#include "COpenCVUtil.h"
#include "CGDALUtil.h"
#include "CEpsUtil.h"
#include "boost/format.hpp"
#include "boost/foreach.hpp"
#include "boost/graph/dijkstra_shortest_paths.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/opencv.hpp"
#include <vector>
#include <set>

// 細線化
Boost3DHashMultiLines COpenCVUtil::Thinning(
    const Boost3DHashPolygon &polygon,
    const double dResolution,
    const bool isRound,
    const int nDigit)
{
    Boost3DHashMultiLines dstLines;

    const int nMargin = 5;    // 余白px

    // ラスタ変換
    double dBaseX, dBaseY, dResoX, dResoY;
    cv::Mat srcImg = CGDALUtil::GetInstance()->Rasterize(dBaseX, dBaseY, dResoX, dResoY, polygon, dResolution);

    // 余白を付与
    cv::Mat marginImg;
    cv::copyMakeBorder(
        srcImg, marginImg, nMargin, nMargin, nMargin, nMargin, cv::BORDER_CONSTANT, cv::Scalar(0));

    // 細線化
    cv::Mat thinImg;
    cv::ximgproc::thinning(marginImg, thinImg, cv::ximgproc::THINNING_GUOHALL);
    // 余白を削除
    cv::Mat clipImg = cv::Mat(thinImg, cv::Rect(nMargin, nMargin, srcImg.cols, srcImg.rows));

    // ベクタ変換
    Boost3DHashMultiLines tmpLines = toVectorLine(clipImg, dBaseX, dBaseY, dResoX, dResoY);
    for (const auto &line : tmpLines)
    {
        if (bg::covered_by(line, polygon))
            dstLines.push_back(line);
    }

    return dstLines;
}

// 細線化処理(細線化後に直線化とひげ除去を行う)
std::vector<std::tuple<Boost3DHashPolyline, bool, bool>> COpenCVUtil::Thinning(
    const Boost3DHashPolygon &polygon,
    const double dResolution,
    const double dDPTh,
    const double dLengthTh,
    const bool isRound,
    const int nDigit)
{
    Boost3DHashMultiLines dstLines, tmpLines;

    // 細線化処理
    tmpLines = Thinning(polygon, dResolution, isRound, nDigit);

    // 直線化とひげの除去
    if (!bg::is_empty(tmpLines))
    {
        // ひげ除去用のグラフ関連
        Graph graph;
        VertexMap map;

        // 簡略化とひげ除去
        Boost3DHashMultiLines simpleLines;
        for (const auto &line : tmpLines)
        {
            Boost3DHashPolyline simpleLine;
            bg::simplify(line, simpleLine, dDPTh);
            simpleLines.push_back(simpleLine);

            // ひげ除去用のグラフ作成
            auto vertex1 = (map.find(simpleLine.front()) == map.end()) ? Graph::null_vertex() : map.find(simpleLine.front())->second;
            auto vertex2 = (map.find(simpleLine.back()) == map.end()) ? Graph::null_vertex() : map.find(simpleLine.back())->second;
            if (vertex1 == Graph::null_vertex())
            {
                // グラフに頂点追加
                VertexProperty v(simpleLine.front());
                vertex1 = boost::add_vertex(v, graph);
                graph[vertex1].desc = vertex1;
                map.insert(VertexMap::value_type(simpleLine.front(), vertex1));    // マップ登録
            }
            if (vertex2 == Graph::null_vertex())
            {
                // グラフに頂点追加
                VertexProperty v(simpleLine.back());
                vertex2 = boost::add_vertex(v, graph);
                graph[vertex2].desc = vertex2;
                map.insert(VertexMap::value_type(simpleLine.back(), vertex2));    // マップ登録
            }

            auto edge = boost::add_edge(vertex1, vertex2, graph);
            graph[edge.first].vertexDesc1 = vertex1;
            graph[edge.first].vertexDesc2 = vertex2;
        }

        // ひげ除去
        for (const auto &line : simpleLines)
        {
            auto vertex1 = (map.find(line.front()) == map.end()) ? Graph::null_vertex() : map.find(line.front())->second;
            auto vertex2 = (map.find(line.back()) == map.end()) ? Graph::null_vertex() : map.find(line.back())->second;

            if (vertex1 != Graph::null_vertex() && vertex2 != Graph::null_vertex())
            {
                size_t degree1 = boost::degree(vertex1, graph);
                size_t degree2 = boost::degree(vertex2, graph);

                if ((degree1 == 1 && degree2 > 2) || (degree1 > 2 && degree2 == 1))
                {
                    // 端点から分岐点までのポリラインを削除候補とする
                    if (CEpsUtil::Less(bg::length(line), dLengthTh))
                    {
                        continue;   // ひげを除去
                    }
                }

                dstLines.push_back(line);
            }
        }
    }

    // ひげ除去済みの状態で分岐点位置でポリラインを分割し接続状況リストを作成する
    // ポリラインの始終点が端点か分岐点であるか判断する用
    std::vector<std::tuple<Boost3DHashPolyline, bool, bool>> dstData;
    if (dstLines.size() > 0)
    {
        Graph graph;
        VertexMap map;
        for (const auto &line : dstLines)
        {
            // グラフ作成
            VertexDesc prevVertex = Graph::null_vertex();
            for (const auto &pt : line)
            {
                VertexDesc currentVertex = Graph::null_vertex();
                auto it = map.find(pt);
                if (it == map.end())
                {
                    // グラフに頂点追加
                    VertexProperty v(pt);
                    currentVertex = boost::add_vertex(v, graph);
                    graph[currentVertex].desc = currentVertex;
                    map.insert(VertexMap::value_type(pt, currentVertex));    // マップ登録
                }
                else
                {
                    currentVertex = it->second;
                }

                if (prevVertex != Graph::null_vertex())
                {
                    auto ret1 = boost::edge(prevVertex, currentVertex, graph);
                    auto ret2 = boost::edge(currentVertex, prevVertex, graph);
                    if (!ret1.second && !ret2.second)
                    {
                        // エッジ追加
                        auto edge = boost::add_edge(prevVertex, currentVertex, graph);
                        graph[edge.first].vertexDesc1 = prevVertex;
                        graph[edge.first].vertexDesc2 = currentVertex;
                        graph[edge.first].dLength = pt.RoundDistance(graph[prevVertex].pt);
                    }
                }
                prevVertex = currentVertex; // 更新
            }
        }

        // 経路探索
        // 端点と分岐点探索
        std::unordered_map<VertexDesc, size_t> branch;
        std::set<VertexDesc> endPoints;
        searchVertex(graph, endPoints, branch);
        // 端点からポリラインを分割していく
        while (endPoints.size() > 0)
        {
            for (const auto &vertexDesc : endPoints)
            {
                if (!graph[vertexDesc].isSearched)
                {
                    // 未探索の場合
                    Boost3DHashPolyline polyline;
                    VertexDesc targetDesc = vertexDesc; // 注目ノード
                    auto itBranch = branch.end();
                    auto itEndPoint = endPoints.end();
                    do
                    {
                        polyline.push_back(graph[targetDesc].pt);
                        graph[targetDesc].isSearched = true;    // 探索済み
                        BOOST_FOREACH(const auto & desc, boost::out_edges(targetDesc, graph))
                        {
                            if (!graph[desc].isSearched)
                            {
                                targetDesc = (graph[desc].vertexDesc1 != targetDesc) ? graph[desc].vertexDesc1 : graph[desc].vertexDesc2;
                                graph[desc].isSearched = true;
                                break;
                            }
                        }
                        itBranch = branch.find(targetDesc);
                        itEndPoint = endPoints.find(targetDesc);
                    } while (itBranch == branch.end() && itEndPoint == endPoints.end());
                    polyline.push_back(graph[targetDesc].pt);

                    size_t degree1 = boost::degree(vertexDesc, graph);  // 始点の接続エッジ数
                    size_t degree2 = boost::degree(targetDesc, graph);  // 終点の接続エッジ数
                    bool bFront = (degree1 == 1) ? false : true;    // true : 連結点, false : 端点
                    bool bBack = (degree2 == 1) ? false : true;
                    dstData.push_back({ polyline, bFront, bBack });


                    if (itBranch != branch.end())
                    {
                        itBranch->second -= 1;  // 分岐数減
                    }
                    if (itEndPoint != endPoints.end())
                    {
                        graph[*itEndPoint].isSearched = true;   // 探索済み
                    }
                }
            }

            // 探索対象の更新
            endPoints.clear();
            std::unordered_map<VertexDesc, size_t> newBranch;
            for (const auto &val : branch)
            {
                if (val.second >= 2)
                {
                    newBranch.insert(val);
                }
                else if (val.second == 1)
                {
                    endPoints.insert(val.first);
                }
            }
            branch = newBranch;
        }
    }

    return dstData;
}

// ラスタ画像サイズの算出
void COpenCVUtil::GetRasterImgSize(
    int &nWidth,
    int &nHeight,
    double &dBaseX,
    double &dBaseY,
    double &dResoX,
    double &dResoY,
    const Boost3DHashPolygon &polygon,
    const double dResolution,
    const int nMargin)
{
    Boost3DHashBox box;
    bg::envelope(polygon, box);
    dResoX = dResolution;
    dResoY = -dResolution;
    dBaseX = box.min_corner().x() - dResoX * nMargin;
    dBaseY = box.max_corner().y() - dResoY * nMargin;
    nWidth = static_cast<int>((box.max_corner().x() - box.min_corner().x()) / dResolution) + 1 + 2 * nMargin;
    nHeight = static_cast<int>((box.max_corner().y() - box.min_corner().y()) / dResolution) + 1 + 2 * nMargin;
}

// ラスタ座標->ベクタ座標変換
void COpenCVUtil::GetWorldPos(
    double &dX,
    double &dY,
    const int nX,
    const int nY,
    const double dBaseX,
    const double dBaseY,
    const double dResoX,
    const double dResoY)
{
    // ピクセル中心座標になるようにdReso * 0.5を追加
    dX = dResoX * static_cast<double>(nX) + dBaseX + (dResoX * 0.5);
    dY = dResoY * static_cast<double>(nY) + dBaseY + (dResoY * 0.5);
}

// 地理座標->画像座標変換
void COpenCVUtil::GetImgPos(
    int &nX,
    int &nY,
    const double dX,
    const double dY,
    const double dBaseX,
    const double dBaseY,
    const double dResoX,
    const double dResoY)
{
    nX = static_cast<int>((dX - dBaseX) / dResoX);
    nY = static_cast<int>((dY - dBaseY) / dResoY);
}

// 線画像のベクタデータ変換
Boost3DHashMultiLines COpenCVUtil::toVectorLine(
    const cv::Mat &img,
    const double dBaseX,
    const double dBaseY,
    const double dResoX,
    const double dResoY)
{
    Boost3DHashMultiLines polylines;

    // 輪郭線抽出
    std::vector<std::vector<cv::Point2i>> contours;
    cv::findContours(img, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

    // グラフ作成
    VertexMap map;
    Graph graph;

    for (const auto &countor : contours)
    {
        VertexDesc prevVertex = Graph::null_vertex();
        for (const auto &pt : countor)
        {
            VertexDesc currentVertex = Graph::null_vertex();
            double dX, dY;
            GetWorldPos(dX, dY, pt.x, pt.y, dBaseX, dBaseY, dResoX, dResoY);
            Boost3DPointHash currentPt(dX, dY, 0);
            auto it = map.find(currentPt);
            if (it == map.end())
            {
                // グラフに頂点追加
                VertexProperty v(currentPt);
                currentVertex = boost::add_vertex(v, graph);
                graph[currentVertex].desc = currentVertex;
                map.insert(VertexMap::value_type(currentPt, currentVertex));    // マップ登録
            }
            else
            {
                currentVertex = it->second;
            }

            if (prevVertex != Graph::null_vertex())
            {
                auto ret1 = boost::edge(prevVertex, currentVertex, graph);
                auto ret2 = boost::edge(currentVertex, prevVertex, graph);
                if (!ret1.second && !ret2.second)
                {
                    // エッジ追加
                    auto edge = boost::add_edge(prevVertex, currentVertex, graph);
                    graph[edge.first].vertexDesc1 = prevVertex;
                    graph[edge.first].vertexDesc2 = currentVertex;
                    graph[edge.first].dLength = currentPt.RoundDistance(graph[prevVertex].pt);
                }
            }
            prevVertex = currentVertex; // 更新
        }
    }

    // 交点部分に閉路が発生している場合があるためマージする
    double dReso = CEpsUtil::Greater(dResoX, dResoY) ? dResoX : dResoY;
    mergeBranch(graph, map, dReso * 2.0);

    // 経路探索
    // 端点と分岐点探索
    std::unordered_map<VertexDesc, size_t> branch;
    std::set<VertexDesc> endPoints;
    searchVertex(graph, endPoints, branch);

    while (endPoints.size() > 0)
    {
        for (const auto &vertexDesc : endPoints)
        {
            if (!graph[vertexDesc].isSearched)
            {
                // 未探索の場合
                Boost3DHashPolyline polyline;
                VertexDesc targetDesc = vertexDesc; // 注目ノード
                auto itBranch = branch.end();
                auto itEndPoint = endPoints.end();
                do
                {
                    polyline.push_back(graph[targetDesc].pt);
                    graph[targetDesc].isSearched = true;    // 探索済み
                    BOOST_FOREACH(const auto &desc, boost::out_edges(targetDesc, graph))
                    {
                        if (!graph[desc].isSearched)
                        {
                            targetDesc = (graph[desc].vertexDesc1 != targetDesc) ? graph[desc].vertexDesc1 : graph[desc].vertexDesc2;
                            graph[desc].isSearched = true;
                            break;
                        }
                    }
                    itBranch = branch.find(targetDesc);
                    itEndPoint = endPoints.find(targetDesc);
                } while (itBranch == branch.end() && itEndPoint == endPoints.end());
                polyline.push_back(graph[targetDesc].pt);

                polylines.push_back(polyline);

                if (itBranch != branch.end())
                {
                    itBranch->second -= 1;  // 分岐数減
                }
                if (itEndPoint != endPoints.end())
                {
                    graph[*itEndPoint].isSearched = true;   // 探索済み
                }
            }
        }

        // 探索対象の更新
        endPoints.clear();
        std::unordered_map<VertexDesc, size_t> newBranch;
        for (const auto &val : branch)
        {
            if (val.second >= 2)
            {
                newBranch.insert(val);
            }
            else if (val.second == 1)
            {
                endPoints.insert(val.first);
            }
        }
        branch = newBranch;
    }

    return polylines;
}

 // グラフ内の端点と分岐点の探索
void COpenCVUtil::searchVertex(
    const Graph &graph,
    std::set<VertexDesc> &endPts,
    std::unordered_map<VertexDesc, size_t> &branch)
{
    BOOST_FOREACH(VertexDesc vertexDesc, boost::vertices(graph))
    {
        size_t degree = boost::degree(vertexDesc, graph);
        if (degree == 1)
        {
            // 端点
            endPts.insert(vertexDesc);
        }
        else if (degree > 2)
        {
            // 分岐点
            branch.insert(std::pair<VertexDesc, size_t>(vertexDesc, degree));
        }
    }
}

// 分岐点のマージ
void COpenCVUtil::mergeBranch(
    Graph &graph,
    VertexMap &map,
    const double dDistTh)
{
    // 分岐点の探索
    std::unordered_map<VertexDesc, size_t> branch;
    std::set<VertexDesc> endPoints;
    searchVertex(graph, endPoints, branch);

    std::set<VertexDesc> ends;
    for (const auto &v : branch)
        ends.insert(v.first);

    for (const auto &target : branch)
    {
        // マージ候補一覧
        std::set<VertexDesc> candidate;
        std::vector<std::vector<VertexDesc>> routes;

        // 経路探索
        std::vector<std::pair<std::vector<VertexDesc>, double>> tmpRoutes1;
        std::vector<VertexDesc> tmpVertices1;
        shortestPath(graph, target.first, ends, tmpRoutes1, tmpVertices1);
        for (size_t t = 0; t < tmpVertices1.size(); t++)
        {
            if (CEpsUtil::LessEqual(tmpRoutes1[t].second, dDistTh))
            {
                // 新規マージ対象点から既存のマージ対象点までの経路を探索
                std::vector<std::pair<std::vector<VertexDesc>, double>> tmpRoutes2;
                std::vector<VertexDesc> tmpVertices2;
                shortestPath(graph, tmpVertices1[t], candidate, tmpRoutes2, tmpVertices2);
                for (const auto &route : tmpRoutes2)
                {
                    routes.push_back(route.first);
                }

                candidate.insert(tmpVertices1[t]);      // マージ対象頂点
                routes.push_back(tmpRoutes1[t].first);  // マージ時に削除対象となるエッジを取得
            }
        }
        candidate.insert(target.first); // 注目点をマージ対象に追加

        if (candidate.size() > 2)
        {
            // 中点にマージ
            Boost3DMultiPointHashs pts;
            for (const auto &desc : candidate)
                pts.push_back(graph[desc].pt);
            Boost3DPointHash center;
            bg::centroid(pts, center);
            auto it = map.find(center);
            VertexDesc centerDesc;
            if (it == map.end())
            {
                // グラフに頂点追加
                VertexProperty v(center);
                centerDesc = boost::add_vertex(v, graph);
                graph[centerDesc].desc = centerDesc;
                map.insert(VertexMap::value_type(center, centerDesc));    // マップ登録
            }
            else
            {
                centerDesc = it->second;
            }

            // エッジの削除
            for (const auto &route : routes)
            {
                VertexDesc prev = Graph::null_vertex();
                for (const auto &current : route)
                {
                    if (prev != Graph::null_vertex())
                    {
                        boost::remove_edge(prev, current, graph);
                    }
                    prev = current;
                }
            }

            // マージ対象の頂点のエッジを修正
            for (const auto &v : candidate)
            {
                BOOST_FOREACH(EdgeDesc e, boost::out_edges(v, graph))
                {
                    auto otherDesc = (boost::target(e, graph) != v) ? boost::target(e, graph) : boost::source(e, graph);
                    auto newEdge = boost::add_edge(centerDesc, otherDesc, graph); // エッジ追加
                    if (newEdge.second)
                    {
                        graph[newEdge.first].vertexDesc1 = centerDesc;
                        graph[newEdge.first].vertexDesc2 = otherDesc;
                        graph[newEdge.first].dLength = graph[centerDesc].pt.RoundDistance(graph[otherDesc].pt);
                    }
                    boost::remove_edge(v, otherDesc, graph);    // エッジの削除
                }
            }
        }
    }
}

// 経路探索
void COpenCVUtil::shortestPath(
    const Graph &graph,
    const VertexDesc &v,
    const std::set<VertexDesc> &ends,
    std::vector<std::pair<std::vector<VertexDesc>, double>> &routes,
    std::vector<VertexDesc> &vertices)
{
    routes.clear();
    std::vector<VertexDesc> pred(boost::num_vertices(graph), Graph::null_vertex());
    std::vector<double> vecDistance(boost::num_vertices(graph));
    boost::dijkstra_shortest_paths(
        graph, v,
        boost::predecessor_map(pred.data()).
        distance_map(vecDistance.data()).
        weight_map(boost::get(&EdgeProperty::dLength, graph)));

    for (const auto &other : ends)
    {
        if (v != other && pred[other] != other)
        {
            // 最短経路が存在する場合
            std::vector<VertexDesc> route;
            for (VertexDesc tmpDesc = other;
                tmpDesc != v; tmpDesc = pred[tmpDesc])
            {
                route.push_back(tmpDesc);
            }
            route.push_back(v);
            routes.push_back(std::pair<std::vector<VertexDesc>, double>(route, vecDistance[other]));
            vertices.push_back(other);
        }
    }
}
