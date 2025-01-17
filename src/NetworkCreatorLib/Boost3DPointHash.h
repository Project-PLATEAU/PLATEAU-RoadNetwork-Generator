#pragma once

#include "BoostCommon.h"
#include "CEpsUtil.h"
#include "CUtil.h"
#include "CommonDef.h"

/*!
 * @brief ハッシュキーを持つBoost3DPointクラス
 * @note  座標値の小数点以下の有効桁数は3桁(平面直角座標系で0.001mの想定)
*/
class Boost3DPointHash : public Boost3DPoint
{
public:

    double dEpsilon;    // 頂点比較の許容誤差(2点間距離の絶対値)

    /*!
     * @brief ハッシュ値取得用
    */
    struct HashFunc
    {
        size_t operator ()(const Boost3DPointHash &data) const { return data.GetHash(); }
    };

    /*!
     * @brief ハッシュマップ用の比較定義
    */
    struct RoundEqualFunc
    {
        bool operator ()(const Boost3DPointHash &a, const Boost3DPointHash &b) const { return a.IsRoundEqual(b); }
    };

    /*!
     * @brief コンストラクタ
    */
    Boost3DPointHash(const double e = 0.03) : Boost3DPoint(), dEpsilon(e){};

    /*!
     * @brief コンストラクタ
    */
    Boost3DPointHash(const double &x, const double &y, const double &z, const double e = 0.03)
        : Boost3DPoint(x, y, z), dEpsilon(e) {};

    /*!
     * @brief コンストラクタ
    */
    Boost3DPointHash(const Boost3DPoint &x, const double e = 0.03) : Boost3DPoint(x), dEpsilon(e) {};

    /*!
     * @brief デストラクタ
    */
    virtual ~Boost3DPointHash() {};

    /*!
     * @brief コピーコンストラクタ
    */
    Boost3DPointHash(const Boost3DPointHash &x) { *this = x; }

    /*!
     * @brief 代入演算子
    */
    Boost3DPointHash &operator = (const Boost3DPointHash &pt)
    {
        if (this != &pt)
        {
            this->x(pt.x());
            this->y(pt.y());
            this->z(pt.z());
            this->dEpsilon = pt.dEpsilon;
        }
        return *this;
    }

    /*!
     * @brief 等号比較関数
     * @param other 比較対称
     * @return 比較結果
     * @retval  true    一致
     * @retval  false   不一致
    */
    bool operator == (const Boost3DPointHash &other) const
    {
        bool bX = CEpsUtil::Equal(this->x(), other.x());
        bool bY = CEpsUtil::Equal(this->y(), other.y());
        bool bZ = CEpsUtil::Equal(this->z(), other.z());
        return bX && bY && bZ;
    }

    /*!
     * @brief 不等号比較関数
     * @param other 比較対象
     * @return 比較結果
     * @retval  true    不一致
     * @retval  false   一致
     */
    bool operator != (const Boost3DPointHash &other) const
    {
        return !(*this == other);
    }

    /*!
     * @brief 比較関数
     * @param other 比較対象
     * @return 比較結果
     * @retval  true    入力パラメータよりも小さい
     * @retval  false   入力パラメータよりも大きいor等しい
    */
    bool operator < (const Boost3DPointHash &other) const
    {
        if (CEpsUtil::Less(this->x(), other.x()))
            return true;
        else if (CEpsUtil::Greater(this->x(), other.x()))
            return false;

        if (CEpsUtil::Less(this->y(), other.y()))
            return true;
        else if (CEpsUtil::Greater(this->y(), other.y()))
            return false;

        if (CEpsUtil::Less(this->z(), other.z()))
            return true;
        else if (CEpsUtil::Greater(this->z(), other.z()))
            return false;

        return false;
    }

    /*!
     * @brief 誤差を考慮した場合の等号比較
     * @param other 比較対象
     * @return 比較結果
     * @retval  true    一致
     * @retval  false   不一致
    */
    bool IsRoundEqual(const Boost3DPointHash &other) const
    {
        double dDist = RoundDistance(other);
        return CEpsUtil::LessEqual(dDist, dEpsilon);
    }

    /*!
     * @brief ハッシュコード取得
     * @return ハッシュコード
    */
    size_t GetHash() const
    {
        Boost3DPoint pt = CUtil::RoundNPoint(*this, ncl_common_def::POINT_SIGNIFICANT_DIGITS);

        return std::hash<double>()(pt.x())
            ^ std::hash<double>()(pt.y())
            ^ std::hash<double>()(pt.z());
    }

    /*!
     * @brief 小数点以下の有効桁数を3桁とした場合の2点間距離
     * @param other 対象点
     * @return  距離
    */
    double RoundDistance(const Boost3DPointHash &other) const;
};

typedef bg::model::box<Boost3DPointHash> Boost3DHashBox;                        // ハッシュ付き3D座標点を使用するBox型
typedef bg::model::multi_point<Boost3DPointHash> Boost3DMultiPointHashs;        // ハッシュ付き3D複数点型
typedef bg::model::linestring<Boost3DPointHash> Boost3DHashPolyline;            // ハッシュ付き3D点のポリライン
typedef bg::model::multi_linestring<Boost3DHashPolyline> Boost3DHashMultiLines; // ハッシュ付き3D点の複数ライン型
typedef bg::model::ring<Boost3DPointHash, false, true> Boost3DHashRing;         // ハッシュ付き3Dリング型
typedef bg::model::polygon<Boost3DPointHash, false, true> Boost3DHashPolygon;   // ハッシュ付きboostの3Dポリゴン
typedef bg::model::multi_polygon<Boost3DHashPolygon> Boost3DHashMultiPolygon;   // ハッシュ付きboostの複数3Dポリゴン
// typedef bg::model::polygon<point, clock_wise, closed> polygon;
// clocke_wise = true(表:時計回り), false(表:反時計回り)
// clocke_wise = false(表:反時計回り)
// closed = true(終点に始点と同一点が挿入してポリゴンを閉じる必要がある)
// closed = false(終点に始点と同一点が挿入しなくても良い)

///
/// Boost.Geometryにカスタムクラス（Boost3DPointHash）を認識させる
///
namespace boost
{
    namespace geometry
    {
        namespace traits
        {
            template<>
            struct tag<Boost3DPointHash>
            {
                typedef point_tag type;
            };

            template<>
            struct coordinate_type<Boost3DPointHash>
            {
                typedef double type;
            };

            template<>
            struct coordinate_system<Boost3DPointHash>
            {
                typedef cs::cartesian type;
            };

            template<>
            struct dimension<Boost3DPointHash> : boost::mpl::int_<3> {};

            template<>
            struct access<Boost3DPointHash, 0>
            {
                static double get(Boost3DPointHash const& p)
                {
                    return p.x();
                }

                static void set(Boost3DPointHash& p, double const& value)
                {
                    p.x(value);
                }
            };

            template<>
            struct access<Boost3DPointHash, 1>
            {
                static double get(Boost3DPointHash const& p)
                {
                    return p.y();
                }

                static void set(Boost3DPointHash& p, double const& value)
                {
                    p.y(value);
                }
            };

            template<>
            struct access<Boost3DPointHash, 2>
            {
                static double get(Boost3DPointHash const& p)
                {
                    return p.z();
                }

                static void set(Boost3DPointHash& p, double const& value)
                {
                    p.z(value);
                }
            };
        }
    }
}