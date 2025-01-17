#pragma once
#include <string>
#include "citygml/citygml.h"
#include "citygml/citymodel.h"
#include "citygml/cityobject.h"
#include "citygml/geometry.h"
#include "citygml/implictgeometry.h"
#include "citygml/linestring.h"
#include "citygml/polygon.h"

// CityGML関連の定義

#pragma region CityObject共通

// enum classをループ処理するための定義
#define ENUM_ITTR(T) \
inline T operator++(T& x) {return x = (T)(std::underlying_type<T>::type(x) + 1); } \
inline T operator*(T x) { return x;} \
inline T begin(T x) { return static_cast<T>(0); } \
inline T end(T x) { return T::LAST; }

/*!
 * @brief LODタイプ
*/
enum class LOD_TYPE
{
    LOD1 = 1,
    LOD2,
    LOD3
};

#pragma endregion

#pragma region 道路

constexpr auto KEY_GML_NAME = "gml:name";           // 路線名
constexpr auto KEY_TRAN_CLASS = "luse:class";       // tran:class 1040 = 道路
constexpr auto KEY_TRAN_FUNCTION = "luse:function"; // tran:function

constexpr auto KEY_TRAN_STRUCTURE_ATTR = "uro:roadStructureAttribute";          // 道路構造属性
constexpr auto KEY_TRAN_STRUCTURE_ATTR_SECTION_TYPE = "uro:sectionType";        // 道路構造種別
constexpr auto KEY_TRAN_STRUCTURE_ATTR_WIDTH_TYPE = "uro:widthType";            // 幅員区分
constexpr auto KEY_TRAN_STRUCTURE_ATTR_WIDTH = "uro:width";                     // 幅員m
constexpr auto KEY_TRAN_STRUCTURE_ATTR_NUMBER_OF_LANES = "uro:numberOfLanes";   // 車線数

constexpr auto KEY_TRAN_DATA_QUALITY_ATTR = "uro:tranDataQualityAttribute"; // 道路品質属性
constexpr auto KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE = "uro:lodType"; // LODの詳細区分

constexpr auto KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE_30 = u8"道路の横断方向の高さは一律とし、車道の高さとする。車道、車道交差部、分離帯及び歩道を区分する。";
constexpr auto KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE_31 = u8"道路の横断方向の高さは一律とし、車道の高さとする。車道、車道交差部、分離帯及び歩道の区分に加え、車道を車線に区分する。";
constexpr auto KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE_32 = u8"道路の横断方向に存在する15㎝以上の高さの差を取得する。車道、車道交差部、分離帯及び歩道の区分に加え、車道を車線に区分し、歩道上の植栽を区分する。";
constexpr auto KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE_33 = u8"道路の横断方向に存在する2㎝以上の高さの差を取得する。車道、車道交差部、分離帯及び歩道の区分に加え、車道を車線に区分し、歩道上の植栽を区分する。";
constexpr auto KEY_TRAN_DATA_QUALITY_ATTR_LOD_TYPE_34 = u8"道路の横断方向に存在する2㎝以上の高さの差を取得する。車道、車道交差部、分離帯及び歩道の区分に加え、車道、分離帯、歩道を以下の区分に細分する。車道は、車線、すりつけ区間、踏切道、軌道敷、待避所、副道、自動車駐車場、非常駐車帯、中央帯、側帯、路肩、停車帯、乗合自動車停車所に区分する。分離帯は、交通島、分離帯、植樹帯、路面電車停車所に区分する。歩道は、歩道、自転車歩行者道、自転車道、植樹ますに区分する。";

/*!
 * @brief 交通領域(tran:TrafficArea)のtran:function種別
 * @note LOD2, LOD3.0 -> 車道部、車道交差部、歩道部(歩道上の植栽を含む)
 *       LOD3.1 -> 車道部、車線、車道交差部、歩道部(歩道上の植栽を含む)
 *       LOD3.2, LOD3.3 -> 車道部、車線、車道交差部、歩道部(歩道上の植栽を含まない)
*/
enum class TRAFFIC_AREA_FUNCTION_TYPE
{
    ROADWAY = 1000,                 // 車道部
    LANE = 1010,                    // 車線
    ROADWAY_INTERSECTION = 1020,    // 車道交差部
    FOOTPATH = 2000,                // 歩道部
    // 以下はループ処理を行えるようにするための値
    LAST,
};

/*!
 * @brief 交通補助領域(tran:AuxiliaryTrafficArea)のtran:function種別
 * @note LOD2, LOD3.0, lod3.1 -> 島
 *       LOD3.2, LOD3.3 -> 島、植栽
*/
enum class AUXILIARY_TRAFFIC_AREA_FUNCTION_TYPE
{
    ISLAND = 3000,  // 島
    PLANTS = 5000,  // 植栽
    // 以下はループ処理を行えるようにするための値
    LAST,
};

#pragma endregion

#pragma region 都市設備
constexpr auto KEY_FRN_CLASS = "frn:class";         // 都市設備区分
constexpr auto KEY_FRN_FUNCTION = "frn:function";   // 都市設備の種類

constexpr auto KEY_FRN_DATA_QUALITY_ATTR = "uro:cityFurnitureDataQualityAttribute"; // 橋梁品質属性
constexpr auto KEY_FRN_DATA_QUALITY_ATTR_LOD_TYPE = "uro:lodType"; // LOD3の詳細区分

/*!
 * @brief 都市設備の種類(frn:function)
*/
enum class FURNITURE_FUNCTION_TYPE
{
    UNKNOWN                                         = 0,        // 不明
    // 以下はCityGML仕様の設定値
    ROAD_MARKINGS                                   = 1000,     // 道路表示
    PARCEL_LINES                                    = 1010,     // 区画線
    ROADWAY_CENTER_LINES                            = 1020,     // 道路中央線
    LANE_BOUNDARY_LINES                             = 1030,     // 車線境界線
    LANE_OUTSIDE_LINES                              = 1040,     // 車道外側線
    SINGS                                           = 1100,     // 指示表示
    PEDESTRIAN_CROSSING                             = 1110,     // 横断歩道
    STOP_LINE                                       = 1120,     // 停止線
    REGULATORY_SIGNS                                = 1200,     // 規制表示
    FENCES_AND_WALLS                                = 2000,     // 柵・壁
    ROAD_SIGNS                                      = 3000,     // 道路標識
    GUIDE_SIGNS                                     = 3110,     // 案内標識
    WARNING_SIGNS                                   = 3120,     // 警戒標識
    REGULATION_SIGNS                                = 3130,     // 規制標識
    INSTRUCTION_SIGNS                               = 3140,     // 指示標識
    AUXILIARY_SIGNS                                 = 3150,     // 補助標識
    BUILDINGS                                       = 4000,     // 建造物
    SHED                                            = 4010,     // 上屋
    UNDERGROUND_ENTRANCE                            = 4020,     // 地下出入口
    ARCADE                                          = 4030,     // アーケード
    SIGHTING_SIGNS                                  = 4100,     // 視線誘導標
    ROAD_REFLECTOR                                  = 4120,     // 道路反射鏡
    LIGHTING_FACILITIES                             = 4200,     // 照明施設
    TOMBSTONE,                                                  // 墓碑
    MONUMENTS,                                                  // 記念碑
    STANDING_STATUE,                                            // 立像
    ROADSIDE_SHRINE,                                            // 路傍祠
    LANTERNS,                                                   // 灯ろう
    TORII                                           = 4207,     // 鳥居
    MONUMENT_FOR_THE_TRADITION_OF_NATURAL_DISASTERS,            // 自然災害伝承碑
    FOUNTAIN                                        = 4223,     // 噴水
    WELLS,                                                      // 井戸
    OIL_AND_GAS_WELLS,                                          // 油井・ガス井
    HOIST_EQUIPMENT                                 = 4228,     // 起重機
    TANKS                                           = 4231,     // タンク
    CHIMNEYS                                        = 4234,     // 煙突
    TALL_TOWERS,                                                // 高塔
    RADIO_TOWERS,                                               // 電波塔
    WINDMILLS                                       = 4239,     // 風車
    LIGHTHOUSES                                     = 4241,     // 灯台
    LIGHT_TOWERS                                    = 4243,     // 灯標
    HELIPORTS                                       = 4245,     // ヘリポート
    WATER_LEVEL_STATION                             = 4251,     // 水位観測所
    ROAD_INFO_MANAGEMENT_FACILITIES                 = 4300,     // 道路情報管
    DISASTER_DETECTORS                              = 4400,     // 災害検知器
    WEATHER_OBSERVATION_DEVICES                     = 4500,     // 気象観測装
    ROAD_INFO_BOARDS                                = 4600,     // 道路情報板
    OPTICAL_FIBER                                   = 4700,     // 光ファイバー
    POLE                                            = 4800,     // 柱
    ROADSIDE                                        = 4810,     // 路側
    CANTILEVERED                                    = 4820,     // 片持
    GANTRY                                          = 4830,     // 門型
    TELEPHONE_POLES                                 = 4840,     // 電柱
    TRAFFIC_SIGNALS                                 = 4900,     // 交通信号機
    STAIRWAY                                        = 5000,     // 階段
    PASSAGEWAY                                      = 5010,     // 通路
    ELEVATORS                                       = 5020,     // エレベータ
    ESCALATORS                                      = 5030,     // エスカレータ
    ADMINISTRATIVE_OVERGROUND_FACILITIES            = 5100,     // 管理用地上
    CABLE_ACCESS_DITCHES                            = 5200,     // 電線共同溝
    CAB                                             = 5300,     // CAB
    INFO_BOX                                        = 5400,     // 情報 BOX
    PIPE_LINE                                       = 5500,     // 管路
    ADMINISTRATIVE_OPENINGS                         = 5600,     // 管理用開口
    MANHOLE                                         = 5610,     // マンホール
    HANDHOLE                                        = 5620,     // ハンドホール
    ENTRANCE_HOLE                                   = 5630,     // 入孔
    DISTANCE_MARKERS                                = 6000,     // 距離標
    BOUNDARY_SIGNS                                  = 6010,     // 境界標識
    ROAD_MARKERS_AND_MILEAGE_MARKERS                = 6020,     // 道路元標・里程標
    TOLL_COLLECTION_FACILITIES                      = 6100,     // 料金徴収施
    SNOW_MELTING_FACILITIES                         = 6200,     // 融雪施設
    DRAINAGE_FACILITIES                             = 7000,     // 排水施設
    WATER_CATCH_BASINS                              = 7100,     // 集水桝
    DRAINAGE_DITCH                                  = 7200,     // 排水溝
    GUTTERS                                         = 7300,     // 側溝
    DRAINAGE_PIPES                                  = 7400,     // 排水管
    DRAINAGE_PUMP                                   = 7500,     // 排水ポンプ
    STOPS                                           = 8010,     // 停留所
    FIRE_HYDRANT                                    = 8020,     // 消火栓
    POST_BOX                                        = 8030,     // 郵便ポスト
    TELEPHONE_BOX                                   = 8040,     // 電話ボック
    TRANSPORT_PIPE                                  = 8050,     // 輸送管
    TRACK                                           = 8060,     // 軌道
    OVERHEAD_LINES                                  = 8070,     // 架空線
    VENDING_MACHINE                                 = 8080,     // 自動販売機
    BULLETIN_BOARDS                                 = 8140,     // 掲示板
    BRAILLE_BLOCKS                                  = 8150,     // 点字ブロック
    BENCHES                                         = 8160,     // ベンチ
    TABLES                                          = 8170,     // テーブル
    OTHERS                                          = 9000,     // その他
    SIGNBOARD,                                                  // 看板(自立式)
    WATER_FOUNTAIN,                                             // 水飲み
    // 以下はループ処理を行えるようにするための値
    LAST,
};

// enum class をループ処理するための準備
ENUM_ITTR(FURNITURE_FUNCTION_TYPE);

#pragma endregion

#pragma region 橋梁
constexpr auto KEY_BRID_FUNCTION = "luse:function"; // brid:function

constexpr auto KEY_BRID_DATA_QUALITY_ATTR = "uro:bridDataQualityAttribute"; // 橋梁品質属性
constexpr auto KEY_BRID_DATA_QUALITY_ATTR_LOD_TYPE = "uro:lodType"; // LODの詳細区分

constexpr auto KEY_BRID_DATA_QUALITY_ATTR_LOD_TYPE_20 = u8"道路橋、桟道橋及び鉄道橋は、床版の外周を、高さをもった面として表現する。横断歩道橋、ペデストリアンデッキ及び跨線橋は、本体（上部工、階段及び踊り場）の外周を取得し、高さをもった面として表現する。階段の個々の段は取得せず、下端と上端を結んだ平面として表現する。";
constexpr auto KEY_BRID_DATA_QUALITY_ATTR_LOD_TYPE_21 = u8"道路橋、桟道橋及び鉄道橋は、床版及び主桁によって、厚みと高さをもった立体として表現する。橋脚などの構造上不可欠な部材を表現してもよい。";

// 標高値を取得するための底面ポリゴン識別用の種別
//constexpr auto KEY_BRID_PART_TYPE_GROUND_SURFACE = "GroundSurface";                 // 橋梁の側面と地表との交線により囲まれた面
constexpr auto KEY_BRID_PART_TYPE_OUTER_FLOOR_SURFACE = "OuterFloorSurface";        // 床版の上方からの正射影の外周とする面
constexpr auto KEY_BRID_PART_TYPE_OUTER_CEILING_SURFACE = "OuterCeilingSurface";    // 屋外天井面

/*!
 * @brief 橋梁の主たる機能による区分(brid:function)の種別
*/
enum class BRIDGE_FUNCTION_TYPE
{
    UNKNOWN = 0,                    // 不明
    // 以下はCityGML仕様の設定値
    ROAD_BRIDGE = 1,                // 道路橋
    RAILROAD_BRIDGE,                // 鉄道橋
    AQUADUCT_BRIDGE,                // 水路橋
    CABLE_BRIDGE,                   // ケーブル橋
    BRIDGESIDE_PEDESTRIAN_BRIDGE,   // 橋側歩道橋
    CANAL_BRIDGE,                   // 運河橋
    PEDESTRIAN_CROSSING_BRIDGE,     // 横断歩道橋
    PEDESTRIAN_DECK,                // ペデストリアンデッキ
    // 以下はループ処理を行えるようにするための値
    LAST,
};
#pragma endregion
