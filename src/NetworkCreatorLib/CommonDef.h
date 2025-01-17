#pragma once

/*!
 * @brief 共通定義
*/
namespace ncl_common_def
{
    constexpr int POINT_SIGNIFICANT_DIGITS = 3; // 頂点座標の小数点以下の有効桁数(平面直角座標系において0.001mを想定)
    constexpr int POINT_SIGNIFICANT_DIGITS_FOR_DISSOLVE = 2; // ポリゴン融合用頂点座標の小数点以下の有効桁数(平面直角座標系において0.01mを想定)
};
