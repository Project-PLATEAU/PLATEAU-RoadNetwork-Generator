#pragma once

#include "CNetworkCreatorLib.h"
#include <string>

using namespace System;
using namespace System::Windows;

namespace NetworkCreatorWrapper
{
    public delegate void ActionCallback(double dRate, bool isInit);

    /*!
     * @brief ネットワークデータ作成ラッパークラス
    */
    public ref class CNetworkCreatorWrapper
    {
    private:
        CNetworkCreatorLib *m_pCNCLib = nullptr;    // ネットワークデータ作成ライブラリのポインタ
        ActionCallback ^m_pCallback;                // UI(C#)から渡されるDelegate
        ActionCallback ^m_innerCallback;            // 関数ポインタ変換用Delegate

        /*!
         * @brief 進捗表示用コールバック関数
         * @param dRate 進捗率
         * @param isInit    進捗率を入力値で初期化するか否か
        */
        void callback(double dRate, bool isInit);

        /*!
         * @brief wchar->std::string変換
         * @param pInput 入力文字列(wcharのポインタ)
         * @return  std::stringの文字列
         * @note    入力文字数制限(1024)あり
        */
        std::string wcharToString(const wchar_t *pInput);

    public:
        /*!
         * @brief コンストラクタ
         * @param callback 進捗表示用コールバック関数
        */
        CNetworkCreatorWrapper(ActionCallback ^callback);

        /*!
         * @brief デストラクタ
        */
        ~CNetworkCreatorWrapper();
        /*!
         * @brief ファイナライザ
        */
        !CNetworkCreatorWrapper();


        bool CreateNetwork(
            String ^strInputFolderPath,
            String ^strOutputFolderPath,
            int nLod,
            int nJPZone,
            bool bSHP,
            bool bGeoJSON);


    };

}


