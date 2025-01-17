#pragma once

#define SPDLOG_WCHAR_TO_UTF8_SUPPORT

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_sinks.h"
#include <string>

/*!
 * @brief ログクラス(spdlogラッパー)
*/
class CLogger
{
private:
    static CLogger m_instance;      // 自クラス唯一のインスタンス

    /*!
     * @brief コンストラクタ
    */
    CLogger();

    /*!
     * @brief デストラクタ
    */
    virtual ~CLogger();

public:
    /*!
     * @brief インスタンスの取得
     * @return ロガーインスタンス
    */
    static CLogger* GetInstance() { return &m_instance; }

    /*!
     * @brief 初期化
     * @param strLogFilePath    ログファイルパス
     * @param strLogger         ロガー名
     * @param logLevel          ログレベル
     * @param bStdOut           ログの標準出力書き出しフラグ
     * @param strFormat         書式
    */
    void Initialize(
        const std::string &strLogFilePath,
        const std::string strLogger = "main",
        const spdlog::level::level_enum logLevel = spdlog::level::info,
        const bool bStdOut = false,
        const std::string strPattern = "");

    /*!
     * @brief ログ出力
     * @param strMsg    メッセージ
     * @param logLevel  ログレベル
     * @param strLogger ロガー名
    */
    void WriteLog(
        std::string strMsg,
        spdlog::level::level_enum logLevel = spdlog::level::info,
        std::string strLogger = "main");
    /*!
     * @brief 区切り線出力
     * @param logLevel  ログレベル
     * @param strLogger ロガー名
    */
    void WriteBorderLine(
        spdlog::level::level_enum logLevel = spdlog::level::info,
        std::string strLogger = "main");

    /*!
     * @brief ログ書き出しを終了する
     * @param strLogger     ロガー名
    */
    void Finish(const std::string strLogger = "main");
};

