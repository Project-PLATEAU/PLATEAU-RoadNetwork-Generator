#include "CLogger.h"
#include "StringEx.h"

CLogger CLogger::m_instance;    // インスタンス

/*!
 * @brief コンストラクタ
*/
CLogger::CLogger()
{

}

/*!
 * @brief デストラクタ
*/
CLogger::~CLogger()
{

}

/*!
 * @brief 初期化
 * @param strLogFilePath    ログファイルパス
 * @param strLogger         ロガー名
 * @param logLevel          ログレベル
 * @param bStdOut           ログの標準出力書き出しフラグ
 * @param strPattern        書式
*/
void CLogger::Initialize(
    const std::string &strLogFilePath,
    const std::string strLogger,
    const spdlog::level::level_enum logLevel,
    const bool bStdOut,
    const std::string strPattern)
{
    // 同一名のロガーが存在する場合は削除
    auto oldLogger = spdlog::get(strLogger);
    if (oldLogger != nullptr)
    {
        spdlog::drop(strLogger);
    }

    // log設定
    std::vector<spdlog::sink_ptr> sinks;

    // 標準出力
    if (bStdOut)
        sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_mt>());

    // ログファイル
    // truncate : true->新規作成, false->追加書き込み
    sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(strLogFilePath, true));
    auto logger = std::make_shared<spdlog::logger>(strLogger, begin(sinks), end(sinks));

    //register it if you need to access it globally
    spdlog::register_logger(logger);
    spdlog::set_level(logLevel);

    // 書式設定
    if (!strPattern.empty())
    {
        logger->set_pattern(strPattern);
    }
}

/*!
 * @brief ログ出力
 * @param strMsg    メッセージ
 * @param logLevel  ログレベル
 * @param strLogger ロガー名
*/
void CLogger::WriteLog(
    std::string strMsg,
    spdlog::level::level_enum logLevel,
    std::string strLogger)
{
    auto logger = spdlog::get(strLogger);
    if (logger != nullptr)
    {
        switch (logLevel)
        {
            case spdlog::level::trace:
                logger->trace(strMsg);
                break;
            case spdlog::level::debug:
                logger->debug(strMsg);
                break;
            case spdlog::level::warn:
                logger->warn(strMsg);
                break;
            case spdlog::level::err:
                logger->error(strMsg);
                break;
            case spdlog::level::critical:
                logger->critical(strMsg);
                break;
            case spdlog::level::info:
            default:
                logger->info(strMsg);
                break;
        }
        logger->flush();
    }
}

/*!
 * @brief 区切り線出力
 * @param logLevel  ログレベル
 * @param strLogger ロガー名
*/
void CLogger::WriteBorderLine(
    spdlog::level::level_enum logLevel,
    std::string strLogger)
{
    WriteLog("------------------------------", logLevel, strLogger);
}

/*!
 * @brief ログ書き出しを終了する
 * @param strLogger     ロガー名
*/
void CLogger::Finish(const std::string strLogger)
{
    // ロガーが存在する場合は削除
    auto logger = spdlog::get(strLogger);
    if (logger != nullptr)
    {
        spdlog::drop(strLogger);
    }
}
