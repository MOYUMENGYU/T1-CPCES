#ifndef RISK_ANALYSIS_HXX
#define RISK_ANALYSIS_HXX

#include <vector>

namespace Risk {

struct RiskScore {
    unsigned fluent;     // fluent id (正文字)
    float    score;      // 风险分数
    unsigned req_cnt;    // required_by 次数
    unsigned add_cnt;    // added_by 次数
    unsigned del_cnt;    // deleted_by 次数

    bool operator<( const RiskScore& rhs ) const {
        return score > rhs.score; // 降序
    }
};

// 计算风险指数并返回“按 score 降序排序”的列表
// topK: 输出Top-K（仅影响日志输出，不影响返回列表）
// alpha/beta/gamma: 三类使用次数的权重
std::vector<RiskScore> compute_risk_index( unsigned topK = 10,
                                          float alpha = 1.0f,
                                          float beta  = 0.2f,
                                          float gamma = 0.2f );

} // namespace Risk

#endif
