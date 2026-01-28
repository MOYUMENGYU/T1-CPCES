#include "risk_analysis.hxx"
#include "PDDL.hxx"

#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>

namespace Risk {

static inline float calc_score( unsigned req_cnt, unsigned add_cnt, unsigned del_cnt,
                                float alpha, float beta, float gamma )
{
    // 线性组合：你后续可以替换为更复杂的风险公式
    // 这里优先强调“作为前置条件出现”的重要性（req_cnt）
    return alpha * (float)req_cnt + beta * (float)add_cnt + gamma * (float)del_cnt;
}

std::vector<RiskScore> compute_risk_index( unsigned topK, float alpha, float beta, float gamma )
{
    PDDL::Task& task = PDDL::Task::instance();

    const unsigned nF = (unsigned)task.fluent_count();
    std::vector<RiskScore> ranked;
    ranked.reserve(nF);

    std::cout << "[RiskIndex] ==================================================" << std::endl;
    std::cout << "[RiskIndex] Start computing fluent risk index" << std::endl;
    std::cout << "[RiskIndex] fluent_count = " << nF
              << ", useful_ops = " << task.useful_ops().size() << std::endl;
    std::cout << "[RiskIndex] weights: alpha(req)=" << alpha
              << ", beta(add)=" << beta
              << ", gamma(del)=" << gamma << std::endl;

    // 逐 fluent 统计 required_by / added_by / deleted_by
    for ( unsigned f = 0; f < nF; ++f )
    {
        unsigned req_cnt = (unsigned)task.required_by(f).size();
        unsigned add_cnt = (unsigned)task.added_by(f).size();
        unsigned del_cnt = (unsigned)task.deleted_by(f).size();

        float score = calc_score(req_cnt, add_cnt, del_cnt, alpha, beta, gamma);

        RiskScore rs;
        rs.fluent  = f;
        rs.score   = score;
        rs.req_cnt = req_cnt;
        rs.add_cnt = add_cnt;
        rs.del_cnt = del_cnt;

        ranked.push_back(rs);

        // 细致逐条日志（你要的“更细”就在这里）
        std::cout << "[RiskIndex][Detail] f=" << std::setw(6) << f << "  ";
        task.print_fluent( f, std::cout ); // 直接打印 fluent 名
        std::cout << "  req=" << std::setw(4) << req_cnt
                  << " add=" << std::setw(4) << add_cnt
                  << " del=" << std::setw(4) << del_cnt
                  << "  score=" << std::fixed << std::setprecision(3) << score
                  << std::endl;
    }

    std::sort( ranked.begin(), ranked.end() );

    std::cout << "[RiskIndex] --------------------------------------------------" << std::endl;
    std::cout << "[RiskIndex] Top-" << topK << " risk fluents:" << std::endl;

    for ( unsigned i = 0; i < topK && i < ranked.size(); ++i )
    {
        const RiskScore& r = ranked[i];
        std::cout << "[RiskIndex][Top] rank=" << std::setw(3) << i
                  << " f=" << std::setw(6) << r.fluent << "  ";
        task.print_fluent( r.fluent, std::cout );
        std::cout << "  req=" << std::setw(4) << r.req_cnt
                  << " add=" << std::setw(4) << r.add_cnt
                  << " del=" << std::setw(4) << r.del_cnt
                  << "  score=" << std::fixed << std::setprecision(3) << r.score
                  << std::endl;
    }

    std::cout << "[RiskIndex] Done." << std::endl;
    std::cout << "[RiskIndex] ==================================================" << std::endl;

    return ranked;
}

} // namespace Risk
