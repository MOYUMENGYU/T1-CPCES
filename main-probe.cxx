
///*
//    Alexandre Albore, Miguel Ramirez, Hector Geffner
//    T1: A conformant planner
//    Copyright UPF (C) 2010

//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//*/
//#include <iostream>
//#include <cstdlib>
//#include <fstream>
//#include <signal.h>

//#include "utils.hxx"
//#include "PDDL.hxx"
//#include "nff_options.hxx"
//#include "global_options.hxx"

//#include "nff_belief.hxx"
//#include "nff_minisat.hxx"
//#include "nff_search_node.hxx"
//#include "nff_bsearch.hxx"

//#include "nff_log.hxx"
//#include "nff_ipc.hxx"
//#include "nff_planner_stats.hxx"
//#include "nff_mutexes.hxx"
//#include "risk_analysis.hxx"   // 加在顶部

//void reset_root_node_for_cegar( NFF::SearchNode* root, PDDL::Task& task )
//{
//    std::cout << "[重置] 重置 root 节点状态以进行新一轮迭代..." << std::endl;

//    // 1. 清理旧的 theory_step
//    if ( root->theory_step != NULL )
//    {
//        delete root->theory_step;
//        root->theory_step = NULL;
//    }

//    // 2. 重新创建初始 belief 状态
//    NFF::Belief* fresh_belief = NFF::Belief::make_initial_belief();
//    if ( root->b != NULL )
//    {
//        delete root->b;
//    }
//    root->b = fresh_belief;

//    // 3. 重置其他状态变量
//    root->gn = 0;
//    root->hn = 0;
//    root->hS = 0;
//    root->hX = 0;
//    root->op = 0;
//    root->father = NULL;
//    root->timestep = 0;

//    std::cout << "[重置] root 节点重置完成" << std::endl;
//}

//void	load_test_sequence( std::string fname, std::vector<PDDL::Operator*>& ops )
//{
//    PDDL::Task&	the_task = PDDL::Task::instance();
//    std::ifstream instream( fname.c_str() );
//    if ( instream.fail() )
//    {
//        std::cerr << "Could not open " << fname << std::endl;
//        std::exit(1);
//    }
//    while ( !instream.eof() )
//    {
//        char buffer[2048];
//        std::string line;
//        instream.getline( buffer, 2048, '\n' );
//        line.assign( buffer );
//        if (!line.empty() )
//        {
//            std::cout << "Got " << line << " from " << fname << std::endl;
//            bool found = false;
//            for ( unsigned k = 0; k < the_task.prototype_ops().size(); k++ )
//            {
//                PDDL::Operator* op = the_task.prototype_ops()[k];
//                if ( the_task.str_tab().get_token( op->code() )
//                    == line )
//                {
//                    ops.push_back(op);
//                    found = true;
//                    break;
//                }
//            }
//            if ( !found )
//            {
//                std::cerr << "Operator " << line << " does not exist" << std::endl;
//                std::exit(1);
//            }
//        }
//    }
//    instream.close();
//    for ( unsigned k = 0; k < ops.size(); k++ )
//    {
//        std::cout << "Loaded ";
//        the_task.print_operator( ops[k], std::cout );
//        std::cout << std::endl;
//    }
//}

//void handle_exit()
//{
//    NFF::Log&		log = NFF::Log::instance();
//    NFF::Planner_Stats&	stats = NFF::Planner_Stats::instance();
//    log.stream() << ";; heuristic_computations=" << stats.heuristic_computations() << std::endl;
//    log.stream() << ";; expanded_ehc=" << stats.expanded_ehc() << std::endl;
//    log.stream() << ";; generated_ehc=" << stats.generated_ehc() << std::endl;
//    log.stream() << ";; expanded_bfs=" << stats.expanded_bfs() << std::endl;
//    log.stream() << ";; generated_bfs=" << stats.generated_bfs() << std::endl;
//    log.stream() << ";; primary_expansions=" << stats.primary_expansions() << std::endl;
//    log.stream() << ";; secondary_expansions=" << stats.secondary_expansions() << std::endl;
//    log.stream() << ";; tertiary_expansions=" << stats.tertiary_expansions() << std::endl;
//    log.stream() << ";; entailment_tests=" << stats.entailment_tests() << std::endl;
//    log.stream() << ";; sat_solver_calls=" << stats.sat_solver_calls() << std::endl;
//    log.stream() << ";; max_num_clauses=" << stats.max_num_clauses() << std::endl;
//    log.stream() << ";; problem_inconsistent=" << ( stats.problem_inconsistent() ? "yes" : "no" ) << std::endl;
//    log.stream() << ";; initial_sample_h=" << stats.initial_sample_heuristic() << std::endl;
//    log.stream() << ";; initial_card_h=" << stats.initial_card_heuristic() << std::endl;
//    log.stream() << ";; avg_dead_models=" << stats.num_dead_models() << std::endl;
//    log.stream() << ";; dead_samples_replaced=" << stats.dead_samples_replaced() << std::endl;
//    log.stream() << ";; width_1=" << ( stats.is_width_1() ? "yes" : "no" ) << std::endl;
////	log.stream() << ";; hash_size=" << stats.hashtable_nodes() << std::endl;
//    log.stream() << ";; nodes_mem=" << stats.node_megabytes() << std::endl;
//    log.stream() << ";; b0_mc=" << stats.b0_mc() << std::endl;
//    log.stream().flush();
//    log.stream().close();
//}

//void handle_signal( int signum )
//{
//    NFF::Log&		log = NFF::Log::instance();
//    NFF::Planner_Stats&	stats = NFF::Planner_Stats::instance();
//    log.stream() << ";; heuristic_computations=" << stats.heuristic_computations() << std::endl;
//    log.stream() << ";; expanded_ehc=" << stats.expanded_ehc() << std::endl;
//    log.stream() << ";; generated_ehc=" << stats.generated_ehc() << std::endl;
//    log.stream() << ";; expanded_bfs=" << stats.expanded_bfs() << std::endl;
//    log.stream() << ";; generated_bfs=" << stats.generated_bfs() << std::endl;
//    log.stream() << ";; primary_expansions=" << stats.primary_expansions() << std::endl;
//    log.stream() << ";; secondary_expansions=" << stats.secondary_expansions() << std::endl;
//    log.stream() << ";; tertiary_expansions=" << stats.tertiary_expansions() << std::endl;
//    log.stream() << ";; entailment_tests=" << stats.entailment_tests() << std::endl;
//    log.stream() << ";; sat_solver_calls=" << stats.sat_solver_calls() << std::endl;
//    log.stream() << ";; max_num_clauses=" << stats.max_num_clauses() << std::endl;
//    log.stream() << ";; problem_inconsistent=" << ( stats.problem_inconsistent() ? "yes" : "no" ) << std::endl;
//    log.stream() << ";; initial_sample_h=" << stats.initial_sample_heuristic() << std::endl;
//    log.stream() << ";; initial_card_h=" << stats.initial_card_heuristic() << std::endl;
//    log.stream() << ";; avg_dead_models=" << stats.num_dead_models() << std::endl;
//    log.stream() << ";; dead_samples_replaced=" << stats.dead_samples_replaced() << std::endl;
//    log.stream() << ";; width_1=" << ( stats.is_width_1() ? "yes" : "no" ) << std::endl;
////	log.stream() << ";; hash_size=" << stats.hashtable_nodes() << std::endl;
//    log.stream() << ";; nodes_mem=" << stats.node_megabytes() << std::endl;
//    log.stream() << ";; b0_mc=" << stats.b0_mc() << std::endl;
//    log.stream().close();

//    std::exit( signum );
//}

//void register_signal_handlers()
//{
//    int signals[] = {SIGXCPU, SIGTERM, SIGTRAP, SIGABRT, SIGFPE, SIGBUS, SIGSEGV};
//    for ( int i = 0; i < 7; i++ )
//        signal(signals[i], handle_signal);

//}

//static bool same_state_vec( const std::vector<int>& a, const std::vector<int>& b )
//{
//    if ( a.size() != b.size() ) return false;
//    for ( size_t i = 0; i < a.size(); ++i )
//        if ( a[i] != b[i] ) return false;
//    return true;
//}


//int main( int argc, char** argv )
//{

//    std::cout << "[TEST] 主程序开始执行" << std::endl;
//    register_signal_handlers();
//    double t0, tf;


//    NFF_Options::parse_command_line( argc, argv );
//    NFF_Options&	prog_opts = NFF_Options::instance();
//        NFF::Ipc&	ipc = NFF::Ipc::instance();
//    NFF::Log&	log = NFF::Log::instance();
//    log.stream() <<	";; t1 - A Heuristic Search Conformant Planner" << std::endl;
//    log.stream() << ";; Alexandre Albore, Hector Geffner & Miquel Ramirez (c) 2010" << std::endl;

//    PDDL::Task& task = PDDL::Task::instance();
//    task.setup();


//    t0 = time_used();

//    // 2026.1.26
//    unsigned topK = 20;
//    std::vector<Risk::RiskScore> ranked = Risk::compute_risk_index(topK);

//    std::cout << "[RiskIndex][Debug] ranked.size()=" << ranked.size() << std::endl;

//    for (size_t i = 0; i < ranked.size(); ++i)
//    {
//        const Risk::RiskScore& rs = ranked[i];
//        std::cout << "[RiskIndex][Ranked] i=" << i
//                  << " f=" << rs.fluent
//                  << " score=" << rs.score
//                  << " name=";
//        task.print_fluent(rs.fluent, std::cout);
//        std::cout << std::endl;
//    }

//    // 你后续要用的关键命题变量集合（先存起来）
//    std::vector<unsigned> critical;
//    for (unsigned i = 0; i < topK && i < ranked.size(); ++i)
//        critical.push_back(ranked[i].fluent);

//    // 交给 Task 保存（供样本生成/搜索使用）
//    PDDL::Task::instance().set_critical_fluents( critical );

//    // 控制台日志：确认 Task 中最终保存的 critical 集合
//    std::cout << "[RiskIndex] ===== critical fluents stored in Task =====" << std::endl;
//    const std::vector<unsigned>& cf = PDDL::Task::instance().critical_fluents();
//    for (unsigned i = 0; i < cf.size(); ++i) {
//        std::cout << "[RiskIndex][Critical] #" << i << " f=" << cf[i] << " ";
//        PDDL::Task::instance().print_fluent( cf[i], std::cout );
//        std::cout << std::endl;
//    }


//    if ( prog_opts.use_invariant_xors() || prog_opts.use_invariant_diameter() )
//    {
//        NFF::Mutexes& m = NFF::Mutexes::instance();
//        task.set_mutexes( &m );
//        m.create_mutexes();
//    }

//        task.calculate_relevance();


//    tf = time_used();
//    log.stream() << ";; preprocessing="; report_interval( t0, tf, log.stream() );

//    if ( prog_opts.verbose_mode() )
//    {
//        std::ofstream fluents_out( "fluents.list" );
//        std::ofstream ops_out( "operators.list" );
//        std::ofstream init_out( "initial.list" );
//        std::ofstream goal_out( "goal.list" );

//        task.print_fluents( fluents_out );
//        task.print_operators( ops_out );
//        task.print_initial_state( init_out );
//        task.print_goal_state( goal_out );

//        fluents_out.close();
//        ops_out.close();
//    }
//    std::cout << "Fluents=" << task.fluents().size() << std::endl;
//    std::cout << "Operators=" << task.useful_ops().size() << std::endl;
//    log.stream() << ";; domain=" << task.domain_name() << std::endl;
//    log.stream() << ";; problem=" << task.problem_name() << std::endl;
//    log.stream() << ";; fluents=" << task.fluents().size() << std::endl;
//    log.stream() << ";; operators=" << task.useful_ops().size() << std::endl;

//        if ( prog_opts.output_ipc() )
//        {
//                ipc.stream() << task.fluents().size()-1 << std::endl;
//                for (unsigned f = 0; f < task.fluents().size()-1; f++)
//                {
//                        task.print_fluent( f, ipc.stream() );
//                        ipc.stream() << " ";
//                }
//                ipc.stream() << std::endl << "%%" << std::endl;
//                ipc.stream() << task.prototype_ops().size() << std::endl;
//                for (unsigned f = 0; f < task.prototype_ops().size(); f++)
//                {
//                        task.print_operator( task.prototype_ops()[f], ipc.stream() );
//                        ipc.stream() << " ";
//                }
//                ipc.stream() << std::endl << "%%" << std::endl;
//        }
//    if ( NFF_Options::instance().only_grounding() )
//    {
//        std::cout << "Stopping after PDDL task has been processed" << std::endl;
//        std::exit(0);
//    }

//    /** Now starts the proper search **/

//    NFF::BeliefSearch search;
//    if ( !prog_opts.test_sequence().empty() )
//    {
//        std::vector<PDDL::Operator*> plan_prefix;
//        load_test_sequence( prog_opts.test_sequence(), plan_prefix );
//        search.execute( plan_prefix );
//        std::exit(0);
//    }
//    if ( prog_opts.only_initial_node() )
//    {
//        search.process_root_node();
//        handle_exit();
//        std::exit(0);
//    }

////	search.solve();
////	handle_exit();
////	std::exit(0);
////	return 0;

//    // 2026.1.26
//    // =========================
//    // CPCES/CEGAR outer loop
//    // =========================
//    const unsigned MAX_CEGAR_ITERS = 50; // can be adjusted
//    unsigned it = 0;

//    std::vector< std::vector<int> > samples;   // persistent sample set across iterations

//    while ( it < MAX_CEGAR_ITERS )
//    {
//        std::cout << std::endl;
//        std::cout << "========================================================" << std::endl;
//        std::cout << "[CEGAR] 开始第 " << (it+1) << " 轮迭代 (最大: " << MAX_CEGAR_ITERS << ")" << std::endl;
//        std::cout << "========================================================" << std::endl;

//        NFF::BeliefSearch search;

//        // 在 CEGAR 循环中，每轮迭代开始时：
//        NFF::SearchNode* root_node = NFF::SearchNode::root();
//        if ( it > 0 )
//        {
//            reset_root_node_for_cegar( root_node, task );
//        }

//        // Reuse previous samples if any
//        if ( !samples.empty() )
//        {
//            search.models() = samples;
//            std::cout << "[样本集] 复用已有样本进行规划, |S|=" << samples.size() << std::endl;
//        }
//        else
//        {
//            std::cout << "[样本集] 首次迭代，生成初始样本..." << std::endl;
//        }

//        bool ok = search.solve();

//        if ( !ok || search.failed() )
//        {
//            std::cout << std::endl;
//            std::cout << "========================================================" << std::endl;
//            std::cout << "[CEGAR] 规划搜索失败，无法找到有效规划。" << std::endl;
//            std::cout << "========================================================" << std::endl;
//            break;
//        }

//        const std::vector<unsigned>& plan_ops = search.last_plan();
//        std::cout << std::endl;
//        std::cout << "[规划] 找到候选规划，长度=" << plan_ops.size() << " 步" << std::endl;
//        std::cout << "[规划] 候选动作序列:" << std::endl;
//        for ( unsigned k = 0; k < plan_ops.size(); k++ )
//        {
//            std::cout << "       " << (k+1) << ". ";
//            task.print_operator( plan_ops[k], std::cout );
//            std::cout << std::endl;
//        }
//        std::cout << std::endl;

//        // --------
//        // Verify plan by replay + SAT goal entailment test
//        // --------
//        // Replay the plan to build theory_step chain:
////        NFF::SearchNode* n = NFF::SearchNode::root();
////        search.set_root(n);

//        // Build the initial approximation (must exist)
//        // If this is iter>0, solve() already reused samples and set initial models.
//        // We still need to create theory fragments along the plan prefix:
////        for ( unsigned k = 0; k < plan_ops.size(); k++ )
////        {
////            n = n->successor( plan_ops[k] );
////            n->belief_tracking_with_sat();
////        }


//        // --------
//        // Verify plan by replay + SAT goal entailment test
//        // --------

//        // 1) 取 root，并让 root 的 belief/unknown 与 samples 对齐
//        NFF::SearchNode* root = NFF::SearchNode::root();
//        NFF::SearchNode* n = root;



//        if ( samples.empty() )
//        {
//            // 理论上不会发生：solve() 成功后 search.models() 至少有初始样本
//            std::cout << "[CEGAR][REPLAY] ERROR: samples empty after solve()" << std::endl;
//        }
//        else
//        {
//            n->do_belief_tracking( samples );
//        }

//        // 2) 关键：root 必须有 theory_step（否则第一步 belief_tracking_with_sat 会用 father->theory_step 解引用空指针）
//        if ( n->theory_step == NULL )
//        {
//            n->theory_step = new NFF::Theory_Fragment();
//            std::cout << "[验证] 初始化 root 节点的 SAT 理论片段 (t=0)" << std::endl;
//            // 添加初始子句
//            n->make_root_clauses();
//            std::cout << "[验证] root 节点的初始子句构建完成" << std::endl;
//        }

//        // 3) 回放计划：每步 successor 后都要 do_belief_tracking(samples)，再生成该步 theory
//        std::cout << "[验证] 开始回放验证候选规划..." << std::endl;
//        bool replay_ok = true;
//        for ( unsigned k = 0; k < plan_ops.size(); k++ )
//        {
//            unsigned op_idx = plan_ops[k];

//            std::cout << "[验证] 回放步骤 " << (k+1) << "/" << plan_ops.size() << ": ";
//            task.print_operator( op_idx, std::cout );
//            std::cout << std::endl;

//            // 动作可执行性检查（避免 silent fail）
//            if ( n->b == NULL || !n->b->can_apply( op_idx ) )
//            {
//                std::cout << "[验证] 错误: 动作在步骤 " << (k+1) << " 不可执行！" << std::endl;
//                std::cout << "       belief 状态为空或前置条件不满足。" << std::endl;
//                replay_ok = false;
//                break;
//            }

//            // 生成后继节点
//            n = n->successor( op_idx );

//            // 用 samples 更新 belief/unknown/known 等
//            n->do_belief_tracking( samples );

//            // 生成这一层 SAT theory（依赖 father->theory_step 已存在）
//            n->belief_tracking_with_sat();
//        }

//        if ( !replay_ok )
//        {
//            std::cout << std::endl;
//            std::cout << "[验证] 规划回放失败，终止。" << std::endl;
//            break;
//        }

//        std::cout << "[验证] 回放完成，开始 SAT 目标蕴含测试..." << std::endl;


//        bool valid = true;
//        std::vector<int> sat_model;
//        unsigned failed_goal = 0;

//        PDDL::Task& t = PDDL::Task::instance();
//        std::vector<unsigned>& G = t.goal_state();


//        for ( unsigned gi = 0; gi < G.size(); gi++ )
//        {
//            unsigned g = G[gi];
//            sat_model.clear();

//            std::cout << "[验证] 检查目标 " << (gi+1) << "/" << G.size() << ": ";
//            t.print_fluent( g, std::cout );

//            // If we can find a model that falsifies goal g => counterexample exists => plan invalid
//            if ( n->get_counterexample_for_literal( g, sat_model ) )
//            {
//                valid = false;
//                failed_goal = g;

//                std::cout << " -> 发现反例!" << std::endl;
//                std::cout << "[反例] 目标 ";
//                t.print_fluent( failed_goal, std::cout );
//                std::cout << " 存在反例状态，规划无效。" << std::endl;
//                break;
//            }
//            else
//            {
//                std::cout << " -> 已蕴含 ✓" << std::endl;
//            }
//        }

//        if ( valid )
//        {
//            std::cout << std::endl;
//            std::cout << "========================================================" << std::endl;
//            std::cout << "=====          [CEGAR] 规划验证成功！             =====" << std::endl;
//            std::cout << "========================================================" << std::endl;
//            std::cout << std::endl;
//            std::cout << "[结果] 所有目标均已蕴含，找到一致性规划！" << std::endl;
//            std::cout << std::endl;
//            std::cout << "[最终规划] 共 " << plan_ops.size() << " 步:" << std::endl;
//            std::cout << "--------------------------------------------------------" << std::endl;
//            for ( unsigned k = 0; k < plan_ops.size(); k++ )
//            {
//                std::cout << "  步骤 " << (k+1) << ": ";
//                task.print_operator( plan_ops[k], std::cout );
//                std::cout << std::endl;
//            }
//            std::cout << "--------------------------------------------------------" << std::endl;
//            std::cout << std::endl;
//            std::cout << "[目标状态] 已达成:" << std::endl;
//            PDDL::Task& t2 = PDDL::Task::instance();
//            std::vector<unsigned>& goals = t2.goal_state();
//            for ( unsigned gi = 0; gi < goals.size(); gi++ )
//            {
//                std::cout << "  - ";
//                t2.print_fluent( goals[gi], std::cout );
//                std::cout << " ✓" << std::endl;
//            }
//            std::cout << std::endl;
//            std::cout << "[CEGAR] 迭代次数: " << (it+1) << std::endl;
//            std::cout << "[样本集] 最终样本数量: |S|=" << samples.size() << std::endl;
//            std::cout << std::endl;
//            std::cout << "========================================================" << std::endl;
//            std::cout << "=====              规划完成！                     =====" << std::endl;
//            std::cout << "========================================================" << std::endl;
//            handle_exit();
//            std::exit(0);
//        }

//        // --------
//        // Convert SAT model -> a new sample state, then update S
//        // --------
//        std::vector<int> new_state;
//        new_state.resize( t.fluent_count(), 0 );

//        // Build state assignment for positive fluents, and fill their neg equivalents accordingly
////        for ( unsigned f = 1; f < (unsigned)(t.fluent_count()-1); f++ )
////        {
////            if ( !t.fluents()[f]->is_pos() ) continue;

////            unsigned v = n->theory_step->fluent_var(f);
////            int assn = 0;
////            if ( v < sat_model.size() )
////                assn = sat_model[v];

////            // If SAT model doesn't provide assignment, skip (keep 0)
////            if ( assn == 0 ) continue;

////            int sign_true = (assn > 0 ? 1 : -1);
////            new_state[f] = sign_true * (int)f;

////            unsigned nf = t.not_equivalent(f);
////            if ( nf != 0 )
////                new_state[nf] = -sign_true * (int)nf;
////        }

//        // 2026.1.27
//        // Build state assignment for positive fluents, and fill their neg equivalents accordingly
//        // IMPORTANT: must project SAT model to t=0 variables using root->theory_step
//        unsigned undef_cnt = 0;

//        for ( unsigned f = 1; f < (unsigned)(t.fluent_count()-1); f++ )
//        {
//            if ( !t.fluents()[f]->is_pos() ) continue;

//            // *** FIX: use root(t=0) mapping, NOT the last node mapping ***
//            unsigned v = root->theory_step->fluent_var( f );

//            int assn = 0;
//            if ( v < sat_model.size() )
//                assn = sat_model[v];

//            // 处理 Undef：为了形成一个“完整样本”，这里默认当作 false（并打印统计）
//            bool is_true = false;
//            if ( assn == 0 )
//            {
//                undef_cnt++;
//                is_true = false;
//            }
//            else
//            {
//                is_true = (assn > 0);
//            }

//            int sign_true = is_true ? 1 : -1;
//            new_state[f] = sign_true * (int)f;

//            unsigned nf = t.not_equivalent( f );
//            if ( nf != 0 )
//                new_state[nf] = -sign_true * (int)nf;
//        }

//        if ( undef_cnt > 0 )
//        {
//            std::cout << "[反例] 警告: SAT 模型中有 " << undef_cnt
//                      << " 个未定义变量 (默认设为 FALSE)" << std::endl;
//        }



//        bool is_new = true;
//        for ( unsigned si = 0; si < (samples.empty() ? search.models().size() : samples.size()); si++ )
//        {
//            const std::vector<int>& oldS = (samples.empty() ? search.models()[si] : samples[si]);
//            if ( same_state_vec(oldS, new_state) )
//            {
//                is_new = false;
//                break;
//            }
//        }

//        // Always synchronize samples from search (solve() may update/replace models internally)
//        samples = search.models();
//        std::cout << "[样本集] 同步求解器样本, |S|=" << samples.size() << std::endl;


//        std::cout << "[样本集] 当前样本数量: |S|=" << samples.size() << std::endl;

//        if ( is_new )
//        {
//            samples.push_back( new_state );
//            std::cout << "[样本集] 添加 1 个反例状态, 新 |S|=" << samples.size() << std::endl;
//        }
//        else
//        {
//            std::cout << "[样本集] 反例与已有样本重复, |S| 不变" << std::endl;
//        }

//        std::cout << std::endl;
//        std::cout << "[CEGAR] 第 " << (it+1) << " 轮迭代完成，继续精化规划..." << std::endl;
//        it++;
//    }

//    std::cout << std::endl;
//    std::cout << "========================================================" << std::endl;
//    std::cout << "[CEGAR] 达到最大迭代次数或搜索失败，终止。" << std::endl;
//    std::cout << "========================================================" << std::endl;
//    handle_exit();
//    std::exit(0);

//}

/*
    Alexandre Albore, Miguel Ramirez, Hector Geffner
    T1: A conformant planner
    Copyright UPF (C) 2010

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <signal.h>

#include "utils.hxx"
#include "PDDL.hxx"
#include "nff_options.hxx"
#include "global_options.hxx"

#include "nff_belief.hxx"
#include "nff_minisat.hxx"
#include "nff_search_node.hxx"
#include "nff_bsearch.hxx"

#include "nff_log.hxx"
#include "nff_ipc.hxx"
#include "nff_planner_stats.hxx"
#include "nff_mutexes.hxx"
#include "risk_analysis.hxx"   // 加在顶部

void reset_root_node_for_cegar( NFF::SearchNode* root, PDDL::Task& task )
{
    std::cout << "[重置] 重置 root 节点状态以进行新一轮迭代..." << std::endl;

    // 1. 清理旧的 theory_step
    if ( root->theory_step != NULL )
    {
        delete root->theory_step;
        root->theory_step = NULL;
    }

    // 2. 重新创建初始 belief 状态
    NFF::Belief* fresh_belief = NFF::Belief::make_initial_belief();
    if ( root->b != NULL )
    {
        delete root->b;
    }
    root->b = fresh_belief;

    // 3. 重置其他状态变量
    root->gn = 0;
    root->hn = 0;
    root->hS = 0;
    root->hX = 0;
    root->op = 0;
    root->father = NULL;
    root->timestep = 0;

    std::cout << "[重置] root 节点重置完成" << std::endl;
}

void	load_test_sequence( std::string fname, std::vector<PDDL::Operator*>& ops )
{
    PDDL::Task&	the_task = PDDL::Task::instance();
    std::ifstream instream( fname.c_str() );
    if ( instream.fail() )
    {
        std::cerr << "Could not open " << fname << std::endl;
        std::exit(1);
    }
    while ( !instream.eof() )
    {
        char buffer[2048];
        std::string line;
        instream.getline( buffer, 2048, '\n' );
        line.assign( buffer );
        if (!line.empty() )
        {
            std::cout << "Got " << line << " from " << fname << std::endl;
            bool found = false;
            for ( unsigned k = 0; k < the_task.prototype_ops().size(); k++ )
            {
                PDDL::Operator* op = the_task.prototype_ops()[k];
                if ( the_task.str_tab().get_token( op->code() )
                    == line )
                {
                    ops.push_back(op);
                    found = true;
                    break;
                }
            }
            if ( !found )
            {
                std::cerr << "Operator " << line << " does not exist" << std::endl;
                std::exit(1);
            }
        }
    }
    instream.close();
    for ( unsigned k = 0; k < ops.size(); k++ )
    {
        std::cout << "Loaded ";
        the_task.print_operator( ops[k], std::cout );
        std::cout << std::endl;
    }
}

void handle_exit()
{
    NFF::Log&		log = NFF::Log::instance();
    NFF::Planner_Stats&	stats = NFF::Planner_Stats::instance();
    log.stream() << ";; heuristic_computations=" << stats.heuristic_computations() << std::endl;
    log.stream() << ";; expanded_ehc=" << stats.expanded_ehc() << std::endl;
    log.stream() << ";; generated_ehc=" << stats.generated_ehc() << std::endl;
    log.stream() << ";; expanded_bfs=" << stats.expanded_bfs() << std::endl;
    log.stream() << ";; generated_bfs=" << stats.generated_bfs() << std::endl;
    log.stream() << ";; primary_expansions=" << stats.primary_expansions() << std::endl;
    log.stream() << ";; secondary_expansions=" << stats.secondary_expansions() << std::endl;
    log.stream() << ";; tertiary_expansions=" << stats.tertiary_expansions() << std::endl;
    log.stream() << ";; entailment_tests=" << stats.entailment_tests() << std::endl;
    log.stream() << ";; sat_solver_calls=" << stats.sat_solver_calls() << std::endl;
    log.stream() << ";; max_num_clauses=" << stats.max_num_clauses() << std::endl;
    log.stream() << ";; problem_inconsistent=" << ( stats.problem_inconsistent() ? "yes" : "no" ) << std::endl;
    log.stream() << ";; initial_sample_h=" << stats.initial_sample_heuristic() << std::endl;
    log.stream() << ";; initial_card_h=" << stats.initial_card_heuristic() << std::endl;
    log.stream() << ";; avg_dead_models=" << stats.num_dead_models() << std::endl;
    log.stream() << ";; dead_samples_replaced=" << stats.dead_samples_replaced() << std::endl;
    log.stream() << ";; width_1=" << ( stats.is_width_1() ? "yes" : "no" ) << std::endl;
//	log.stream() << ";; hash_size=" << stats.hashtable_nodes() << std::endl;
    log.stream() << ";; nodes_mem=" << stats.node_megabytes() << std::endl;
    log.stream() << ";; b0_mc=" << stats.b0_mc() << std::endl;
    log.stream().flush();
    log.stream().close();
}

void handle_signal( int signum )
{
    NFF::Log&		log = NFF::Log::instance();
    NFF::Planner_Stats&	stats = NFF::Planner_Stats::instance();
    log.stream() << ";; heuristic_computations=" << stats.heuristic_computations() << std::endl;
    log.stream() << ";; expanded_ehc=" << stats.expanded_ehc() << std::endl;
    log.stream() << ";; generated_ehc=" << stats.generated_ehc() << std::endl;
    log.stream() << ";; expanded_bfs=" << stats.expanded_bfs() << std::endl;
    log.stream() << ";; generated_bfs=" << stats.generated_bfs() << std::endl;
    log.stream() << ";; primary_expansions=" << stats.primary_expansions() << std::endl;
    log.stream() << ";; secondary_expansions=" << stats.secondary_expansions() << std::endl;
    log.stream() << ";; tertiary_expansions=" << stats.tertiary_expansions() << std::endl;
    log.stream() << ";; entailment_tests=" << stats.entailment_tests() << std::endl;
    log.stream() << ";; sat_solver_calls=" << stats.sat_solver_calls() << std::endl;
    log.stream() << ";; max_num_clauses=" << stats.max_num_clauses() << std::endl;
    log.stream() << ";; problem_inconsistent=" << ( stats.problem_inconsistent() ? "yes" : "no" ) << std::endl;
    log.stream() << ";; initial_sample_h=" << stats.initial_sample_heuristic() << std::endl;
    log.stream() << ";; initial_card_h=" << stats.initial_card_heuristic() << std::endl;
    log.stream() << ";; avg_dead_models=" << stats.num_dead_models() << std::endl;
    log.stream() << ";; dead_samples_replaced=" << stats.dead_samples_replaced() << std::endl;
    log.stream() << ";; width_1=" << ( stats.is_width_1() ? "yes" : "no" ) << std::endl;
//	log.stream() << ";; hash_size=" << stats.hashtable_nodes() << std::endl;
    log.stream() << ";; nodes_mem=" << stats.node_megabytes() << std::endl;
    log.stream() << ";; b0_mc=" << stats.b0_mc() << std::endl;
    log.stream().close();

    std::exit( signum );
}

void register_signal_handlers()
{
    int signals[] = {SIGXCPU, SIGTERM, SIGTRAP, SIGABRT, SIGFPE, SIGBUS, SIGSEGV};
    for ( int i = 0; i < 7; i++ )
        signal(signals[i], handle_signal);

}

static bool same_state_vec( const std::vector<int>& a, const std::vector<int>& b )
{
    if ( a.size() != b.size() ) return false;
    for ( size_t i = 0; i < a.size(); ++i )
        if ( a[i] != b[i] ) return false;
    return true;
}

// 打印样本集摘要，便于定位“样本为空/包含 0/长度异常”等问题
static void dump_samples_summary( const std::vector< std::vector<int> >& S, PDDL::Task& task, const std::string& where )
{
    size_t n = S.size();
    size_t min_len = (n == 0 ? 0 : S[0].size());
    size_t max_len = 0;
    size_t bad_len = 0;
    size_t has_zero = 0;

    for ( size_t i = 0; i < n; ++i )
    {
        size_t len = S[i].size();
        if ( len < min_len ) min_len = len;
        if ( len > max_len ) max_len = len;
        if ( len != (size_t)task.fluent_count() ) bad_len++;
        for ( size_t j = 0; j < len; ++j )
        {
            if ( S[i][j] == 0 )
            {
                has_zero++;
                break;
            }
        }
    }

    std::cout << "[样本检查] " << where
              << " |S|=" << n
              << " min_len=" << min_len
              << " max_len=" << max_len
              << " bad_len=" << bad_len
              << " has_zero=" << has_zero;

    if ( n > 0 )
    {
        unsigned gid = task.dummy_goal();
        int glit = 0;
        if ( gid < S[n-1].size() ) glit = S[n-1][gid];
        std::cout << " last_sample_GOAL=" << glit;
    }
    std::cout << std::endl;
}


int main( int argc, char** argv )
{

    std::cout << "[TEST] 主程序开始执行" << std::endl;
    register_signal_handlers();
    double t0, tf;


    NFF_Options::parse_command_line( argc, argv );
    NFF_Options&	prog_opts = NFF_Options::instance();
        NFF::Ipc&	ipc = NFF::Ipc::instance();
    NFF::Log&	log = NFF::Log::instance();
    log.stream() <<	";; t1 - A Heuristic Search Conformant Planner" << std::endl;
    log.stream() << ";; Alexandre Albore, Hector Geffner & Miquel Ramirez (c) 2010" << std::endl;

    PDDL::Task& task = PDDL::Task::instance();
    task.setup();


    t0 = time_used();

    // 2026.1.26
    unsigned topK = 20;
    std::vector<Risk::RiskScore> ranked = Risk::compute_risk_index(topK);

    std::cout << "[RiskIndex][Debug] ranked.size()=" << ranked.size() << std::endl;

    for (size_t i = 0; i < ranked.size(); ++i)
    {
        const Risk::RiskScore& rs = ranked[i];
        std::cout << "[RiskIndex][Ranked] i=" << i
                  << " f=" << rs.fluent
                  << " score=" << rs.score
                  << " name=";
        task.print_fluent(rs.fluent, std::cout);
        std::cout << std::endl;
    }

    // 你后续要用的关键命题变量集合（先存起来）
    std::vector<unsigned> critical;
    for (unsigned i = 0; i < topK && i < ranked.size(); ++i)
        critical.push_back(ranked[i].fluent);

    // 交给 Task 保存（供样本生成/搜索使用）
    PDDL::Task::instance().set_critical_fluents( critical );

    // 控制台日志：确认 Task 中最终保存的 critical 集合
    std::cout << "[RiskIndex] ===== critical fluents stored in Task =====" << std::endl;
    const std::vector<unsigned>& cf = PDDL::Task::instance().critical_fluents();
    for (unsigned i = 0; i < cf.size(); ++i) {
        std::cout << "[RiskIndex][Critical] #" << i << " f=" << cf[i] << " ";
        PDDL::Task::instance().print_fluent( cf[i], std::cout );
        std::cout << std::endl;
    }


    if ( prog_opts.use_invariant_xors() || prog_opts.use_invariant_diameter() )
    {
        NFF::Mutexes& m = NFF::Mutexes::instance();
        task.set_mutexes( &m );
        m.create_mutexes();
    }

        task.calculate_relevance();


    tf = time_used();
    log.stream() << ";; preprocessing="; report_interval( t0, tf, log.stream() );

    if ( prog_opts.verbose_mode() )
    {
        std::ofstream fluents_out( "fluents.list" );
        std::ofstream ops_out( "operators.list" );
        std::ofstream init_out( "initial.list" );
        std::ofstream goal_out( "goal.list" );

        task.print_fluents( fluents_out );
        task.print_operators( ops_out );
        task.print_initial_state( init_out );
        task.print_goal_state( goal_out );

        fluents_out.close();
        ops_out.close();
    }
    std::cout << "Fluents=" << task.fluents().size() << std::endl;
    std::cout << "Operators=" << task.useful_ops().size() << std::endl;
    log.stream() << ";; domain=" << task.domain_name() << std::endl;
    log.stream() << ";; problem=" << task.problem_name() << std::endl;
    log.stream() << ";; fluents=" << task.fluents().size() << std::endl;
    log.stream() << ";; operators=" << task.useful_ops().size() << std::endl;

        if ( prog_opts.output_ipc() )
        {
                ipc.stream() << task.fluents().size()-1 << std::endl;
                for (unsigned f = 0; f < task.fluents().size()-1; f++)
                {
                        task.print_fluent( f, ipc.stream() );
                        ipc.stream() << " ";
                }
                ipc.stream() << std::endl << "%%" << std::endl;
                ipc.stream() << task.prototype_ops().size() << std::endl;
                for (unsigned f = 0; f < task.prototype_ops().size(); f++)
                {
                        task.print_operator( task.prototype_ops()[f], ipc.stream() );
                        ipc.stream() << " ";
                }
                ipc.stream() << std::endl << "%%" << std::endl;
        }
    if ( NFF_Options::instance().only_grounding() )
    {
        std::cout << "Stopping after PDDL task has been processed" << std::endl;
        std::exit(0);
    }

    /** Now starts the proper search **/

    NFF::BeliefSearch search;
    if ( !prog_opts.test_sequence().empty() )
    {
        std::vector<PDDL::Operator*> plan_prefix;
        load_test_sequence( prog_opts.test_sequence(), plan_prefix );
        search.execute( plan_prefix );
        std::exit(0);
    }
    if ( prog_opts.only_initial_node() )
    {
        search.process_root_node();
        handle_exit();
        std::exit(0);
    }

//	search.solve();
//	handle_exit();
//	std::exit(0);
//	return 0;

    // 2026.1.26
    // =========================
    // CPCES/CEGAR outer loop
    // =========================
    const unsigned MAX_CEGAR_ITERS = 50; // can be adjusted
    unsigned it = 0;

    std::vector< std::vector<int> > samples;   // persistent sample set across iterations

    while ( it < MAX_CEGAR_ITERS )
    {
        std::cout << std::endl;
        std::cout << "========================================================" << std::endl;
        std::cout << "[CEGAR] 开始第 " << (it+1) << " 轮迭代 (最大: " << MAX_CEGAR_ITERS << ")" << std::endl;
        std::cout << "========================================================" << std::endl;

        NFF::BeliefSearch search;

        // 在 CEGAR 循环中，每轮迭代开始时：
        NFF::SearchNode* root_node = NFF::SearchNode::root();
        if ( it > 0 )
        {
            reset_root_node_for_cegar( root_node, task );
        }

        // Reuse previous samples if any
        if ( !samples.empty() )
        {
            search.models() = samples;
            std::cout << "[样本集] 复用已有样本进行规划, |S|=" << samples.size() << std::endl;
        }
        else
        {
            std::cout << "[样本集] 首次迭代，生成初始样本..." << std::endl;
        }

        bool ok = search.solve();
        if ( !ok || search.failed() )
        {
            std::cout << std::endl;
            std::cout << "========================================================" << std::endl;
            std::cout << "[CEGAR] 规划搜索失败，无法找到有效规划。" << std::endl;
            std::cout << "========================================================" << std::endl;
            break;
        }

        const std::vector<unsigned>& plan_ops = search.last_plan();
        std::cout << std::endl;
        std::cout << "[规划] 找到候选规划，长度=" << plan_ops.size() << " 步" << std::endl;
        std::cout << "[规划] 候选动作序列:" << std::endl;
        for ( unsigned k = 0; k < plan_ops.size(); k++ )
        {
            std::cout << "       " << (k+1) << ". ";
            task.print_operator( plan_ops[k], std::cout );
            std::cout << std::endl;
        }
        std::cout << std::endl;

        // ===== 关键修复：验证/反例测试前必须同步本轮求解器使用/生成的样本集 =====
        // 第 1 轮迭代时 samples 为空；solve() 内部会生成初始样本，但如果不同步出来，
        // 后续回放验证会用空样本，导致 belief/unknown/theory 构建错误。
        samples = search.models();
        std::cout << "[样本集] 从本轮求解器同步样本, |S|=" << samples.size() << std::endl;
        dump_samples_summary( samples, task, "after_solve_sync" );
        if ( samples.empty() )
        {
            std::cout << "[CEGAR][验证] 错误: solve() 成功但样本集为空，无法进行回放/反例测试，终止。" << std::endl;
            break;
        }

        // --------
        // Verify plan by replay + SAT goal entailment test
        // --------
        // Replay the plan to build theory_step chain:
//        NFF::SearchNode* n = NFF::SearchNode::root();
//        search.set_root(n);

        // Build the initial approximation (must exist)
        // If this is iter>0, solve() already reused samples and set initial models.
        // We still need to create theory fragments along the plan prefix:
//        for ( unsigned k = 0; k < plan_ops.size(); k++ )
//        {
//            n = n->successor( plan_ops[k] );
//            n->belief_tracking_with_sat();
//        }


        // --------
        // Verify plan by replay + SAT goal entailment test
        // --------

        // 1) 取 root，并让 root 的 belief/unknown 与 samples 对齐
        NFF::SearchNode* root = NFF::SearchNode::root();
        NFF::SearchNode* n = root;



        if ( samples.empty() )
        {
            // 理论上不会发生：solve() 成功后 search.models() 至少有初始样本
            std::cout << "[CEGAR][REPLAY] ERROR: samples empty after solve()" << std::endl;
        }
        else
        {
            n->do_belief_tracking( samples );
        }

        // 2) 关键：root 必须有 theory_step（否则第一步 belief_tracking_with_sat 会用 father->theory_step 解引用空指针）
        if ( n->theory_step == NULL )
        {
            n->theory_step = new NFF::Theory_Fragment();
            std::cout << "[验证] 初始化 root 节点的 SAT 理论片段 (t=0)" << std::endl;
            // 添加初始子句
            n->make_root_clauses();
            std::cout << "[验证] root 节点的初始子句构建完成" << std::endl;
        }

        // 3) 回放计划：每步 successor 后都要 do_belief_tracking(samples)，再生成该步 theory
        std::cout << "[验证] 开始回放验证候选规划..." << std::endl;
        bool replay_ok = true;
        for ( unsigned k = 0; k < plan_ops.size(); k++ )
        {
            unsigned op_idx = plan_ops[k];

            std::cout << "[验证] 回放步骤 " << (k+1) << "/" << plan_ops.size() << ": ";
            task.print_operator( op_idx, std::cout );
            std::cout << std::endl;

            // 动作可执行性检查（避免 silent fail）
            if ( n->b == NULL || !n->b->can_apply( op_idx ) )
            {
                std::cout << "[验证] 错误: 动作在步骤 " << (k+1) << " 不可执行！" << std::endl;
                std::cout << "       belief 状态为空或前置条件不满足。" << std::endl;
                replay_ok = false;
                break;
            }

            // 生成后继节点
            n = n->successor( op_idx );

            // 用 samples 更新 belief/unknown/known 等
            n->do_belief_tracking( samples );

            // 生成这一层 SAT theory（依赖 father->theory_step 已存在）
            n->belief_tracking_with_sat();
        }

        if ( !replay_ok )
        {
            std::cout << std::endl;
            std::cout << "[验证] 规划回放失败，终止。" << std::endl;
            break;
        }

        std::cout << "[验证] 回放完成，开始 SAT 目标蕴含测试..." << std::endl;


        bool valid = true;
        std::vector<int> sat_model;
        unsigned failed_goal = 0;

        PDDL::Task& t = PDDL::Task::instance();
        std::vector<unsigned>& G = t.goal_state();


        for ( unsigned gi = 0; gi < G.size(); gi++ )
        {
            unsigned g = G[gi];
            sat_model.clear();

            std::cout << "[验证] 检查目标 " << (gi+1) << "/" << G.size() << ": ";
            t.print_fluent( g, std::cout );

            // If we can find a model that falsifies goal g => counterexample exists => plan invalid
            if ( n->get_counterexample_for_literal( g, sat_model ) )
            {
                valid = false;
                failed_goal = g;

                std::cout << " -> 发现反例!" << std::endl;
                std::cout << "[反例] 目标 ";
                t.print_fluent( failed_goal, std::cout );
                std::cout << " 存在反例状态，规划无效。" << std::endl;
                break;
            }
            else
            {
                std::cout << " -> 已蕴含 ✓" << std::endl;
            }
        }

        if ( valid )
        {
            std::cout << std::endl;
            std::cout << "========================================================" << std::endl;
            std::cout << "=====          [CEGAR] 规划验证成功！             =====" << std::endl;
            std::cout << "========================================================" << std::endl;
            std::cout << std::endl;
            std::cout << "[结果] 所有目标均已蕴含，找到一致性规划！" << std::endl;
            std::cout << std::endl;
            std::cout << "[最终规划] 共 " << plan_ops.size() << " 步:" << std::endl;
            std::cout << "--------------------------------------------------------" << std::endl;
            for ( unsigned k = 0; k < plan_ops.size(); k++ )
            {
                std::cout << "  步骤 " << (k+1) << ": ";
                task.print_operator( plan_ops[k], std::cout );
                std::cout << std::endl;
            }
            std::cout << "--------------------------------------------------------" << std::endl;
            std::cout << std::endl;
            std::cout << "[目标状态] 已达成:" << std::endl;
            PDDL::Task& t2 = PDDL::Task::instance();
            std::vector<unsigned>& goals = t2.goal_state();
            for ( unsigned gi = 0; gi < goals.size(); gi++ )
            {
                std::cout << "  - ";
                t2.print_fluent( goals[gi], std::cout );
                std::cout << " ✓" << std::endl;
            }
            std::cout << std::endl;
            std::cout << "[CEGAR] 迭代次数: " << (it+1) << std::endl;
            std::cout << "[样本集] 最终样本数量: |S|=" << samples.size() << std::endl;
            std::cout << std::endl;
            std::cout << "========================================================" << std::endl;
            std::cout << "=====              规划完成！                     =====" << std::endl;
            std::cout << "========================================================" << std::endl;
            handle_exit();
            std::exit(0);
        }

        // --------
        // Convert SAT model -> a new sample state, then update S
        // --------
        std::vector<int> new_state;
        new_state.resize( t.fluent_count(), 0 );

        // Build state assignment for positive fluents, and fill their neg equivalents accordingly
//        for ( unsigned f = 1; f < (unsigned)(t.fluent_count()-1); f++ )
//        {
//            if ( !t.fluents()[f]->is_pos() ) continue;

//            unsigned v = n->theory_step->fluent_var(f);
//            int assn = 0;
//            if ( v < sat_model.size() )
//                assn = sat_model[v];

//            // If SAT model doesn't provide assignment, skip (keep 0)
//            if ( assn == 0 ) continue;

//            int sign_true = (assn > 0 ? 1 : -1);
//            new_state[f] = sign_true * (int)f;

//            unsigned nf = t.not_equivalent(f);
//            if ( nf != 0 )
//                new_state[nf] = -sign_true * (int)nf;
//        }

        // 2026.1.27
        // Build state assignment for positive fluents, and fill their neg equivalents accordingly
        // IMPORTANT: must project SAT model to t=0 variables using root->theory_step
        unsigned undef_cnt = 0;

        for ( unsigned f = 1; f < (unsigned)(t.fluent_count()-1); f++ )
        {
            if ( !t.fluents()[f]->is_pos() ) continue;

            // *** FIX: use root(t=0) mapping, NOT the last node mapping ***
            unsigned v = root->theory_step->fluent_var( f );

            int assn = 0;
            if ( v < sat_model.size() )
                assn = sat_model[v];

            // 处理 Undef：为了形成一个“完整样本”，这里默认当作 false（并打印统计）
            bool is_true = false;
            if ( assn == 0 )
            {
                undef_cnt++;
                is_true = false;
            }
            else
            {
                is_true = (assn > 0);
            }

            int sign_true = is_true ? 1 : -1;
            new_state[f] = sign_true * (int)f;

            unsigned nf = t.not_equivalent( f );
            if ( nf != 0 )
                new_state[nf] = -sign_true * (int)nf;
        }

        if ( undef_cnt > 0 )
        {
            std::cout << "[反例] 警告: SAT 模型中有 " << undef_cnt
                      << " 个未定义变量 (默认设为 FALSE)" << std::endl;
        }

        // ===== 关键修复：保证反例样本是“完整状态” =====
        // 1) dummy GOAL fluent 必须显式赋值（通常默认 FALSE）；否则可能留下 0 导致后续崩溃/错误
        {
            unsigned goal_id = t.dummy_goal();
            if ( goal_id < new_state.size() )
                new_state[goal_id] = -(int)goal_id;
        }

        // 2) 对所有正 fluent：若仍为 0（undef），默认 FALSE，并同步其 not_equiv
        for ( unsigned f = 1; f < (unsigned)(t.fluent_count()-1); ++f )
        {
            if ( !t.fluents()[f]->is_pos() ) continue;
            if ( new_state[f] == 0 ) new_state[f] = -(int)f;

            unsigned nf = t.not_equivalent( f );
            if ( nf != 0 && nf < new_state.size() )
                new_state[nf] = ( new_state[f] > 0 ? -(int)nf : +(int)nf );
        }

        // 3) 对剩余 fluent：避免任何 0，统一默认 FALSE
        for ( unsigned f = 1; f < (unsigned)new_state.size(); ++f )
            if ( new_state[f] == 0 ) new_state[f] = -(int)f;



        // 判断反例是否为新样本（这里 samples 已在 solve() 后同步过）
        bool is_new = true;
        for ( unsigned si = 0; si < samples.size(); ++si )
        {
            if ( same_state_vec( samples[si], new_state ) )
            {
                is_new = false;
                break;
            }
        }

        std::cout << "[样本集] 当前样本数量(加入反例前): |S|=" << samples.size() << std::endl;

        if ( is_new )
        {
            samples.push_back( new_state );
            std::cout << "[样本集] 添加 1 个反例状态, 新 |S|=" << samples.size() << std::endl;
            dump_samples_summary( samples, task, "after_add_cex" );
        }
        else
        {
            std::cout << "[样本集] 反例与已有样本重复, |S| 不变" << std::endl;
        }

        std::cout << std::endl;
        std::cout << "[CEGAR] 第 " << (it+1) << " 轮迭代完成，继续精化规划..." << std::endl;
        it++;
    }

    std::cout << std::endl;
    std::cout << "========================================================" << std::endl;
    std::cout << "[CEGAR] 达到最大迭代次数或搜索失败，终止。" << std::endl;
    std::cout << "========================================================" << std::endl;
    handle_exit();
    std::exit(0);

}



