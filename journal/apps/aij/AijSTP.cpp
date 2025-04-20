//
// Created by Lior Siag on 15/07/2024.
//

#include <stdexcept>
#include "AijSTP.h"
#include "Timer.h"
#include "MNPuzzle.h"
#include "STPInstances.h"
#include "CSBS.h"
#include "BAE.h"
#include "MinCriterion.h"
#include "DBBS.h"
#include "BAELB.h"
#include "DBBSLB.h"
#include "GMX.h"
#include "TemplateAStar.h"

#define StpBAELB BAELB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4 >>
#define StpBAELBPair std::pair<std::string, StpBAELB>

void TestStp4(const struct ArgParameters &ap) {
//        int instanceId, int numOfInstances, const std::string &alg, const std::vector<double> &csbsWeights) {
    puts("[D] STP4 MD");
    Timer timer;
    MNPuzzle<4, 4> env;
    MNPuzzleState<4, 4> start;
    MNPuzzleState<4, 4> goal;
    std::vector<MNPuzzleState<4, 4>> solutionPath;
    for (int i = ap.instanceId; i < ap.instanceId + ap.numOfInstances; ++i) {
        start = STP::GetKorfInstance(i);
        std::cout << "[I] ID " << i << " Instance: " << start << std::endl;
        if (ap.alg == "csbs" || ap.alg == "all") {
            CSBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> csbs(ap.weights);
            timer.StartTimer();
            csbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", csbs.GetName().c_str(),
                   env.GetPathLength(solutionPath), csbs.GetNodesExpanded(), csbs.GetNecessaryExpansions(),
                   timer.GetElapsedTime());
        }

        if (ap.alg == "bae" || ap.alg == "all") {
            BAE<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> bae(SideCriterion::OptCount);
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "BAE-FBI",
                   env.GetPathLength(solutionPath), bae.GetNodesExpanded(), bae.GetNecessaryExpansions(),
                   timer.GetElapsedTime());
        }

        if (ap.alg == "dbbs-max-a" || ap.alg == "dbbs-max" || ap.alg == "all") {
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MaxTot> dbbs(SideCriterion::Alt);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-a-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (ap.alg == "dbbs-max-p" || ap.alg == "dbbs-max" || ap.alg == "all") {
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MaxTot> dbbs(SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-p-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (ap.alg == "dbbs-max-fbi" || ap.alg == "dbbs-max" || ap.alg == "all") {
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MaxTot> dbbs(SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-fbi-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (ap.alg == "dbbsf-max-a" || ap.alg == "dbbsf-max" || ap.alg == "all") {
            DBBSLB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MaxTot> dbbs(SideCriterion::Alt);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-a-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (ap.alg == "dbbsf-max-p" || ap.alg == "dbbsf-max" || ap.alg == "all") {
            DBBSLB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MaxTot> dbbs(
                    SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-p-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (ap.alg == "dbbsf-max-fbi" || ap.alg == "dbbsf-max" || ap.alg == "all") {
            DBBSLB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MaxTot> dbbs(SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-fbi-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (ap.alg == "dbbs-b-fbi" || ap.alg == "dbbs-b" || ap.alg == "all") {
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinB> dbbs(SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-fbi-b",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (ap.alg == "dbbsf-b-fbi" || ap.alg == "dbbsf-b" || ap.alg == "all") {
            DBBSLB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinB> dbbs(SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-fbi-b",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (ap.alg == "dbbs-ss-p" || ap.alg == "dbbs-ss" || ap.alg == "all") {
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinTot> dbbs(SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-p-MinTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (ap.alg == "dbbsf-ss-p" || ap.alg == "dbbsf-ss" || ap.alg == "all") {
            DBBSLB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinTot> dbbs(
                    SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-p-MinTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }


        std::vector<StpBAELBPair > baelbs;
        baelbs.emplace_back("tb-fd-fbi", StpBAELB(ivF, ivD, SideCriterion::OptCount));
        baelbs.emplace_back("tb-df-fbi", StpBAELB(ivD, ivF, SideCriterion::OptCount));
        baelbs.emplace_back("tb-g-fbi", StpBAELB(ivG, ivG, SideCriterion::OptCount));
        baelbs.emplace_back("tb-b-fbi", StpBAELB(ivB, ivB, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gfgd-fbi", StpBAELB(ivGF, ivGD, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gdgf-fbi", StpBAELB(ivGD, ivGF, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gb-fbi", StpBAELB(ivGB, ivGB, SideCriterion::OptCount));
        baelbs.emplace_back("tb-fd-d", StpBAELB(ivF, ivD, SideCriterion::Decay));
        baelbs.emplace_back("tb-df-d", StpBAELB(ivD, ivF, SideCriterion::Decay));
        baelbs.emplace_back("tb-g-d", StpBAELB(ivG, ivG, SideCriterion::Decay));
        baelbs.emplace_back("tb-b-d", StpBAELB(ivB, ivB, SideCriterion::Decay));
        baelbs.emplace_back("tb-gfgd-d", StpBAELB(ivGF, ivGD, SideCriterion::Decay));
        baelbs.emplace_back("tb-gdgf-d", StpBAELB(ivGD, ivGF, SideCriterion::Decay));
        baelbs.emplace_back("tb-gb-d", StpBAELB(ivGB, ivGB, SideCriterion::Decay));

        for (StpBAELBPair &solver: baelbs) {
            if (ap.alg == solver.first || ap.alg == "tb" || ap.alg == "all") {
                timer.StartTimer();
                solver.second.GetPath(&env, start, goal, &env, &env, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", solver.first.c_str(),
                       env.GetPathLength(solutionPath), solver.second.GetNodesExpanded(),
                       solver.second.GetNecessaryExpansions(), timer.GetElapsedTime());
            }
        }

        if (ap.alg == "gmxc" || ap.alg == "all") {
            std::cout << "[A] GMX; NB\n";
            GMX<MNPuzzleState<4, 4>> gmx;
            timer.StartTimer();
            {
                TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
                astar.GetPath(&env, start, goal, solutionPath);
                gmx.GenerateBuckets(astar.openClosedList.elements, &env, start, env.GetPathLength(solutionPath), true);
            }

            {
                TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> rastar;
                goal.Reset();
                rastar.GetPath(&env, goal, start, solutionPath);
                gmx.GenerateBuckets(rastar.openClosedList.elements, &env, goal, env.GetPathLength(solutionPath), false);
            }
            int expanded = gmx.GetMinimalExpansions();
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %d; necessary %d; time %1.2fs\n", "GMXC",
                   gmx.GetSolutionCost(), expanded, expanded, timer.GetElapsedTime());
        }
    }


}

void TestStp(const struct ArgParameters &ap) {
    if (ap.heuristic != "md") {
        throw std::invalid_argument(ERROR_MSG("Currently only MD is a defined heuristic"));
    }

    TestStp4(ap);
}
