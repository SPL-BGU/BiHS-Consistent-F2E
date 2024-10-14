//
// Created by Lior Siag on 13/05/2024.
//

#include "AijPancake.h"
#include "PancakePuzzle.h"
#include "BAE.h"
#include "CSBS.h"
#include "Utilities.h"
#include "MinCriterion.h"
#include "DBBS.h"
#include "BAELB.h"
#include "DBBSLB.h"
#include "TemplateAStar.h"
#include "GMX.h"

const int N = 14;

#define PancakeBAELB BAELB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>>
#define PancakeBAELBPair std::pair<std::string, PancakeBAELB>

template<int N>
PancakePuzzleState<N> GetRandomInstance(int instanceId) {
    // Find the correct seed for the current instance
    PancakePuzzleState<N> state;
    srandom(0);
    for (int count = 0; count < instanceId; count++) {
        srandom(random());
        for (int x = 0; x < N; x++) {
            random();
        }
    }

    // Initialize the current instance
    srandom(random());
    state.Reset();
    for (int x = 0; x < N; x++) {
        std::swap(state.puzzle[x], state.puzzle[x + random() % (N - x)]);
    }
    return state;
}

int GetGap(const std::string &h) {
    return std::stoi(h.substr(h.find('-') + 1));
}

template<int N>
void TestPancakeOfSize(int instanceId, int numOfInstances, int gap,
                       const std::string &alg, const std::vector<double> &csbsWeights) {
    printf("[D] Pancake-%d GAP-%d\n", N, gap);
    Timer timer;
    PancakePuzzle<N> env(gap);
    PancakePuzzleState<N> start;
    PancakePuzzleState<N> goal;
    std::vector<PancakePuzzleState<N>> solutionPath;
    for (int i = instanceId; i < instanceId + numOfInstances; ++i) {
        start = GetRandomInstance<N>(i);
        std::cout << "[I] ID " << i << " Instance: " << start << std::endl;
        if (alg == "csbs" || alg == "all") {
            CSBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> csbs(csbsWeights);
            timer.StartTimer();
            csbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", csbs.GetName().c_str(),
                   env.GetPathLength(solutionPath), csbs.GetNodesExpanded(), csbs.GetNecessaryExpansions(),
                   timer.GetElapsedTime());
        }
        if (alg == "bae" || alg == "all") {
            BAE<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> bae(SideCriterion::OptCount);
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "BAE-FBI",
                   env.GetPathLength(solutionPath), bae.GetNodesExpanded(), bae.GetNecessaryExpansions(),
                   timer.GetElapsedTime());
        }


        if (alg == "dbbs-max-a" || alg == "dbbs-max" || alg == "all") {
            DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MaxTot> dbbs(
                    SideCriterion::Alt);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-a-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }


        if (alg == "dbbs-max-p" || alg == "dbbs-max" || alg == "all") {
            DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MaxTot> dbbs(
                    SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-p-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbs-max-fbi" || alg == "dbbs-max" || alg == "all") {
            DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MaxTot> dbbs(
                    SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-fbi-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbsf-max-a" || alg == "dbbsf-max" || alg == "all") {
            DBBSLB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MaxTot> dbbs(
                    SideCriterion::Alt);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-a-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }


        if (alg == "dbbsf-max-p" || alg == "dbbsf-max" || alg == "all") {
            DBBSLB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MaxTot> dbbs(
                    SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-p-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbsf-max-fbi" || alg == "dbbsf-max" || alg == "all") {
            DBBSLB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MaxTot> dbbs(
                    SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-fbi-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbs-b-fbi" || alg == "dbbs-b" || alg == "all") {
            DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinB> dbbs(
                    SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-fbi-b",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbsf-b-fbi" || alg == "dbbsf-b" || alg == "all") {
            DBBSLB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinB> dbbs(
                    SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-fbi-b",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbs-ss-p" || alg == "dbbs-ss" || alg == "all") {
            DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinTot> dbbs(
                    SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-p-MinTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbsf-ss-p" || alg == "dbbsf-ss" || alg == "all") {
            DBBSLB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinTot> dbbs(
                    SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-p-MinTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        std::vector<PancakeBAELBPair > baelbs;
        baelbs.emplace_back("tb-fd-fbi", PancakeBAELB(ivF, ivD, SideCriterion::OptCount));
        baelbs.emplace_back("tb-df-fbi", PancakeBAELB(ivD, ivF, SideCriterion::OptCount));
        baelbs.emplace_back("tb-g-fbi", PancakeBAELB(ivG, ivG, SideCriterion::OptCount));
        baelbs.emplace_back("tb-b-fbi", PancakeBAELB(ivB, ivB, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gfgd-fbi", PancakeBAELB(ivGF, ivGD, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gdgf-fbi", PancakeBAELB(ivGD, ivGF, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gb-fbi", PancakeBAELB(ivGB, ivGB, SideCriterion::OptCount));

        for (PancakeBAELBPair &solver: baelbs) {
            if (alg == solver.first || alg == "tb" || alg == "all") {
                timer.StartTimer();
                solver.second.GetPath(&env, start, goal, &env, &env, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", solver.first.c_str(),
                       env.GetPathLength(solutionPath), solver.second.GetNodesExpanded(),
                       solver.second.GetNecessaryExpansions(), timer.GetElapsedTime());
            }
        }

        if (alg == "gmxc" || alg == "all") {
            GMX<PancakePuzzleState<N>> gmx;
            timer.StartTimer();
            {
                TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar;
                std::vector<PancakePuzzleState<N>> path;
                astar.GetPath(&env, start, goal, path);
                gmx.GenerateBuckets(astar.openClosedList.elements, &env, start, env.GetPathLength(path), true);
            }

            {
                TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> rastar;
                std::vector<PancakePuzzleState<N>> path;
                rastar.GetPath(&env, goal, start, path);
                gmx.GenerateBuckets(rastar.openClosedList.elements, &env, goal, env.GetPathLength(path), false);
            }
            int expanded = gmx.GetMinimalExpansions();
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %d; necessary %d; time %1.2fs\n", "GMXC",
                   gmx.GetSolutionCost(), expanded, expanded, timer.GetElapsedTime());
        }
    }
}

void TestPancake(const struct ArgParameters &argParameters) {
    int gap = GetGap(argParameters.heuristic);
    TestPancakeOfSize<14>(argParameters.instanceId, argParameters.numOfInstances, gap,
                          argParameters.alg, argParameters.weights);
}
