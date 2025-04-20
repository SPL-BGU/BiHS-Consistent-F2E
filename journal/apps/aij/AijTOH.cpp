//
// Created by Lior Siag on 16/06/2024.
//

#include <stdexcept>
#include "AijTOH.h"
#include "TOH.h"
#include "CSBS.h"
#include "BAE.h"
#include "BAELB.h"
#include "DBBS.h"
#include "DBBSLB.h"
#include "GMX.h"
#include "TemplateAStar.h"

#define ToHBAELB BAELB<TOHState<N>, TOHMove, TOH<N>>
#define ToHBAELBPair std::pair<std::string, ToHBAELB>

template<int numDisks, int pdb1Disks, int pdb2Disks = numDisks - pdb1Disks>
Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal) {
    TOH<numDisks> toh;
    TOH<pdb1Disks> absToh1;
    TOH<pdb2Disks> absToh2;
    TOHState<pdb1Disks> absTohState1;
    TOHState<pdb2Disks> absTohState2;


    auto *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
    auto *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks
    pdb1->BuildPDB(goal, std::thread::hardware_concurrency(), false);
    pdb2->BuildPDB(goal, std::thread::hardware_concurrency(), false);

    auto *h = new Heuristic<TOHState<numDisks>>;

    h->lookups.resize(0);
    h->lookups.push_back({kAddNode, 1, 2});
    h->lookups.push_back({kLeafNode, 0, 0});
    h->lookups.push_back({kLeafNode, 1, 1});
    h->heuristics.resize(0);
    h->heuristics.push_back(pdb1);
    h->heuristics.push_back(pdb2);

    return h;
}

template<int N>
void GenerateState(TOHState<N> &state, int id) {
    const int table[] = {52058078, 116173544, 208694125, 131936966, 141559500, 133800745, 194246206, 50028346,
                         167007978, 207116816, 163867037, 119897198, 201847476, 210859515, 117688410, 121633885};
    const int table2[] = {145008714, 165971878, 154717942, 218927374, 182772845, 5808407, 19155194, 137438954,
                          13143598,
                          124513215, 132635260, 39667704, 2462244, 41006424, 214146208, 54305743};
    srandom(table[id & 0xF] ^ table2[(id >> 4) & 0xF]);
    state.counts[0] = state.counts[1] = state.counts[2] = state.counts[3] = 0;
    for (int x = N; x > 0; x--) {
        int whichPeg = random() % 4;
        state.disks[whichPeg][state.counts[whichPeg]] = x;
        state.counts[whichPeg]++;
    }
}

template<int N, int TOP_DISKS, int BOTTOM_DISKS = N - TOP_DISKS>
void TestTohOfSize(int instanceId, int numOfInstances, const std::string &alg, const std::vector<double> &csbsWeights) {
    printf("[D] ToH-%d (%d+%d)\n", N, BOTTOM_DISKS, TOP_DISKS);
    Timer timer;
    TOH<N> env;
    TOHState<N> start;
    TOHState<N> goal;
    auto heuristicF = BuildPDB<N, TOP_DISKS, BOTTOM_DISKS>(goal);
    std::vector<TOHState<N>> solutionPath;
    for (int i = instanceId; i < instanceId + numOfInstances; ++i) {
        GenerateState(start, i);
        auto heuristicB = BuildPDB<N, TOP_DISKS, BOTTOM_DISKS>(start);
        std::cout << "[I] ID " << i << " Instance: " << start << std::endl;
        if (alg == "csbs" || alg == "all") {
            CSBS<TOHState<N>, TOHMove, TOH<N>> csbs(csbsWeights);
            timer.StartTimer();
            csbs.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", csbs.GetName().c_str(),
                   env.GetPathLength(solutionPath), csbs.GetNodesExpanded(), csbs.GetNecessaryExpansions(),
                   timer.GetElapsedTime());
        }

        if (alg == "bae" || alg == "all") {
            BAE<TOHState<N>, TOHMove, TOH<N>> bae(SideCriterion::OptCount);
            timer.StartTimer();
            bae.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "BAE-FBI",
                   env.GetPathLength(solutionPath), bae.GetNodesExpanded(), bae.GetNecessaryExpansions(),
                   timer.GetElapsedTime());
        }

        if (alg == "dbbs-max-a" || alg == "dbbs-max" || alg == "all") {
            DBBS<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MaxTot> dbbs(SideCriterion::Alt);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-a-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbs-max-p" || alg == "dbbs-max" || alg == "all") {
            DBBS<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MaxTot> dbbs(SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-p-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbs-max-fbi" || alg == "dbbs-max" || alg == "all") {
            DBBS<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MaxTot> dbbs(SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-fbi-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbsf-max-a" || alg == "dbbsf-max" || alg == "all") {
            DBBSLB<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MaxTot> dbbs(SideCriterion::Alt);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-a-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbsf-max-p" || alg == "dbbsf-max" || alg == "all") {
            DBBSLB<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MaxTot> dbbs(SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-p-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbsf-max-fbi" || alg == "dbbsf-max" || alg == "all") {
            DBBSLB<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MaxTot> dbbs(SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-fbi-MaxTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbs-b-fbi" || alg == "dbbs-b" || alg == "all") {
            DBBS<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinB> dbbs(SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-fbi-b",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbsf-b-fbi" || alg == "dbbsf-b" || alg == "all") {
            DBBSLB<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinB> dbbs(SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-fbi-b",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbs-ss-p" || alg == "dbbs-ss" || alg == "all") {
            DBBS<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinTot> dbbs(SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-p-MinTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbsf-ss-p" || alg == "dbbsf-ss" || alg == "all") {
            DBBSLB<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinTot> dbbs(SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-p-MinTot",
                   env.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }


        std::vector<ToHBAELBPair > baelbs;
        baelbs.emplace_back("tb-fd-fbi", ToHBAELB(ivF, ivD, SideCriterion::OptCount));
        baelbs.emplace_back("tb-df-fbi", ToHBAELB(ivD, ivF, SideCriterion::OptCount));
        baelbs.emplace_back("tb-g-fbi", ToHBAELB(ivG, ivG, SideCriterion::OptCount));
        baelbs.emplace_back("tb-b-fbi", ToHBAELB(ivB, ivB, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gfgd-fbi", ToHBAELB(ivGF, ivGD, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gdgf-fbi", ToHBAELB(ivGD, ivGF, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gb-fbi", ToHBAELB(ivGB, ivGB, SideCriterion::OptCount));
        baelbs.emplace_back("tb-fd-d", ToHBAELB(ivF, ivD, SideCriterion::Decay));
        baelbs.emplace_back("tb-df-d", ToHBAELB(ivD, ivF, SideCriterion::Decay));
        baelbs.emplace_back("tb-g-d", ToHBAELB(ivG, ivG, SideCriterion::Decay));
        baelbs.emplace_back("tb-b-d", ToHBAELB(ivB, ivB, SideCriterion::Decay));
        baelbs.emplace_back("tb-gfgd-d", ToHBAELB(ivGF, ivGD, SideCriterion::Decay));
        baelbs.emplace_back("tb-gdgf-d", ToHBAELB(ivGD, ivGF, SideCriterion::Decay));
        baelbs.emplace_back("tb-gb-d", ToHBAELB(ivGB, ivGB, SideCriterion::Decay));

        for (ToHBAELBPair &solver: baelbs) {
            if (alg == solver.first || alg == "tb" || alg == "all") {
                timer.StartTimer();
                solver.second.GetPath(&env, start, goal, heuristicF, heuristicB, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", solver.first.c_str(),
                       env.GetPathLength(solutionPath), solver.second.GetNodesExpanded(),
                       solver.second.GetNecessaryExpansions(), timer.GetElapsedTime());
            }
        }

        if (alg == "gmxc" || alg == "all") {
            GMX<TOHState<N>> gmx;
            timer.StartTimer();
            {
                TemplateAStar<TOHState<N>, TOHMove, TOH<N>> astar;
                astar.SetHeuristic(heuristicF);
                astar.GetPath(&env, start, goal, solutionPath);
                gmx.GenerateBuckets(astar.openClosedList.elements, heuristicB, start, env.GetPathLength(solutionPath),
                                    true);
            }

            {
                TemplateAStar<TOHState<N>, TOHMove, TOH<N>> rastar;
                rastar.SetHeuristic(heuristicB);
                rastar.GetPath(&env, goal, start, solutionPath);
                gmx.GenerateBuckets(rastar.openClosedList.elements, heuristicF, goal, env.GetPathLength(solutionPath),
                                    false);
            }
            int expanded = gmx.GetMinimalExpansions();
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %d; necessary %d; time %1.2fs\n", "GMXC",
                   gmx.GetSolutionCost(), expanded, expanded, timer.GetElapsedTime());
        }
    }
}

void GetPdbSizes(const std::string &heuristic, int &bottom, int &top) {
    auto heuristic_split_loc = heuristic.find('+');
    bottom = std::stoi(std::string(&heuristic[0], &heuristic[heuristic_split_loc]));
    top = std::stoi(std::string(&heuristic[heuristic_split_loc], &heuristic[heuristic.size()]));
}

void TestToh(const struct ArgParameters &ap) {
    int bottom, top;
    GetPdbSizes(ap.heuristic, bottom, top);
    if (bottom + top != 12) {
        throw std::invalid_argument("PDB size does not add up to domain size");
    }

    switch (top) {
        case 2:
            TestTohOfSize<12, 2>(ap.instanceId, ap.numOfInstances, ap.alg, ap.weights);
            break;
        case 4:
            TestTohOfSize<12, 4>(ap.instanceId, ap.numOfInstances, ap.alg, ap.weights);
            break;
        case 6:
            TestTohOfSize<12, 6>(ap.instanceId, ap.numOfInstances, ap.alg, ap.weights);
            break;
        default:
            assert(!"Non-programmed PDB value");
    }
}
