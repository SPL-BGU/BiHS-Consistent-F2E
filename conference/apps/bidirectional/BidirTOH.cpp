//
//  BidirTOH.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/14/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirTOH.h"
#include "TOH.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "DVCBS.h"
#include "MM.h"
#include "BSStar.h"
#include "BAE.h"
#include "Baseline.h"
#include "GBFHS.h"
#include "DBBS.h"
#include "BTB.h"
#include "CalculateWVC.h"
#include "BAELB.h"
#include "GMX.h"
#include "DBBSLB.h"

#define ToHBAELBPair std::pair<std::string, BAELB<TOHState<N>, TOHMove, TOH<N>>>
#define ToHBAELB BAELB<TOHState<N>, TOHMove, TOH<N>>


template<int numDisks, int pdb1Disks, int pdb2Disks = numDisks - pdb1Disks>
Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal) {
    TOH<numDisks> toh;
    TOH<pdb1Disks> absToh1;
    TOH<pdb2Disks> absToh2;
    TOHState<pdb1Disks> absTohState1;
    TOHState<pdb2Disks> absTohState2;


    auto *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
    auto *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks
    pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
    pdb2->BuildPDB(goal, std::thread::hardware_concurrency());

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

template<int N, int pdb1Disks>
void TestTOH(ArgParameters ap) {
    BSStar<TOHState<N>, TOHMove, TOH<N >> bs;
    MM<TOHState<N>, TOHMove, TOH<N>> mm;


    TOH<N> toh;
    TOHState<N> s;
    TOHState<N> g;
    std::vector<TOHState<N>> thePath;
    Heuristic<TOHState<N>> *f;
    Heuristic<TOHState<N>> *b;

    int table[] = {52058078, 116173544, 208694125, 131936966, 141559500, 133800745, 194246206, 50028346, 167007978,
                   207116816, 163867037, 119897198, 201847476, 210859515, 117688410, 121633885};
    int table2[] = {145008714, 165971878, 154717942, 218927374, 182772845, 5808407, 19155194, 137438954, 13143598,
                    124513215, 132635260, 39667704, 2462244, 41006424, 214146208, 54305743};

    for (int count = ap.instanceId; count < ap.instanceId + ap.numOfInstances; count++) {

        srandom(table[count & 0xF] ^ table2[(count >> 4) & 0xF]);

        s.counts[0] = s.counts[1] = s.counts[2] = s.counts[3] = 0;
        for (int x = N; x > 0; x--) {
            int whichPeg = random() % 4;
            s.disks[whichPeg][s.counts[whichPeg]] = x;
            s.counts[whichPeg]++;
        }

        b = BuildPDB<N, pdb1Disks>(s);

        g.counts[0] = g.counts[1] = g.counts[2] = g.counts[3] = 0;
        for (int x = N; x > 0; x--) {
            int whichPeg = random() % 4;
            g.disks[whichPeg][g.counts[whichPeg]] = x;
            g.counts[whichPeg]++;
        }
        g.Reset();
        f = BuildPDB<N, pdb1Disks>(g);

        Timer timer;

        std::cout << "[I] PDB-" << pdb1Disks << " (ToH problem: " << count << " of "
                  << (ap.instanceId + ap.numOfInstances) << ") Stacks: " << s << std::endl;

        double optimal_cost = -1.0;

        // BAE*-o-a
        if (ap.HasAlgorithm("BAE*-o-a")) {
            BAE<TOHState<N>, TOHMove, TOH<N >> bae(true);
            std::cout << "[A] BAE*-o-a; NB\n";
            timer.StartTimer();
            bae.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), bae.GetNodesExpanded(),
                   bae.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("BAE*-o-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        // BAE*-o-p
        if (ap.HasAlgorithm("BAE*-o-p")) {
            BAE<TOHState<N>, TOHMove, TOH<N >> bae(false);
            std::cout << "[A] BAE*-o-p; NB\n";
            timer.StartTimer();
            bae.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), bae.GetNodesExpanded(),
                   bae.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("BAE*-o-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        // A*
        if (ap.HasAlgorithm("A*")) {
            TemplateAStar<TOHState<N>, TOHMove, TOH<N>> astar;
            std::cout << "[A] A*; NB\n";
            astar.SetHeuristic(f);
            timer.StartTimer();
            astar.GetPath(&toh, s, g, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), astar.GetNodesExpanded(),
                   astar.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("A* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //NBS
        if (ap.HasAlgorithm("NBS")) {
            NBS<TOHState<N>, TOHMove, TOH<N>, NBSQueue<TOHState<N>, 1, true >> nbs(false, true);
            std::cout << "[A] NBS; NB\n";
            timer.StartTimer();
            nbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), nbs.GetNodesExpanded(),
                   nbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("NBS-E-LEQ reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }

        }

        //DVCBS
        if (ap.HasAlgorithm("DVCBS")) {
            DVCBS<TOHState<N>, TOHMove, TOH<N>, DVCBSQueue<TOHState<N>, 1, false >> dvcbs(false, true);
            std::cout << "[A] DVCBS; NB\n";
            timer.StartTimer();
            dvcbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dvcbs.GetNodesExpanded(),
                   dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DVCBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBS-a
        if (ap.HasAlgorithm("DBBS-a")) {
            DBBS<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinB> dbbs(SideCriterion::Alt);
            std::cout << "[A] DBBS-a; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBS-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBS-p
        if (ap.HasAlgorithm("DBBS-p")) {
            DBBS<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinB> dbbs(SideCriterion::Cardinality);
            std::cout << "[A] DBBS-p; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBS-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBS-a-MaxTot
        if (ap.HasAlgorithm("DBBS-a-MaxTot")) {
            DBBS<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MaxTot> dbbs(SideCriterion::Alt);
            std::cout << "[A] DBBS-a-MaxTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] DBBS-a-MaxTot found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBS-a-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBS-a-MinTot
        if (ap.HasAlgorithm("DBBS-a-MinTot")) {
            DBBS<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinTot> dbbs(SideCriterion::Alt);
            std::cout << "[A] DBBS-a-MinTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBS-a-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBS-o-MaxTot
        if (ap.HasAlgorithm("DBBS-o-MaxTot")) {
            DBBS<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MaxTot> dbbs(SideCriterion::OptCount);
            std::cout << "[A] DBBS-o-MaxTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBS-o-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBS-o-MinTot
        if (ap.HasAlgorithm("DBBS-o-MinTot")) {
            DBBS<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinTot> dbbs(SideCriterion::OptCount);
            std::cout << "[A] DBBS-o-MinTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBS-o-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBSLB-a
        if (ap.HasAlgorithm("DBBSLB-a")) {
            DBBSLB<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinB> dbbslb(SideCriterion::Alt);
            std::cout << "[A] DBBSLB-a; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBSLB-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBSLB-p
        if (ap.HasAlgorithm("DBBSLB-p")) {
            DBBSLB<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinB> dbbslb(SideCriterion::Cardinality);
            std::cout << "[A] DBBSLB-p; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBSLB-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBSLB-a-MaxTot
        if (ap.HasAlgorithm("DBBSLB-a-MaxTot")) {
            DBBSLB<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MaxTot> dbbslb(SideCriterion::Alt);
            std::cout << "[A] DBBSLB-a-MaxTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBSLB-a-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBSLB-a-MinTot
        if (ap.HasAlgorithm("DBBSLB-a-MinTot")) {
            DBBSLB<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinTot> dbbslb(SideCriterion::Alt);
            std::cout << "[A] DBBSLB-a-MinTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBSLB-a-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBSLB-o-MaxTot
        if (ap.HasAlgorithm("DBBSLB-o-MaxTot")) {
            DBBSLB<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MaxTot> dbbslb(SideCriterion::OptCount);
            std::cout << "[A] DBBSLB-o-MaxTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBSLB-o-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //DBBSLB-o-MinTot
        if (ap.HasAlgorithm("DBBSLB-o-MinTot")) {
            DBBSLB<TOHState<N>, TOHMove, TOH<N>, MinCriterion::MinTot> dbbslb(SideCriterion::OptCount);
            std::cout << "[A] DBBSLB-o-MinTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("DBBSLB-o-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        // GMX
        if (ap.HasAlgorithm("GMX")) {
            std::cout << "[A] GMX; NB\n";
            GMX<TOHState<N>> gmx;
            timer.StartTimer();
            {
                TemplateAStar<TOHState<N>, TOHMove, TOH<N>> astar;
                astar.SetHeuristic(f);
                astar.GetPath(&toh, s, g, thePath);
                gmx.GenerateBuckets(astar.openClosedList.elements, b, s, toh.GetPathLength(thePath), true);
            }

            {
                TemplateAStar<TOHState<N>, TOHMove, TOH<N>> rastar;
                rastar.SetHeuristic(b);
                rastar.GetPath(&toh, g, s, thePath);
                gmx.GenerateBuckets(rastar.openClosedList.elements, f, g, toh.GetPathLength(thePath), false);
            }
            int expanded = gmx.GetMinimalExpansions();
            timer.EndTimer();
            printf("[R] Path length %1.0f; %d expanded; %d necessary; %1.2fs elapsed\n",
                   optimal_cost, expanded, expanded, timer.GetElapsedTime());
        }

        //NGMX
        if (ap.HasAlgorithm("NGMX")) {
            CalculateWVC<TOHState<N>> calculateWVC;
            double C = -1;
            std::map<int, int> gCountMapForwardSingle;
            std::map<int, int> gCountMapBackwardSingle;
            std::vector<double> times;
            timer.StartTimer();
            {
                TemplateAStar<TOHState<N>, TOHMove, TOH<N>> astar;
                astar.SetHeuristic(f);
                astar.GetPath(&toh, s, g, thePath);
                C = toh.GetPathLength(thePath);
                gCountMapForwardSingle = calculateWVC.initGCountMap(astar.openClosedList.elements, C);
            }
            timer.EndTimer();
            times.push_back(timer.GetElapsedTime());

            {
                timer.StartTimer();
                TemplateAStar<TOHState<N>, TOHMove, TOH<N>> rastar;
                rastar.SetHeuristic(b);
                rastar.GetPath(&toh, g, s, thePath);
                if (toh.GetPathLength(thePath) != C) {
                    assert(!"Unequal astar and raster paths");
                }
                timer.EndTimer();
                times.push_back(timer.GetElapsedTime());
                std::cout << "[A] RA*; NB\n";
                printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                       C, rastar.GetNodesExpanded(), rastar.GetNecessaryExpansions(), timer.GetElapsedTime());
                gCountMapBackwardSingle = calculateWVC.initGCountMap(rastar.openClosedList.elements, C);
            }
            timer.StartTimer();
            int expanded = calculateWVC.CalcWVC(gCountMapForwardSingle, gCountMapBackwardSingle, C, 1, false);
            timer.EndTimer();
            std::cout << "[A] NGMX; NB\n";
            printf("[R] Path length %1.0f; %d expanded; %d necessary; %1.2fs elapsed\n",
                   C, expanded, expanded, times[0] + times[1] + timer.GetElapsedTime());
        }

        //BTB-conn
        if (ap.HasAlgorithm("BTB-conn")) {
            BTB<TOHState<N>, TOHMove, TOH<N >> btb(BTBPolicy::MostConnected);
            std::cout << "[A] BTB-conn; NB\n";
            timer.StartTimer();
            btb.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), btb.GetNodesExpanded(),
                   btb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("BTB-conn reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }

        //BTB-nbs
        if (ap.HasAlgorithm("BTB-nbs")) {
            BTB<TOHState<N>, TOHMove, TOH<N >> btb(BTBPolicy::Alternating);
            std::cout << "[A] BTB-nbs; NB\n";
            timer.StartTimer();
            btb.GetPath(&toh, s, g, f, b, thePath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   toh.GetPathLength(thePath), btb.GetNodesExpanded(),
                   btb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
            else if (optimal_cost != toh.GetPathLength(thePath)) {
                printf("BTB-nbs reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, toh.GetPathLength(thePath));
                exit(0);
            }
        }


        std::vector<ToHBAELBPair > baelbs;
        baelbs.push_back(ToHBAELBPair("BAE*-fd-a", ToHBAELB(ivF, ivD)));
        baelbs.push_back(ToHBAELBPair("BAE*-fd-p", ToHBAELB(ivF, ivD, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-rfrd-a", ToHBAELB(ivRF, ivRD)));
        baelbs.push_back(ToHBAELBPair("BAE*-rfrd-p", ToHBAELB(ivRF, ivRD, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-df-a", ToHBAELB(ivD, ivF)));
        baelbs.push_back(ToHBAELBPair("BAE*-df-p", ToHBAELB(ivD, ivF, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-rdrf-a", ToHBAELB(ivRD, ivRF)));
        baelbs.push_back(ToHBAELBPair("BAE*-rdrf-p", ToHBAELB(ivRD, ivRF, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-g-a", ToHBAELB(ivG)));
        baelbs.push_back(ToHBAELBPair("BAE*-g-p", ToHBAELB(ivG, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-b-a", ToHBAELB(ivB)));
        baelbs.push_back(ToHBAELBPair("BAE*-b-p", ToHBAELB(ivB, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-frdrfd-a", ToHBAELB(ivFRD, ivRFD)));
        baelbs.push_back(ToHBAELBPair("BAE*-frdrfd-p", ToHBAELB(ivFRD, ivRFD, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-gfgd-a", ToHBAELB(ivGF, ivGD)));
        baelbs.push_back(ToHBAELBPair("BAE*-gfgd-p", ToHBAELB(ivGF, ivGD, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-rfdfrd-a", ToHBAELB(ivRFD, ivFRD)));
        baelbs.push_back(ToHBAELBPair("BAE*-rfdfrd-p", ToHBAELB(ivRFD, ivFRD, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-rfrdx2-a", ToHBAELB(ivRFRD, ivRFRD)));
        baelbs.push_back(ToHBAELBPair("BAE*-rfrdx2-p", ToHBAELB(ivRFRD, ivRFRD, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-grfgrd-a", ToHBAELB(ivGRF, ivGRD)));
        baelbs.push_back(ToHBAELBPair("BAE*-grfgrd-p", ToHBAELB(ivGRF, ivGRD, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-gdgf-a", ToHBAELB(ivGD, ivGF)));
        baelbs.push_back(ToHBAELBPair("BAE*-gdgf-p", ToHBAELB(ivGD, ivGF, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-grdgrf-a", ToHBAELB(ivGRD, ivGRF)));
        baelbs.push_back(ToHBAELBPair("BAE*-grdgrf-p", ToHBAELB(ivGRD, ivGRF, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-gb-a", ToHBAELB(ivGB)));
        baelbs.push_back(ToHBAELBPair("BAE*-gb-p", ToHBAELB(ivGB, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-gfrdgrfd-a", ToHBAELB(ivGFRD, ivGRFD)));
        baelbs.push_back(ToHBAELBPair("BAE*-gfrdgrfd-p", ToHBAELB(ivGFRD, ivGRFD, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-grfdgfrd-a", ToHBAELB(ivGRFD, ivGFRD)));
        baelbs.push_back(ToHBAELBPair("BAE*-grfdgfrd-p", ToHBAELB(ivGRFD, ivGFRD, false)));
        baelbs.push_back(ToHBAELBPair("BAE*-grfrd-a", ToHBAELB(ivGRFRD)));
        baelbs.push_back(ToHBAELBPair("BAE*-grfrd-p", ToHBAELB(ivGRFRD, false)));


        for (ToHBAELBPair &solver: baelbs) {
            if (ap.HasAlgorithm(solver.first)) {
                std::cout << "[A] " << solver.first.c_str() << "; WB\n";
                timer.StartTimer();
                solver.second.GetPath(&toh, s, g, f, b, thePath);
                timer.EndTimer();
                printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                       toh.GetPathLength(thePath), solver.second.GetNodesExpanded(),
                       solver.second.GetNecessaryExpansions(), timer.GetElapsedTime());

                // test optimality
                if (optimal_cost < 0.0) optimal_cost = toh.GetPathLength(thePath);
                else if (optimal_cost != toh.GetPathLength(thePath)) {
                    printf("%s reported bad value!! optimal %1.0f; reported %1.0f;\n", solver.first.c_str(),
                           optimal_cost, toh.GetPathLength(thePath));
                    exit(0);
                }
                solver.second.ClearMemory();
            }

        }


        while (b->heuristics.size() > 0) {
            delete b->heuristics.back();
            b->heuristics.pop_back();
        }
        delete b;
        while (f->heuristics.size() > 0) {
            delete f->heuristics.back();
            f->heuristics.pop_back();
        }
        delete f;
    }

}

void TestTOH(const ArgParameters &ap) {
    int pdb = stoi(ap.heuristic);
    switch (pdb) {
        case 2:
            TestTOH<12, 2>(ap);
            break;
        case 4:
            TestTOH<12, 4>(ap);
            break;
        case 6:
            TestTOH<12, 6>(ap);
            break;
        case 8:
            TestTOH<12, 8>(ap);
            break;
        default:
            assert(!"Non-programmed PDB value");
    }
    exit(0);
}
