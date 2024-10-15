//
//  BidirPancake.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/7/17.
//  Copyright Â© 2017 University of Denver. All rights reserved.
//

#include "BidirPancake.h"
#include "PancakePuzzle.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "Baseline.h"
#include "GBFHS.h"
#include "DBBS.h"
#include "DBBSLB.h"
#include "BTB.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "BAE.h"
#include "BAELB.h"
#include "DVCBS.h"
#include "PancakeInstances.h"
#include "CalculateWVC.h"
#include "fMM.h"
#include "AStarMultiOpenClosed.h"
#include "GMX.h"

#define PancakeBAELBPair std::pair<std::string, BAELB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>>>
#define PancakeBAELB BAELB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>>

const int N = 14;


void TestSingleInstance(const ArgParameters &ap, int currId) {
    int gap = std::stoi(ap.heuristic);
    PancakePuzzleState<N> start;
    PancakePuzzleState<N> original;
    PancakePuzzleState<N> goal;
    PancakePuzzle<N> pancake(gap);
    PancakePuzzle<N> pancake2(gap);

    std::vector<PancakePuzzleState<N>> baelbPath;
    std::vector<PancakePuzzleState<N>> baePath;

    Timer t1;

    // Find the correct seed for the current instance
    srandom(0);
    for (int count = 0; count < currId; count++) {
        srandom(random());
        for (int x = 0; x < N; x++) {
            random();
        }
    }

    // Initialize the current instance
    srandom(random());
    goal.Reset();
    original.Reset();
    for (int x = 0; x < N; x++) {
        std::swap(original.puzzle[x], original.puzzle[x + random() % (N - x)]);
    }

    std::cout << "[I] GAP-" << gap << " (Pancake problem: " << currId << " of "
              << (ap.instanceId + ap.numOfInstances) << ") Stack: " << original << std::endl;

    double optimal_cost = -1.0;

    // BAE*-o-a
    if (ap.HasAlgorithm("BAE*-o-a")) {
        BAE<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N >> bae(true);
        start = original;
        std::cout << "[A] BAE*-o-a; NB\n";
        t1.StartTimer();
        bae.GetPath(&pancake, start, goal, &pancake, &pancake2, baePath);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(baePath), bae.GetNodesExpanded(),
               bae.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(baePath);
        else if (optimal_cost != pancake.GetPathLength(baePath)) {
            printf("BAE*-o-a reported bad value!! optimal %1.0f; reported %1.0f;\n", optimal_cost,
                   pancake.GetPathLength(baePath));
            exit(0);
        }
    }

    // BAE*-o-p
    if (ap.HasAlgorithm("BAE*-o-p")) {
        BAE<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N >> bae(false);
        start = original;
        std::cout << "[A] BAE*-o-p; NB\n";
        t1.StartTimer();
        bae.GetPath(&pancake, start, goal, &pancake, &pancake2, baePath);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; NB\n",
               pancake.GetPathLength(baePath), bae.GetNodesExpanded(),
               bae.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(baePath);
        else if (optimal_cost != pancake.GetPathLength(baePath)) {
            printf("BAE*-o-p reported bad value!! optimal %1.0f; reported %1.0f;\n", optimal_cost,
                   pancake.GetPathLength(baePath));
            exit(0);
        }
    }

    //A*
    if (ap.HasAlgorithm("A*")) {
        TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar;
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] A*; NB\n";
        t1.StartTimer();
        astar.GetPath(&pancake, start, goal, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), astar.GetNodesExpanded(),
               astar.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("A* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //NBS
    if (ap.HasAlgorithm("NBS")) {
        NBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, NBSQueue<PancakePuzzleState<N>, 1, false >> nbs(
                false, true);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] NBS; NB\n";
        t1.StartTimer();
        nbs.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), nbs.GetNodesExpanded(),
               nbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("NBS reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DVCBS
    if (ap.HasAlgorithm("DVCBS")) {
        DVCBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, DVCBSQueue<PancakePuzzleState<N>, 1, false >> dvcbs(
                false, true);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DVCBS; NB\n";
        t1.StartTimer();
        dvcbs.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dvcbs.GetNodesExpanded(),
               dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DVCBS reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBS-a
    if (ap.HasAlgorithm("DBBS-a")) {
        DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinB> dbbs(SideCriterion::Alt);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBS-a; NB\n";
        t1.StartTimer();
        dbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbs.GetNodesExpanded(),
               dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBS-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBS-p
    if (ap.HasAlgorithm("DBBS-p")) {
        DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinB> dbbs(
                SideCriterion::Cardinality);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBS-p; NB\n";
        t1.StartTimer();
        dbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbs.GetNodesExpanded(),
               dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBS-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBS-a-MaxTot
    if (ap.HasAlgorithm("DBBS-a-MaxTot")) {
        DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MaxTot> dbbs(
                SideCriterion::Alt);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBS-a-MaxTot; NB\n";
        t1.StartTimer();
        dbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbs.GetNodesExpanded(),
               dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBS-a-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBS-a-MinTot
    if (ap.HasAlgorithm("DBBS-a-MinTot")) {
        DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinTot> dbbs(
                SideCriterion::Alt);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBS-a-MinTot; NB\n";
        t1.StartTimer();
        dbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbs.GetNodesExpanded(),
               dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBS-a-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBS-o-MaxTot
    if (ap.HasAlgorithm("DBBS-o-MaxTot")) {
        DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MaxTot> dbbs(
                SideCriterion::OptCount);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBS-o-MaxTot; NB\n";
        t1.StartTimer();
        dbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbs.GetNodesExpanded(),
               dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBS-o-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBS-o-MinTot
    if (ap.HasAlgorithm("DBBS-o-MinTot")) {
        DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinTot> dbbs(
                SideCriterion::OptCount);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBS-o-MinTot; NB\n";
        t1.StartTimer();
        dbbs.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbs.GetNodesExpanded(),
               dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBS-o-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBSLB-a
    if (ap.HasAlgorithm("DBBSLB-a")) {
        DBBSLB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinB> dbbslb(
                SideCriterion::Alt);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBSLB-a; NB\n";
        t1.StartTimer();
        dbbslb.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbslb.GetNodesExpanded(),
               dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBSLB-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBSLB-p
    if (ap.HasAlgorithm("DBBSLB-p")) {
        DBBSLB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinB> dbbslb(
                SideCriterion::Cardinality);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBSLB-p; NB\n";
        t1.StartTimer();
        dbbslb.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbslb.GetNodesExpanded(),
               dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBSLB-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBSLB-a-MaxTot
    if (ap.HasAlgorithm("DBBSLB-a-MaxTot")) {
        DBBSLB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MaxTot> dbbslb(
                SideCriterion::Alt);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBSLB-a-MaxTot; NB\n";
        t1.StartTimer();
        dbbslb.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbslb.GetNodesExpanded(),
               dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBSLB-a-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBSLB-a-MinTot
    if (ap.HasAlgorithm("DBBSLB-a-MinTot")) {
        DBBSLB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinTot> dbbslb(
                SideCriterion::Alt);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBSLB-a-MinTot; NB\n";
        t1.StartTimer();
        dbbslb.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbslb.GetNodesExpanded(),
               dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBSLB-a-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBSLB-o-MaxTot
    if (ap.HasAlgorithm("DBBSLB-o-MaxTot")) {
        DBBSLB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MaxTot> dbbslb(
                SideCriterion::OptCount);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBSLB-o-MaxTot; NB\n";
        t1.StartTimer();
        dbbslb.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbslb.GetNodesExpanded(),
               dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBSLB-o-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //DBBSLB-o-MinTot
    if (ap.HasAlgorithm("DBBSLB-o-MinTot")) {
        DBBSLB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinTot> dbbslb(
                SideCriterion::OptCount);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] DBBSLB-o-MinTot; NB\n";
        t1.StartTimer();
        dbbslb.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), dbbslb.GetNodesExpanded(),
               dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("DBBSLB-o-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    if (ap.HasAlgorithm("GMX")) {
        std::cout << "[A] GMX; NB\n";
        GMX<PancakePuzzleState<N>> gmx;
        t1.StartTimer();
        {
            TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar;
            goal.Reset();
            start = original;
            std::vector<PancakePuzzleState<N>> path;
            astar.GetPath(&pancake, start, goal, path);
            gmx.GenerateBuckets(astar.openClosedList.elements, &pancake, start, pancake.GetPathLength(path), true);
        }

        {
            TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> rastar;
            goal.Reset();
            start = original;
            std::vector<PancakePuzzleState<N>> path;
            rastar.GetPath(&pancake, goal, start, path);
            gmx.GenerateBuckets(rastar.openClosedList.elements, &pancake, goal, pancake.GetPathLength(path), false);
        }
        int expanded = gmx.GetMinimalExpansions();
        t1.EndTimer();
        printf("[R] Path length %1.0f; %d expanded; %d necessary; %1.2fs elapsed\n",
               optimal_cost, expanded, expanded, t1.GetElapsedTime());
    }

    if (ap.HasAlgorithm("RA*")) {
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> rastar;
        goal.Reset();
        start = original;
        std::cout << "[A] RA*; NB\n";
        t1.StartTimer();
        rastar.GetPath(&pancake, goal, start, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), rastar.GetNodesExpanded(),
               rastar.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("RA* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    if (ap.HasAlgorithm("NGMX")) {
        CalculateWVC<PancakePuzzleState<N>> calculateWVC;
        double C = -1;
        std::map<int, int> gCountMapForwardSingle;
        std::map<int, int> gCountMapBackwardSingle;
        std::vector<double> times;
        t1.StartTimer();
        {
            TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar;
            goal.Reset();
            start = original;
            std::vector<PancakePuzzleState<N>> path;
            astar.GetPath(&pancake, start, goal, path);
            C = pancake.GetPathLength(path);
            gCountMapForwardSingle = calculateWVC.initGCountMap(astar.openClosedList.elements, C);
        }
        t1.EndTimer();
        times.push_back(t1.GetElapsedTime());

        {
            t1.StartTimer();
            TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> rastar;
            goal.Reset();
            start = original;
            std::vector<PancakePuzzleState<N>> path;
            rastar.GetPath(&pancake, goal, start, path);
            if (pancake.GetPathLength(path) != C) {
                assert(!"Unequal astar and raster paths");
            }
            t1.EndTimer();
            times.push_back(t1.GetElapsedTime());
            std::cout << "[A] RA*; NB\n";
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   C, rastar.GetNodesExpanded(), rastar.GetNecessaryExpansions(), t1.GetElapsedTime());
            gCountMapBackwardSingle = calculateWVC.initGCountMap(rastar.openClosedList.elements, C);
        }
        t1.StartTimer();
        int expanded = calculateWVC.CalcWVC(gCountMapForwardSingle, gCountMapBackwardSingle, C, 1, false);
        t1.EndTimer();
        std::cout << "[A] NGMX; NB\n";
        printf("[R] Path length %1.0f; %d expanded; %d necessary; %1.2fs elapsed\n",
               C, expanded, expanded, times[0] + times[1] + t1.GetElapsedTime());
    }

    //BTB-conn
    if (ap.HasAlgorithm("BTB-conn")) {
        BTB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N >> btb(BTBPolicy::MostConnected);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] BTB-conn; NB\n";
        t1.StartTimer();
        btb.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), btb.GetNodesExpanded(),
               btb.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("BTB-conn reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    //BTB-nbs
    if (ap.HasAlgorithm("BTB-nbs")) {
        BTB<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N >> btb(BTBPolicy::Alternating);
        std::vector<PancakePuzzleState<N>> path;
        start = original;
        std::cout << "[A] BTB-nbs; NB\n";
        t1.StartTimer();
        btb.GetPath(&pancake, start, goal, &pancake, &pancake2, path);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(path), btb.GetNodesExpanded(),
               btb.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(path);
        else if (optimal_cost != pancake.GetPathLength(path)) {
            printf("BTB-nbs reported bad value!! optimal %1.0f; reported %1.0f;\n",
                   optimal_cost, pancake.GetPathLength(path));
            exit(0);
        }
    }

    std::vector<PancakeBAELBPair > baelbs;
    baelbs.push_back(PancakeBAELBPair("BAE*-fd-a", PancakeBAELB(ivF, ivD)));
    baelbs.push_back(PancakeBAELBPair("BAE*-fd-p", PancakeBAELB(ivF, ivD, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-rfrd-a", PancakeBAELB(ivRF, ivRD)));
    baelbs.push_back(PancakeBAELBPair("BAE*-rfrd-p", PancakeBAELB(ivRF, ivRD, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-df-a", PancakeBAELB(ivD, ivF)));
    baelbs.push_back(PancakeBAELBPair("BAE*-df-p", PancakeBAELB(ivD, ivF, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-rdrf-a", PancakeBAELB(ivRD, ivRF)));
    baelbs.push_back(PancakeBAELBPair("BAE*-rdrf-p", PancakeBAELB(ivRD, ivRF, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-g-a", PancakeBAELB(ivG)));
    baelbs.push_back(PancakeBAELBPair("BAE*-g-p", PancakeBAELB(ivG, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-b-a", PancakeBAELB(ivB, true, 1.0, 1.0, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-b-p", PancakeBAELB(ivB, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-frdrfd-a", PancakeBAELB(ivFRD, ivRFD)));
    baelbs.push_back(PancakeBAELBPair("BAE*-frdrfd-p", PancakeBAELB(ivFRD, ivRFD, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-gfgd-a", PancakeBAELB(ivGF, ivGD)));
    baelbs.push_back(PancakeBAELBPair("BAE*-gfgd-p", PancakeBAELB(ivGF, ivGD, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-rfdfrd-a", PancakeBAELB(ivRFD, ivFRD)));
    baelbs.push_back(PancakeBAELBPair("BAE*-rfdfrd-p", PancakeBAELB(ivRFD, ivFRD, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-rfrdx2-a", PancakeBAELB(ivRFRD, ivRFRD)));
    baelbs.push_back(PancakeBAELBPair("BAE*-rfrdx2-p", PancakeBAELB(ivRFRD, ivRFRD, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-grfgrd-a", PancakeBAELB(ivGRF, ivGRD)));
    baelbs.push_back(PancakeBAELBPair("BAE*-grfgrd-p", PancakeBAELB(ivGRF, ivGRD, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-gdgf-a", PancakeBAELB(ivGD, ivGF)));
    baelbs.push_back(PancakeBAELBPair("BAE*-gdgf-p", PancakeBAELB(ivGD, ivGF, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-grdgrf-a", PancakeBAELB(ivGRD, ivGRF)));
    baelbs.push_back(PancakeBAELBPair("BAE*-grdgrf-p", PancakeBAELB(ivGRD, ivGRF, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-gb-a", PancakeBAELB(ivGB)));
    baelbs.push_back(PancakeBAELBPair("BAE*-gb-p", PancakeBAELB(ivGB, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-gfrdgrfd-a", PancakeBAELB(ivGFRD, ivGRFD)));
    baelbs.push_back(PancakeBAELBPair("BAE*-gfrdgrfd-p", PancakeBAELB(ivGFRD, ivGRFD, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-grfdgfrd-a", PancakeBAELB(ivGRFD, ivGFRD)));
    baelbs.push_back(PancakeBAELBPair("BAE*-grfdgfrd-p", PancakeBAELB(ivGRFD, ivGFRD, false)));
    baelbs.push_back(PancakeBAELBPair("BAE*-grfrd-a", PancakeBAELB(ivGRFRD)));
    baelbs.push_back(PancakeBAELBPair("BAE*-grfrd-p", PancakeBAELB(ivGRFRD, false)));

    for (PancakeBAELBPair &solver: baelbs) {
        if (!ap.HasAlgorithm(solver.first)) {
            continue;
        }
        start = original;
        std::cout << "[A] " << solver.first.c_str() << "; WB\n";
        t1.StartTimer();
        solver.second.GetPath(&pancake, start, goal, &pancake, &pancake2, baelbPath);
        t1.EndTimer();
        printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
               pancake.GetPathLength(baelbPath), solver.second.GetNodesExpanded(),
               solver.second.GetNecessaryExpansions(), t1.GetElapsedTime());

        // test optimality
        if (optimal_cost < 0.0) optimal_cost = pancake.GetPathLength(baelbPath);
        else if (optimal_cost != pancake.GetPathLength(baelbPath)) {
            printf("%s reported bad value!! optimal %1.0f; reported %1.0f;\n", solver.first.c_str(),
                   optimal_cost, pancake.GetPathLength(baelbPath));
            exit(0);
        }
        solver.second.ClearMemory();
    }
}

void TestPancake(const ArgParameters &ap) {
    for (int count = ap.instanceId; count < ap.instanceId + ap.numOfInstances; count++) {
        TestSingleInstance(ap, count);
    }
}
