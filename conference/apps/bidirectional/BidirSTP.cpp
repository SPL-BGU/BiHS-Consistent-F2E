//
//  BidirSTP.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/30/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirSTP.h"
#include "MNPuzzle.h"
#include "NBS.h"
#include "DVCBS.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "BAE.h"
#include "TemplateAStar.h"
#include "Baseline.h"
#include "GBFHS.h"
#include "DBBS.h"
#include "BTB.h"
#include "BAELB.h"
#include "GMX.h"
#include "DBBSLB.h"
#include "CalculateWVC.h"

#define STPBAELBPair std::pair<std::string, BAELB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4 >>>
#define STPBAELB BAELB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4 >>

MNPuzzleState<4, 4> GetKorfInstance(int which) {
    int instances[100][16] =
            {{14, 13, 15, 7,  11, 12, 9,  5,  6,  0,  2,  1,  4,  8,  10, 3},
             {13, 5,  4,  10, 9,  12, 8,  14, 2,  3,  7,  1,  0,  15, 11, 6},
             {14, 7,  8,  2,  13, 11, 10, 4,  9,  12, 5,  0,  3,  6,  1,  15},
             {5,  12, 10, 7,  15, 11, 14, 0,  8,  2,  1,  13, 3,  4,  9,  6},
             {4,  7,  14, 13, 10, 3,  9,  12, 11, 5,  6,  15, 1,  2,  8,  0},
             {14, 7,  1,  9,  12, 3,  6,  15, 8,  11, 2,  5,  10, 0,  4,  13},
             {2,  11, 15, 5,  13, 4,  6,  7,  12, 8,  10, 1,  9,  3,  14, 0},
             {12, 11, 15, 3,  8,  0,  4,  2,  6,  13, 9,  5,  14, 1,  10, 7},
             {3,  14, 9,  11, 5,  4,  8,  2,  13, 12, 6,  7,  10, 1,  15, 0},
             {13, 11, 8,  9,  0,  15, 7,  10, 4,  3,  6,  14, 5,  12, 2,  1},
             {5,  9,  13, 14, 6,  3,  7,  12, 10, 8,  4,  0,  15, 2,  11, 1},
             {14, 1,  9,  6,  4,  8,  12, 5,  7,  2,  3,  0,  10, 11, 13, 15},
             {3,  6,  5,  2,  10, 0,  15, 14, 1,  4,  13, 12, 9,  8,  11, 7},
             {7,  6,  8,  1,  11, 5,  14, 10, 3,  4,  9,  13, 15, 2,  0,  12},
             {13, 11, 4,  12, 1,  8,  9,  15, 6,  5,  14, 2,  7,  3,  10, 0},
             {1,  3,  2,  5,  10, 9,  15, 6,  8,  14, 13, 11, 12, 4,  7,  0},
             {15, 14, 0,  4,  11, 1,  6,  13, 7,  5,  8,  9,  3,  2,  10, 12},
             {6,  0,  14, 12, 1,  15, 9,  10, 11, 4,  7,  2,  8,  3,  5,  13},
             {7,  11, 8,  3,  14, 0,  6,  15, 1,  4,  13, 9,  5,  12, 2,  10},
             {6,  12, 11, 3,  13, 7,  9,  15, 2,  14, 8,  10, 4,  1,  5,  0},
             {12, 8,  14, 6,  11, 4,  7,  0,  5,  1,  10, 15, 3,  13, 9,  2},
             {14, 3,  9,  1,  15, 8,  4,  5,  11, 7,  10, 13, 0,  2,  12, 6},
             {10, 9,  3,  11, 0,  13, 2,  14, 5,  6,  4,  7,  8,  15, 1,  12},
             {7,  3,  14, 13, 4,  1,  10, 8,  5,  12, 9,  11, 2,  15, 6,  0},
             {11, 4,  2,  7,  1,  0,  10, 15, 6,  9,  14, 8,  3,  13, 5,  12},
             {5,  7,  3,  12, 15, 13, 14, 8,  0,  10, 9,  6,  1,  4,  2,  11},
             {14, 1,  8,  15, 2,  6,  0,  3,  9,  12, 10, 13, 4,  7,  5,  11},
             {13, 14, 6,  12, 4,  5,  1,  0,  9,  3,  10, 2,  15, 11, 8,  7},
             {9,  8,  0,  2,  15, 1,  4,  14, 3,  10, 7,  5,  11, 13, 6,  12},
             {12, 15, 2,  6,  1,  14, 4,  8,  5,  3,  7,  0,  10, 13, 9,  11},
             {12, 8,  15, 13, 1,  0,  5,  4,  6,  3,  2,  11, 9,  7,  14, 10},
             {14, 10, 9,  4,  13, 6,  5,  8,  2,  12, 7,  0,  1,  3,  11, 15},
             {14, 3,  5,  15, 11, 6,  13, 9,  0,  10, 2,  12, 4,  1,  7,  8},
             {6,  11, 7,  8,  13, 2,  5,  4,  1,  10, 3,  9,  14, 0,  12, 15},
             {1,  6,  12, 14, 3,  2,  15, 8,  4,  5,  13, 9,  0,  7,  11, 10},
             {12, 6,  0,  4,  7,  3,  15, 1,  13, 9,  8,  11, 2,  14, 5,  10},
             {8,  1,  7,  12, 11, 0,  10, 5,  9,  15, 6,  13, 14, 2,  3,  4},
             {7,  15, 8,  2,  13, 6,  3,  12, 11, 0,  4,  10, 9,  5,  1,  14},
             {9,  0,  4,  10, 1,  14, 15, 3,  12, 6,  5,  7,  11, 13, 8,  2},
             {11, 5,  1,  14, 4,  12, 10, 0,  2,  7,  13, 3,  9,  15, 6,  8},
             {8,  13, 10, 9,  11, 3,  15, 6,  0,  1,  2,  14, 12, 5,  4,  7},
             {4,  5,  7,  2,  9,  14, 12, 13, 0,  3,  6,  11, 8,  1,  15, 10},
             {11, 15, 14, 13, 1,  9,  10, 4,  3,  6,  2,  12, 7,  5,  8,  0},
             {12, 9,  0,  6,  8,  3,  5,  14, 2,  4,  11, 7,  10, 1,  15, 13},
             {3,  14, 9,  7,  12, 15, 0,  4,  1,  8,  5,  6,  11, 10, 2,  13},
             {8,  4,  6,  1,  14, 12, 2,  15, 13, 10, 9,  5,  3,  7,  0,  11},
             {6,  10, 1,  14, 15, 8,  3,  5,  13, 0,  2,  7,  4,  9,  11, 12},
             {8,  11, 4,  6,  7,  3,  10, 9,  2,  12, 15, 13, 0,  1,  5,  14},
             {10, 0,  2,  4,  5,  1,  6,  12, 11, 13, 9,  7,  15, 3,  14, 8},
             {12, 5,  13, 11, 2,  10, 0,  9,  7,  8,  4,  3,  14, 6,  15, 1},
             {10, 2,  8,  4,  15, 0,  1,  14, 11, 13, 3,  6,  9,  7,  5,  12},
             {10, 8,  0,  12, 3,  7,  6,  2,  1,  14, 4,  11, 15, 13, 9,  5},
             {14, 9,  12, 13, 15, 4,  8,  10, 0,  2,  1,  7,  3,  11, 5,  6},
             {12, 11, 0,  8,  10, 2,  13, 15, 5,  4,  7,  3,  6,  9,  14, 1},
             {13, 8,  14, 3,  9,  1,  0,  7,  15, 5,  4,  10, 12, 2,  6,  11},
             {3,  15, 2,  5,  11, 6,  4,  7,  12, 9,  1,  0,  13, 14, 10, 8},
             {5,  11, 6,  9,  4,  13, 12, 0,  8,  2,  15, 10, 1,  7,  3,  14},
             {5,  0,  15, 8,  4,  6,  1,  14, 10, 11, 3,  9,  7,  12, 2,  13},
             {15, 14, 6,  7,  10, 1,  0,  11, 12, 8,  4,  9,  2,  5,  13, 3},
             {11, 14, 13, 1,  2,  3,  12, 4,  15, 7,  9,  5,  10, 6,  8,  0},
             {6,  13, 3,  2,  11, 9,  5,  10, 1,  7,  12, 14, 8,  4,  0,  15},
             {4,  6,  12, 0,  14, 2,  9,  13, 11, 8,  3,  15, 7,  10, 1,  5},
             {8,  10, 9,  11, 14, 1,  7,  15, 13, 4,  0,  12, 6,  2,  5,  3},
             {5,  2,  14, 0,  7,  8,  6,  3,  11, 12, 13, 15, 4,  10, 9,  1},
             {7,  8,  3,  2,  10, 12, 4,  6,  11, 13, 5,  15, 0,  1,  9,  14},
             {11, 6,  14, 12, 3,  5,  1,  15, 8,  0,  10, 13, 9,  7,  4,  2},
             {7,  1,  2,  4,  8,  3,  6,  11, 10, 15, 0,  5,  14, 12, 13, 9},
             {7,  3,  1,  13, 12, 10, 5,  2,  8,  0,  6,  11, 14, 15, 4,  9},
             {6,  0,  5,  15, 1,  14, 4,  9,  2,  13, 8,  10, 11, 12, 7,  3},
             {15, 1,  3,  12, 4,  0,  6,  5,  2,  8,  14, 9,  13, 10, 7,  11},
             {5,  7,  0,  11, 12, 1,  9,  10, 15, 6,  2,  3,  8,  4,  13, 14},
             {12, 15, 11, 10, 4,  5,  14, 0,  13, 7,  1,  2,  9,  8,  3,  6},
             {6,  14, 10, 5,  15, 8,  7,  1,  3,  4,  2,  0,  12, 9,  11, 13},
             {14, 13, 4,  11, 15, 8,  6,  9,  0,  7,  3,  1,  2,  10, 12, 5},
             {14, 4,  0,  10, 6,  5,  1,  3,  9,  2,  13, 15, 12, 7,  8,  11},
             {15, 10, 8,  3,  0,  6,  9,  5,  1,  14, 13, 11, 7,  2,  12, 4},
             {0,  13, 2,  4,  12, 14, 6,  9,  15, 1,  10, 3,  11, 5,  8,  7},
             {3,  14, 13, 6,  4,  15, 8,  9,  5,  12, 10, 0,  2,  7,  1,  11},
             {0,  1,  9,  7,  11, 13, 5,  3,  14, 12, 4,  2,  8,  6,  10, 15},
             {11, 0,  15, 8,  13, 12, 3,  5,  10, 1,  4,  6,  14, 9,  7,  2},
             {13, 0,  9,  12, 11, 6,  3,  5,  15, 8,  1,  10, 4,  14, 2,  7},
             {14, 10, 2,  1,  13, 9,  8,  11, 7,  3,  6,  12, 15, 5,  4,  0},
             {12, 3,  9,  1,  4,  5,  10, 2,  6,  11, 15, 0,  14, 7,  13, 8},
             {15, 8,  10, 7,  0,  12, 14, 1,  5,  9,  6,  3,  13, 11, 4,  2},
             {4,  7,  13, 10, 1,  2,  9,  6,  12, 8,  14, 5,  3,  0,  11, 15},
             {6,  0,  5,  10, 11, 12, 9,  2,  1,  7,  4,  3,  14, 8,  13, 15},
             {9,  5,  11, 10, 13, 0,  2,  1,  8,  6,  14, 12, 4,  7,  3,  15},
             {15, 2,  12, 11, 14, 13, 9,  5,  1,  3,  8,  7,  0,  10, 6,  4},
             {11, 1,  7,  4,  10, 13, 3,  8,  9,  14, 0,  15, 6,  5,  2,  12},
             {5,  4,  7,  1,  11, 12, 14, 15, 10, 13, 8,  6,  2,  0,  9,  3},
             {9,  7,  5,  2,  14, 15, 12, 10, 11, 3,  6,  1,  8,  13, 0,  4},
             {3,  2,  7,  9,  0,  15, 12, 4,  6,  11, 5,  14, 8,  13, 10, 1},
             {13, 9,  14, 6,  12, 8,  1,  2,  3,  4,  0,  7,  5,  10, 11, 15},
             {5,  7,  11, 8,  0,  14, 9,  13, 10, 12, 3,  15, 6,  1,  4,  2},
             {4,  3,  6,  13, 7,  15, 9,  0,  10, 5,  8,  11, 2,  12, 1,  14},
             {1,  7,  15, 14, 2,  6,  4,  9,  12, 11, 13, 3,  0,  8,  5,  10},
             {9,  14, 5,  7,  8,  15, 1,  2,  10, 4,  13, 6,  12, 0,  11, 3},
             {0,  11, 3,  12, 5,  2,  1,  9,  8,  10, 14, 15, 7,  4,  13, 6},
             {7,  15, 4,  0,  10, 9,  2,  5,  12, 11, 13, 6,  1,  3,  14, 8},
             {11, 4,  0,  8,  6,  10, 5,  13, 12, 7,  14, 3,  1,  2,  9,  15}};

    MNPuzzleState<4, 4> s;
    for (int x = 0; x < 16; x++) {
        s.puzzle[x] = instances[which][x];
        if (s.puzzle[x] == 0)
            s.blank = x;
    }
    return s;
}

void TestSTP(std::string alg, int instanceId, int md_ignore) {
    MNPuzzle<4, 4> mnp(md_ignore);

    int startId = 0;
    int lastId = 100;
    if (instanceId > -1) {
        startId = instanceId;
        lastId = startId + 1;
    }

    for (int x = startId; x < lastId; x++) // 547 to 540
    {

        MNPuzzleState<4, 4> start, goal;
        printf("[I] MD-%d Problem %d of %d\n", md_ignore, x, lastId);


        double optimal_cost = -1.0;

        goal.Reset();
        start = GetKorfInstance(x);

        // BAE*-a
        if (alg == "BAE*-o-a") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            BAE<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4 >> bae;
            Timer timer;
            std::cout << "[A] BAE*-o-a; NB\n";
            timer.StartTimer();
            bae.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), bae.GetNodesExpanded(),
                   bae.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("BAE*-o-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        // BAE*-p
        if (alg == "BAE*-o-p") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            BAE<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4 >> bae(false);
            Timer timer;
            std::cout << "[A] BAE*-o-p; NB\n";
            timer.StartTimer();
            bae.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), bae.GetNodesExpanded(),
                   bae.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("BAE*-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        // A*
        if (alg == "A*") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
            Timer timer;
            std::cout << "[A] A*; NB\n";
            timer.StartTimer();
            astar.GetPath(&mnp, start, goal, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), astar.GetNodesExpanded(),
                   astar.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("A* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        // NBS
        if (alg == "NBS") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            NBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, NBSQueue<MNPuzzleState<4, 4>, 1, true>> nbs(false, true);
            Timer timer;
            goal.Reset();
            start = GetKorfInstance(x);
            std::cout << "[A] NBS; NB\n";
            timer.StartTimer();
            nbs.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();

            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), nbs.GetNodesExpanded(),
                   nbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("NBS reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        // DVCBS
        if (alg == "DVCBS") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DVCBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, DVCBSQueue<MNPuzzleState<4, 4>, 1, false >> dvcbs(
                    false, true);
            Timer timer;
            goal.Reset();
            start = GetKorfInstance(x);
            std::cout << "[A] DVCBS; NB\n";
            timer.StartTimer();
            dvcbs.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();

            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dvcbs.GetNodesExpanded(),
                   dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DVCBS reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBS-a
        if (alg == "DBBS") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinB> dbbs(SideCriterion::Alt);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBS-a; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBS-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBS-p
        if (alg == "DBBS") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinB> dbbs(SideCriterion::Cardinality);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBS-p; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBS-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBS-a-MaxTot
        if (alg == "DBBS") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MaxTot> dbbs(SideCriterion::Alt);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBS-a-MaxTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBS-a-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBS-a-MinTot
        if (alg == "DBBS") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinTot> dbbs(SideCriterion::Alt);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBS-a-MinTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBS-a-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBS-o-MaxTot
        if (alg == "DBBS") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MaxTot> dbbs(SideCriterion::OptCount);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBS-o-MaxTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBS-o-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBS-o-MinTot
        if (alg == "DBBS") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinTot> dbbs(SideCriterion::OptCount);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBS-o-MinTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBS-o-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBSLB-a
        if (alg == "DBBSLB") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBSLB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinB> dbbslb(SideCriterion::Alt);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBSLB-a; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBSLB-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBSLB-p
        if (alg == "DBBSLB") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBSLB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinB> dbbslb(SideCriterion::Cardinality);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBSLB-p; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBSLB-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBSLB-a-MaxTot
        if (alg == "DBBSLB") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBSLB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MaxTot> dbbslb(SideCriterion::Alt);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBSLB-a-MaxTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBSLB-a-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBSLB-a-MinTot
        if (alg == "DBBSLB") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBSLB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinTot> dbbslb(SideCriterion::Alt);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBSLB-a-MinTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBSLB-a-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBSLB-o-MaxTot
        if (alg == "DBBSLB") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBSLB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MaxTot> dbbslb(SideCriterion::OptCount);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBSLB-o-MaxTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBSLB-o-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //DBBSLB-o-MinTot
        if (alg == "DBBSLB") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            DBBSLB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinTot> dbbslb(SideCriterion::OptCount);
            Timer timer;
            goal.Reset();
            std::cout << "[A] DBBSLB-o-MinTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("DBBSLB-o-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }


        // GMX
        if (alg == "GMX") {
            std::cout << "[A] GMX; NB\n";
            GMX<MNPuzzleState<4, 4>> gmx;
            Timer timer;
            timer.StartTimer();
            {
                TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
                goal.Reset();
                std::vector<MNPuzzleState<4, 4>> solutionPath;
                astar.GetPath(&mnp, start, goal, solutionPath);
                gmx.GenerateBuckets(astar.openClosedList.elements, &mnp, start, mnp.GetPathLength(solutionPath), true);
            }

            {
                TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> rastar;
                goal.Reset();
                std::vector<MNPuzzleState<4, 4>> solutionPath;
                rastar.GetPath(&mnp, goal, start, solutionPath);
                gmx.GenerateBuckets(rastar.openClosedList.elements, &mnp, goal, mnp.GetPathLength(solutionPath), false);
            }
            int expanded = gmx.GetMinimalExpansions();
            timer.EndTimer();
            printf("[R] Path length %1.0f; %d expanded; %d necessary; %1.2fs elapsed\n",
                   optimal_cost, expanded, expanded, timer.GetElapsedTime());
        }

        if (alg == "RA*") {
            Timer timer;
            TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> rastar;
            goal.Reset();
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            std::cout << "[A] RA*; NB\n";
            timer.StartTimer();
            rastar.GetPath(&mnp, goal, start, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), rastar.GetNodesExpanded(),
                   rastar.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("RA* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //NGMX
        if (alg == "NGMX") {
            Timer timer;
            CalculateWVC<MNPuzzleState<4, 4>> calculateWVC;
            double C = -1;
            std::map<int, int> gCountMapForwardSingle;
            std::map<int, int> gCountMapBackwardSingle;
            std::vector<double> times;
            timer.StartTimer();
            {
                TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> astar;
                goal.Reset();
                std::vector<MNPuzzleState<4, 4>> solutionPath;
                astar.GetPath(&mnp, start, goal, solutionPath);
                C = mnp.GetPathLength(solutionPath);
                gCountMapForwardSingle = calculateWVC.initGCountMap(astar.openClosedList.elements, C);
            }
            timer.EndTimer();
            times.push_back(timer.GetElapsedTime());

            {
                timer.StartTimer();
                TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> rastar;
                goal.Reset();
                std::vector<MNPuzzleState<4, 4>> solutionPath;
                rastar.GetPath(&mnp, goal, start, solutionPath);
                if (mnp.GetPathLength(solutionPath) != C) {
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
        if (alg == "BTB") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            BTB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4 >> btb(BTBPolicy::MostConnected);
            goal.Reset();
            std::cout << "[A] BTB-conn; NB\n";
            Timer timer;
            timer.StartTimer();
            btb.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), btb.GetNodesExpanded(),
                   btb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("RA* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }

        //BTB-nbs
        if (alg == "BTB") {
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            BTB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4 >> btb(BTBPolicy::Alternating);
            goal.Reset();
            std::cout << "[A] BTB-nbs; NB\n";
            Timer timer;
            timer.StartTimer();
            btb.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), btb.GetNodesExpanded(),
                   btb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("RA* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
        }


        std::vector<STPBAELBPair > baelbs;
        baelbs.push_back(STPBAELBPair("BAE*-fd-a", STPBAELB(ivF, ivD)));
        baelbs.push_back(STPBAELBPair("BAE*-fd-p", STPBAELB(ivF, ivD, false)));
        baelbs.push_back(STPBAELBPair("BAE*-rfrd-a", STPBAELB(ivRF, ivRD)));
        baelbs.push_back(STPBAELBPair("BAE*-rfrd-p", STPBAELB(ivRF, ivRD, false)));
        baelbs.push_back(STPBAELBPair("BAE*-df-a", STPBAELB(ivD, ivF)));
        baelbs.push_back(STPBAELBPair("BAE*-df-p", STPBAELB(ivD, ivF, false)));
        baelbs.push_back(STPBAELBPair("BAE*-rdrf-a", STPBAELB(ivRD, ivRF)));
        baelbs.push_back(STPBAELBPair("BAE*-rdrf-p", STPBAELB(ivRD, ivRF, false)));
        baelbs.push_back(STPBAELBPair("BAE*-g-a", STPBAELB(ivG)));
        baelbs.push_back(STPBAELBPair("BAE*-g-p", STPBAELB(ivG, false)));
        baelbs.push_back(STPBAELBPair("BAE*-b-a", STPBAELB(ivB)));
        baelbs.push_back(STPBAELBPair("BAE*-b-p", STPBAELB(ivB, false)));
        baelbs.push_back(STPBAELBPair("BAE*-frdrfd-a", STPBAELB(ivFRD, ivRFD)));
        baelbs.push_back(STPBAELBPair("BAE*-frdrfd-p", STPBAELB(ivFRD, ivRFD, false)));
        baelbs.push_back(STPBAELBPair("BAE*-gfgd-a", STPBAELB(ivGF, ivGD)));
        baelbs.push_back(STPBAELBPair("BAE*-gfgd-p", STPBAELB(ivGF, ivGD, false)));
        baelbs.push_back(STPBAELBPair("BAE*-rfdfrd-a", STPBAELB(ivRFD, ivFRD)));
        baelbs.push_back(STPBAELBPair("BAE*-rfdfrd-p", STPBAELB(ivRFD, ivFRD, false)));
        baelbs.push_back(STPBAELBPair("BAE*-rfrdx2-a", STPBAELB(ivRFRD, ivRFRD)));
        baelbs.push_back(STPBAELBPair("BAE*-rfrdx2-p", STPBAELB(ivRFRD, ivRFRD, false)));
        baelbs.push_back(STPBAELBPair("BAE*-grfgrd-a", STPBAELB(ivGRF, ivGRD)));
        baelbs.push_back(STPBAELBPair("BAE*-grfgrd-p", STPBAELB(ivGRF, ivGRD, false)));
        baelbs.push_back(STPBAELBPair("BAE*-gdgf-a", STPBAELB(ivGD, ivGF)));
        baelbs.push_back(STPBAELBPair("BAE*-gdgf-p", STPBAELB(ivGD, ivGF, false)));
        baelbs.push_back(STPBAELBPair("BAE*-grdgrf-a", STPBAELB(ivGRD, ivGRF)));
        baelbs.push_back(STPBAELBPair("BAE*-grdgrf-p", STPBAELB(ivGRD, ivGRF, false)));
        baelbs.push_back(STPBAELBPair("BAE*-gb-a", STPBAELB(ivGB)));
        baelbs.push_back(STPBAELBPair("BAE*-gb-p", STPBAELB(ivGB, false)));
        baelbs.push_back(STPBAELBPair("BAE*-gfrdgrfd-a", STPBAELB(ivGFRD, ivGRFD)));
        baelbs.push_back(STPBAELBPair("BAE*-gfrdgrfd-p", STPBAELB(ivGFRD, ivGRFD, false)));
        baelbs.push_back(STPBAELBPair("BAE*-grfdgfrd-a", STPBAELB(ivGRFD, ivGFRD)));
        baelbs.push_back(STPBAELBPair("BAE*-grfdgfrd-p", STPBAELB(ivGRFD, ivGFRD, false)));
        baelbs.push_back(STPBAELBPair("BAE*-grfrd-a", STPBAELB(ivGRFRD)));
        baelbs.push_back(STPBAELBPair("BAE*-grfrd-p", STPBAELB(ivGRFRD, false)));

        for (STPBAELBPair &solver: baelbs) {
            if (solver.first != alg) {
                continue;
            }
            std::cout << "[A] " << solver.first.c_str() << "; WB\n";
            std::vector<MNPuzzleState<4, 4>> solutionPath;
            Timer timer;
            timer.StartTimer();
            solver.second.GetPath(&mnp, start, goal, &mnp, &mnp, solutionPath);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   mnp.GetPathLength(solutionPath), solver.second.GetNodesExpanded(),
                   solver.second.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = mnp.GetPathLength(solutionPath);
            else if (optimal_cost != mnp.GetPathLength(solutionPath)) {
                printf("%s reported bad value!! optimal %1.0f; reported %1.0f;\n", solver.first.c_str(),
                       optimal_cost, mnp.GetPathLength(solutionPath));
                exit(0);
            }
            solver.second.ClearMemory();
        }
    }
    exit(0);
}
