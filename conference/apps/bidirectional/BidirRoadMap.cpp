//
// Created by Lior Siag on 24/12/2022.
//


#include "BidirRoadMap.h"
#include "TemplateAStar.h"
#include "BAE.h"
#include "BAELB.h"
#include "NBSQueue.h"
#include "NBS.h"
#include "DVCBSQueue.h"
#include "DVCBS.h"
#include "GMX.h"
#include "DBBS.h"
#include "DBBSLB.h"
#include "CalculateWVC.h"

#define RMBAELBPair std::pair<std::string, BAELB<intersection, neighbor, RoadMap>>
#define RMBAELB BAELB<intersection, neighbor, RoadMap>

void TestRoad(const ArgParameters &ap) {
    Timer t1;
    std::string graphStr = "../../maps/roadmaps/USA-road-t.COL.gr";
    std::string coordStr = "../../maps/roadmaps/USA-road-d.COL.co";
    RoadMap rm(graphStr.c_str(), coordStr.c_str(), false);

    int starti = ap.instanceId;
    int endi = starti + ap.numOfInstances;

    for (int i = starti; i < endi; i++) {
        srandom(i * 16);
        std::vector<intersection> path;
        node *n1 = rm.GetGraph()->GetRandomNode();
        node *n2 = rm.GetGraph()->GetRandomNode();
        std::cout << "[I] COL ("
                  << n1->GetLabelF(GraphSearchConstants::kXCoordinate) << ","
                  << n1->GetLabelF(GraphSearchConstants::kYCoordinate)
                  << ") (" << n2->GetLabelF(GraphSearchConstants::kXCoordinate) << ","
                  << n2->GetLabelF(GraphSearchConstants::kYCoordinate) << ")\n";

        intersection start = rm.GetGraph()->GetRandomNode()->GetNum();
        intersection goal = rm.GetGraph()->GetRandomNode()->GetNum();

        double optimal_cost = -1.0;

        //BAE*-o-a
        if (ap.HasAlgorithm("BAE*-o-a")) {
            BAE<intersection, neighbor, RoadMap> bae(true);
            std::cout << "[A] BAE*-o-a; NB\n";
            t1.StartTimer();
            bae.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), bae.GetNodesExpanded(),
                   bae.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("BAE*-o-a reported bad value!! optimal %1.0f; reported %1.0f;\n", optimal_cost,
                       rm.GetPathLength(path));
                exit(0);
            }
        }

        //BAE*-o-p
        if (ap.HasAlgorithm("BAE*-o-p")) {
            BAE<intersection, neighbor, RoadMap> bae(false);
            std::cout << "[A] BAE*-o-p; NB\n";
            t1.StartTimer();
            bae.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), bae.GetNodesExpanded(),
                   bae.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("BAE*-o-a reported bad value!! optimal %1.0f; reported %1.0f;\n", optimal_cost,
                       rm.GetPathLength(path));
                exit(0);
            }
        }

        //A*
        if (ap.HasAlgorithm("A*")) {
            std::cout << "[A] A*; NB\n";
            TemplateAStar<intersection, neighbor, RoadMap> astar;
            t1.StartTimer();
            astar.GetPath(&rm, start, goal, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), astar.GetNodesExpanded(),
                   astar.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("A* reported bad value!! optimal %1.0f; reported %1.0f;\n", optimal_cost,
                       rm.GetPathLength(path));
                exit(0);
            }
        }

        //NBS
        if (ap.HasAlgorithm("NBS")) {
            NBS<intersection, neighbor, RoadMap, NBSQueue<intersection, 1, false >> nbs(false, true);
            std::cout << "[A] NBS; NB\n";
            t1.StartTimer();
            nbs.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), nbs.GetNodesExpanded(),
                   nbs.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("NBS reported bad value!! optimal %1.0f; reported %1.0f;\n", optimal_cost,
                       rm.GetPathLength(path));
                exit(0);
            }
        }

        //DVCBS
        if (ap.HasAlgorithm("DVCBS")) {
            DVCBS<intersection, neighbor, RoadMap, DVCBSQueue<intersection, 1, false >> dvcbs(false, true);
            std::cout << "[A] DVCBS; NB\n";
            t1.StartTimer();
            dvcbs.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dvcbs.GetNodesExpanded(),
                   dvcbs.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DVCBS reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-a
        if (ap.HasAlgorithm("DBBS-a")) {
            DBBS<intersection, neighbor, RoadMap, MinCriterion::MinB> dbbs(SideCriterion::Alt);
            std::cout << "[A] DBBS-a; NB\n";
            t1.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBS-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-p
        if (ap.HasAlgorithm("DBBS-p")) {
            DBBS<intersection, neighbor, RoadMap, MinCriterion::MinB> dbbs(SideCriterion::Cardinality);
            std::cout << "[A] DBBS-p; NB\n";
            t1.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBS-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-a-MaxTot
        if (ap.HasAlgorithm("DBBS-a-MaxTot")) {
            DBBS<intersection, neighbor, RoadMap, MinCriterion::MaxTot> dbbs(SideCriterion::Alt);
            std::cout << "[A] DBBS-a-MaxTot; NB\n";
            t1.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBS-a-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-a-MinTot
        if (ap.HasAlgorithm("DBBS-a-MinTot")) {
            DBBS<intersection, neighbor, RoadMap, MinCriterion::MinTot> dbbs(SideCriterion::Alt);
            std::cout << "[A] DBBS-a-MinTot; NB\n";
            t1.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBS-a-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-o-MaxTot
        if (ap.HasAlgorithm("DBBS-o-MaxTot")) {
            DBBS<intersection, neighbor, RoadMap, MinCriterion::MaxTot> dbbs(SideCriterion::OptCount);
            std::cout << "[A] DBBS-o-MaxTot; NB\n";
            t1.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBS-o-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-o-MinTot
        if (ap.HasAlgorithm("DBBS-o-MinTot")) {
            DBBS<intersection, neighbor, RoadMap, MinCriterion::MinTot> dbbs(SideCriterion::OptCount);
            std::cout << "[A] DBBS-o-MinTot; NB\n";
            t1.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBS-o-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-a
        if (ap.HasAlgorithm("DBBSLB-a")) {
            DBBSLB<intersection, neighbor, RoadMap, MinCriterion::MinB> dbbslb(SideCriterion::Alt);
            std::cout << "[A] DBBSLB-a; NB\n";
            t1.StartTimer();
            dbbslb.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBSLB-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-p
        if (ap.HasAlgorithm("DBBSLB-p")) {
            DBBSLB<intersection, neighbor, RoadMap, MinCriterion::MinB> dbbslb(SideCriterion::Cardinality);
            std::cout << "[A] DBBSLB-p; NB\n";
            t1.StartTimer();
            dbbslb.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBSLB-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-a-MaxTot
        if (ap.HasAlgorithm("DBBSLB-a-MaxTot")) {
            DBBSLB<intersection, neighbor, RoadMap, MinCriterion::MaxTot> dbbslb(SideCriterion::Alt);
            std::cout << "[A] DBBSLB-a-MaxTot; NB\n";
            t1.StartTimer();
            dbbslb.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBSLB-a-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-a-MinTot
        if (ap.HasAlgorithm("DBBSLB-a-MinTot")) {
            DBBSLB<intersection, neighbor, RoadMap, MinCriterion::MinTot> dbbslb(SideCriterion::Alt);
            std::cout << "[A] DBBSLB-a-MinTot; NB\n";
            t1.StartTimer();
            dbbslb.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBSLB-a-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-o-MaxTot
        if (ap.HasAlgorithm("DBBSLB-o-MaxTot")) {
            DBBSLB<intersection, neighbor, RoadMap, MinCriterion::MaxTot> dbbslb(SideCriterion::OptCount);
            std::cout << "[A] DBBSLB-o-MaxTot; NB\n";
            t1.StartTimer();
            dbbslb.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBSLB-o-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-o-MinTot
        if (ap.HasAlgorithm("DBBSLB-o-MinTot")) {
            DBBSLB<intersection, neighbor, RoadMap, MinCriterion::MinTot> dbbslb(SideCriterion::OptCount);
            std::cout << "[A] DBBSLB-o-MinTot; NB\n";
            t1.StartTimer();
            dbbslb.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("DBBSLB-o-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
        }



        // GMX
        if (ap.HasAlgorithm("GMX")) {
            std::cout << "[A] GMX; NB\n";
            GMX<intersection> gmx;
            t1.StartTimer();
            {
                TemplateAStar<intersection, neighbor, RoadMap> astar;
                std::vector<intersection> path;
                astar.SetHeuristic(&rm);
                astar.GetPath(&rm, start, goal, path);
                gmx.GenerateBuckets(astar.openClosedList.elements, &rm, start, rm.GetPathLength(path), true);
            }

            {
                TemplateAStar<intersection, neighbor, RoadMap> rastar;
                std::vector<intersection> path;
                rastar.SetHeuristic(&rm);
                rastar.GetPath(&rm, goal, start, path);
                gmx.GenerateBuckets(rastar.openClosedList.elements, &rm, start, rm.GetPathLength(path), false);
            }

            int expanded = gmx.GetMinimalExpansions();
            t1.EndTimer();
            printf("[R] Path length %1.0f; %d expanded; %d necessary; %1.2fs elapsed\n",
                   optimal_cost, expanded, expanded, t1.GetElapsedTime());
        }

        // NGMX
        if (ap.HasAlgorithm("NGMX")) {
            CalculateWVC<intersection> calculateWVC;
            int C = -1;
            std::map<int, int> gCountMapForwardSingle;
            std::map<int, int> gCountMapBackwardSingle;
            std::vector<double> times;
            t1.StartTimer();
            {
                TemplateAStar<intersection, neighbor, RoadMap> astar;
                std::vector<intersection> path;
                astar.SetHeuristic(&rm);
                astar.GetPath(&rm, start, goal, path);
                C = rm.GetPathLength(path);
                gCountMapForwardSingle = calculateWVC.initGCountMap(astar.openClosedList.elements, C);
            }
            t1.EndTimer();
            times.push_back(t1.GetElapsedTime());

            {
                t1.StartTimer();
                TemplateAStar<intersection, neighbor, RoadMap> rastar;
                std::vector<intersection> path;
                rastar.SetHeuristic(&rm);
                rastar.GetPath(&rm, goal, start, path);
                if (rm.GetPathLength(path) != C) {
                    assert(!"Unequal astar and raster paths");
                }
                t1.EndTimer();
                times.push_back(t1.GetElapsedTime());
                std::cout << "[A] RA*; NB\n";
                printf("[R] Path length %d; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                       C, rastar.GetNodesExpanded(), rastar.GetNecessaryExpansions(), t1.GetElapsedTime());
                gCountMapBackwardSingle = calculateWVC.initGCountMap(rastar.openClosedList.elements, C);
            }

            t1.StartTimer();
            double expanded = calculateWVC.CalcWVC(gCountMapForwardSingle, gCountMapBackwardSingle, C, 1, false);
            t1.EndTimer();
            std::cout << "[A] NGMX; NB\n";
            printf("[R] Path length %d; %1.0f expanded; %1.0f necessary; %1.2fs elapsed\n",
                   C, expanded, expanded, times[0] + times[1] + t1.GetElapsedTime());
        }


        std::vector<RMBAELBPair> baelbs;
        baelbs.push_back(RMBAELBPair("BAE*-fd-a", RMBAELB(ivF, ivD, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-fd-p", RMBAELB(ivF, ivD, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-rfrd-a", RMBAELB(ivRF, ivRD, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-rfrd-p", RMBAELB(ivRF, ivRD, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-df-a", RMBAELB(ivD, ivF, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-df-p", RMBAELB(ivD, ivF, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-rdrf-a", RMBAELB(ivRD, ivRF, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-rdrf-p", RMBAELB(ivRD, ivRF, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-g-a", RMBAELB(ivG, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-g-p", RMBAELB(ivG, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-b-a", RMBAELB(ivB, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-b-p", RMBAELB(ivB, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-frdrfd-a", RMBAELB(ivFRD, ivRFD, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-frdrfd-p", RMBAELB(ivFRD, ivRFD, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-gfgd-a", RMBAELB(ivGF, ivGD, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-gfgd-p", RMBAELB(ivGF, ivGD, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-rfdfrd-a", RMBAELB(ivRFD, ivFRD, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-rfdfrd-p", RMBAELB(ivRFD, ivFRD, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-rfrdx2-a", RMBAELB(ivRFRD, ivRFRD, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-rfrdx2-p", RMBAELB(ivRFRD, ivRFRD, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-grfgrd-a", RMBAELB(ivGRF, ivGRD, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-grfgrd-p", RMBAELB(ivGRF, ivGRD, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-gdgf-a", RMBAELB(ivGD, ivGF, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-gdgf-p", RMBAELB(ivGD, ivGF, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-grdgrf-a", RMBAELB(ivGRD, ivGRF, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-grdgrf-p", RMBAELB(ivGRD, ivGRF, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-gb-a", RMBAELB(ivGB, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-gb-p", RMBAELB(ivGB, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-gfrdgrfd-a", RMBAELB(ivGFRD, ivGRFD, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-gfrdgrfd-p", RMBAELB(ivGFRD, ivGRFD, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-grfdgfrd-a", RMBAELB(ivGRFD, ivGFRD, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-grfdgfrd-p", RMBAELB(ivGRFD, ivGFRD, false, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-grfrd-a", RMBAELB(ivGRFRD, true, 1.0, 1.0, false)));
        baelbs.push_back(RMBAELBPair("BAE*-grfrd-p", RMBAELB(ivGRFRD, false, 1.0, 1.0, false)));

        for (RMBAELBPair &solver: baelbs) {
            if (!ap.HasAlgorithm(solver.first)) {
                continue;
            }

            std::cout << "[A] " << solver.first.c_str() << "; NB\n"; // Change to WB if withLog=true
            t1.StartTimer();
            solver.second.GetPath(&rm, start, goal, &rm, &rm, path);
            t1.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   rm.GetPathLength(path), solver.second.GetNodesExpanded(),
                   solver.second.GetNecessaryExpansions(), t1.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = rm.GetPathLength(path);
            else if (optimal_cost != rm.GetPathLength(path)) {
                printf("%s reported bad value!! optimal %1.0f; reported %1.0f;\n", solver.first.c_str(),
                       optimal_cost, rm.GetPathLength(path));
                exit(0);
            }
            solver.second.ClearMemory();
        }

    }
}
