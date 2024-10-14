#include "BidirMaps.h"
#include "GMX.h"
#include "DBBSLB.h"
#include "CalculateWVC.h"

#define MapBAELBPair std::pair<std::string, BAELB<xyLoc, tDirection, MapEnvironment>>
#define MapBAELB BAELB<xyLoc, tDirection, MapEnvironment>

void MapExperiment::runMap(const char *map, const char *scenario, double weight) {
    ScenarioLoader s(scenario);
    Map *m = new Map(map);
    MapEnvironment *me = new MapEnvironment(m);
    me->SetDiagonalCost(1.5);

    // 406 is bad! (?)
    for (int x = s.GetNumExperiments() - 1; x >= 0; x--) // 547 to 540
    {
        if (fequal(s.GetNthExperiment(x).GetDistance(), 0))
            continue;

        //Speedup for testing purposes - this runs 20 instances per map
        if (x != 0 && ((x + 1) % ((int) (s.GetNumExperiments() / 19)) != 0))
            continue;

        xyLoc start, goal;
        start.x = s.GetNthExperiment(x).GetStartX();
        start.y = s.GetNthExperiment(x).GetStartY();
        goal.x = s.GetNthExperiment(x).GetGoalX();
        goal.y = s.GetNthExperiment(x).GetGoalY();

        std::cout << "[I] " << map << "-" << start << "-" << goal << std::endl;

        double optimal_cost = -1.0;


        // BAE*
        if (false) {
            BAE<xyLoc, tDirection, MapEnvironment> bae(true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] BAE*-o-a; NB\n";
            timer.StartTimer();
            bae.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), bae.GetNodesExpanded(),
                   bae.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("BAE*-o-a reported bad value!! optimal %1.1f; reported %1.1f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // BAE*-p
        if (true) {
            BAE<xyLoc, tDirection, MapEnvironment> bae(false, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] BAE*-o-p; NB\n";
            timer.StartTimer();
            bae.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), bae.GetNodesExpanded(),
                   bae.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("BAE*-o-p reported bad value!! optimal %1.1f; reported %1.1f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // A*
        if (true) {
            TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
            std::vector<xyLoc> path;
            std::cout << "[A] A*; NB\n";
            Timer timer;
            astar.SetHeuristic(me);
            timer.StartTimer();
            astar.GetPath(me, start, goal, path);
            timer.EndTimer();
            printf("[R] Path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), astar.GetNodesExpanded(),
                   astar.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("A* reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // NBS
        if (true) {
            NBS<xyLoc, tDirection, MapEnvironment, NBSQueue<xyLoc, 1, false>> nbs(false, true);
            std::vector<xyLoc> path;
            std::cout << "[A] NBS; NB\n";
            Timer timer;
            timer.StartTimer();
            nbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), nbs.GetNodesExpanded(),
                   nbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("NBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // DVCBS
        if (true) {
            DVCBS<xyLoc, tDirection, MapEnvironment, DVCBSQueue<xyLoc, 1, false>> dvcbs(false, true);
            std::vector<xyLoc> path;
            std::cout << "[A] DVCBS; NB\n";
            Timer timer;
            timer.StartTimer();
            dvcbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dvcbs.GetNodesExpanded(),
                   dvcbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DVCBS-E-L reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-a
        if (false) {
            DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinB> dbbs(SideCriterion::Alt, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBS-a; NB\n";
            timer.StartTimer();
            dbbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBS-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-p
        if (false) {
            DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinB> dbbs(SideCriterion::Cardinality, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBS-p; NB\n";
            timer.StartTimer();
            dbbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBS-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-a-MaxTot
        if (false) {
            DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MaxTot> dbbs(SideCriterion::Alt, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBS-a-MaxTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBS-a-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-a-MinTot
        if (false) {
            DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinTot> dbbs(SideCriterion::Alt, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBS-a-MinTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBS-a-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-o-MaxTot
        if (false) {
            DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MaxTot> dbbs(SideCriterion::OptCount, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBS-o-MaxTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBS-o-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBS-o-MinTot
        if (false) {
            DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinTot> dbbs(SideCriterion::OptCount, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBS-o-MinTot; NB\n";
            timer.StartTimer();
            dbbs.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBS-o-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-a
        if (false) {
            DBBSLB<xyLoc, tDirection, MapEnvironment, MinCriterion::MinB> dbbslb(SideCriterion::Alt, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBSLB-a; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBSLB-a reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-p
        if (false) {
            DBBSLB<xyLoc, tDirection, MapEnvironment, MinCriterion::MinB> dbbslb(SideCriterion::Cardinality, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBSLB-p; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBSLB-p reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-a-MaxTot
        if (false) {
            DBBSLB<xyLoc, tDirection, MapEnvironment, MinCriterion::MaxTot> dbbslb(SideCriterion::Alt, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBSLB-a-MaxTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBSLB-a-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-a-MinTot
        if (false) {
            DBBSLB<xyLoc, tDirection, MapEnvironment, MinCriterion::MinTot> dbbslb(SideCriterion::Alt, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBSLB-a-MinTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBSLB-a-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-o-MaxTot
        if (false) {
            DBBSLB<xyLoc, tDirection, MapEnvironment, MinCriterion::MaxTot> dbbslb(SideCriterion::OptCount, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBSLB-o-MaxTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBSLB-o-MaxTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        //DBBSLB-o-MinTot
        if (false) {
            DBBSLB<xyLoc, tDirection, MapEnvironment, MinCriterion::MinTot> dbbslb(SideCriterion::OptCount, true, 1.0, 0.5);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] DBBSLB-o-MinTot; NB\n";
            timer.StartTimer();
            dbbslb.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), dbbslb.GetNodesExpanded(),
                   dbbslb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("DBBSLB-o-MinTot reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        // GMX
        if (false) {
            std::cout << "[A] GMX; NB\n";
            GMX<xyLoc> gmx;
            Timer timer;
            timer.StartTimer();
            {
                TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
                std::vector<xyLoc> path;
                astar.SetHeuristic(me);
                astar.GetPath(me, start, goal, path);
                gmx.GenerateBuckets(astar.openClosedList.elements, me, start, me->GetPathLength(path), true);
            }

            {
                TemplateAStar<xyLoc, tDirection, MapEnvironment> rastar;
                std::vector<xyLoc> path;
                rastar.SetHeuristic(me);
                rastar.GetPath(me, goal, start, path);
                gmx.GenerateBuckets(rastar.openClosedList.elements, me, start, me->GetPathLength(path), false);
            }
            int expanded = gmx.GetMinimalExpansions();
            timer.EndTimer();
            printf("[R] Path length %1.1f; %d expanded; %d necessary; %1.2fs elapsed\n",
                   optimal_cost, expanded, expanded, timer.GetElapsedTime());
        }

        //NGMX
        if (false) {
            CalculateWVC <xyLoc> calculateWVC;
            int C = -1;
            std::map<int, int> gCountMapForwardSingle;
            std::map<int, int> gCountMapBackwardSingle;
            std::vector<double> times;
            Timer timer;
            timer.StartTimer();
            {
                TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
                std::vector<xyLoc> path;
                astar.SetHeuristic(me);
                astar.GetPath(me, start, goal, path);
                C = me->GetPathLength(path)*2;
                for (auto &element: astar.openClosedList.elements) {
                    element.g *= 2;
                    element.h *= 2;
                }
                gCountMapForwardSingle = calculateWVC.initGCountMap(astar.openClosedList.elements, C);
            }
            timer.EndTimer();
            times.push_back(timer.GetElapsedTime());

            {
                timer.StartTimer();
                TemplateAStar<xyLoc, tDirection, MapEnvironment> rastar;
                std::vector<xyLoc> path;
                rastar.SetHeuristic(me);
                rastar.GetPath(me, goal, start, path);
                if (me->GetPathLength(path)*2 != C) {
                    assert(!"Unequal astar and raster paths");
                }
                timer.EndTimer();
                times.push_back(timer.GetElapsedTime());
                std::cout << "[A] RA*; NB\n";
                printf("[R] Path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                       C/2.0, rastar.GetNodesExpanded(), rastar.GetNecessaryExpansions(), timer.GetElapsedTime());
                timer.StartTimer();
                for (auto &element: rastar.openClosedList.elements) {
                    element.g *= 2;
                    element.h *= 2;
                }
                gCountMapBackwardSingle = calculateWVC.initGCountMap(rastar.openClosedList.elements, C);
            }

            int expanded = calculateWVC.CalcWVC(gCountMapForwardSingle, gCountMapBackwardSingle, C, 2, false);
            timer.EndTimer();
            std::cout << "[A] NGMX; NB\n";
            printf("[R] Path length %1.1f; %d expanded; %d necessary; %1.2fs elapsed\n",
                   C/2.0, expanded, expanded, times[0] + times[1] + timer.GetElapsedTime());
        }

        if(true) {
            BTB<xyLoc, tDirection, MapEnvironment> btb(BTBPolicy::Alternating);
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] BTB-nbs; NB\n";
            timer.StartTimer();
            btb.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), btb.GetNodesExpanded(),
                   btb.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("BTB-nbs reported bad value!! optimal %1.0f; reported %1.0f;\n",
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
        }

        std::vector<MapBAELBPair> baelbs;
        baelbs.push_back(MapBAELBPair("BAE*-fd-a", MapBAELB(ivF, ivD, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-fd-p", MapBAELB(ivF, ivD, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-rfrd-a", MapBAELB(ivRF, ivRD, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-rfrd-p", MapBAELB(ivRF, ivRD, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-df-a", MapBAELB(ivD, ivF, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-df-p", MapBAELB(ivD, ivF, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-rdrf-a", MapBAELB(ivRD, ivRF, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-rdrf-p", MapBAELB(ivRD, ivRF, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-g-a", MapBAELB(ivG, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-g-p", MapBAELB(ivG, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-b-a", MapBAELB(ivB, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-b-p", MapBAELB(ivB, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-frdrfd-a", MapBAELB(ivFRD, ivRFD, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-frdrfd-p", MapBAELB(ivFRD, ivRFD, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-gfgd-a", MapBAELB(ivGF, ivGD, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-gfgd-p", MapBAELB(ivGF, ivGD, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-rfdfrd-a", MapBAELB(ivRFD, ivFRD, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-rfdfrd-p", MapBAELB(ivRFD, ivFRD, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-rfrdx2-a", MapBAELB(ivRFRD, ivRFRD, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-rfrdx2-p", MapBAELB(ivRFRD, ivRFRD, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-grfgrd-a", MapBAELB(ivGRF, ivGRD, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-grfgrd-p", MapBAELB(ivGRF, ivGRD, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-gdgf-a", MapBAELB(ivGD, ivGF, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-gdgf-p", MapBAELB(ivGD, ivGF, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-grdgrf-a", MapBAELB(ivGRD, ivGRF, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-grdgrf-p", MapBAELB(ivGRD, ivGRF, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-gb-a", MapBAELB(ivGB, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-gb-p", MapBAELB(ivGB, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-gfrdgrfd-a", MapBAELB(ivGFRD, ivGRFD, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-gfrdgrfd-p", MapBAELB(ivGFRD, ivGRFD, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-grfdgfrd-a", MapBAELB(ivGRFD, ivGFRD, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-grfdgfrd-p", MapBAELB(ivGRFD, ivGFRD, false, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-grfrd-a", MapBAELB(ivGRFRD, true, 1.0, 0.5, false)));
        baelbs.push_back(MapBAELBPair("BAE*-grfrd-p", MapBAELB(ivGRFRD, false, 1.0, 0.5, false)));

        for (MapBAELBPair &solver: baelbs) {
            std::vector<xyLoc> path;
            Timer timer;
            std::cout << "[A] " << solver.first.c_str() << "; NB\n"; // Change to WB if withLog=true
            timer.StartTimer();
            solver.second.GetPath(me, start, goal, me, me, path);
            timer.EndTimer();
            printf("[R] Path length %1.1f; %llu expanded; %llu necessary; %1.2fs elapsed\n",
                   me->GetPathLength(path), solver.second.GetNodesExpanded(),
                   solver.second.GetNecessaryExpansions(), timer.GetElapsedTime());

            // test optimality
            if (optimal_cost < 0.0) optimal_cost = me->GetPathLength(path);
            else if (optimal_cost != me->GetPathLength(path)) {
                printf("%s reported bad value!! optimal %1.1f; reported %1.1f;\n", solver.first.c_str(),
                       optimal_cost, me->GetPathLength(path));
                exit(0);
            }
            solver.second.ClearMemory();
        }
    }
}

