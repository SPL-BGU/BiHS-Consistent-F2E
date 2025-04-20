//
// Created by Lior Siag on 15/07/2024.
//

#include <stdexcept>
#include <algorithm>
#include "AijMaps.h"
#include "Utilities.h"
#include "ScenarioLoader.h"
#include "Map.h"
#include "Map2DEnvironment.h"
#include "CSBS.h"
#include "BAE.h"
#include "TemplateAStar.h"
#include "BAELB.h"
#include "DBBS.h"
#include "DBBSLB.h"

#define MapsBAELB BAELB<xyLoc, tDirection, MapEnvironment>
#define MapsBAELBPair std::pair<std::string, MapsBAELB>

void TestSpecificMap(const char *map, const char *scenario, int instanceId, int numOfInstances, const std::string &alg,
                     const std::vector<double> &csbsWeights) {

    printf("[D] DAO %s\n", map);
    ScenarioLoader s(scenario);
    Map m(map);
    MapEnvironment me(&m);
    me.SetDiagonalCost(1.5);
    int currInstance = -1;
    std::vector<xyLoc> solutionPath;
    Timer timer;

    for (int x = s.GetNumExperiments() - 1; x >= 0; x--) {
        if (fequal(s.GetNthExperiment(x).GetDistance(), 0)) {
            continue;

        }
        if (x != 0 && ((x + 1) % ((int) (s.GetNumExperiments() / 19)) != 0)) {
            continue;
        }

        currInstance++;
        if (currInstance >= instanceId && currInstance < instanceId + numOfInstances) {
            xyLoc start, goal;
            start.x = s.GetNthExperiment(x).GetStartX();
            start.y = s.GetNthExperiment(x).GetStartY();
            goal.x = s.GetNthExperiment(x).GetGoalX();
            goal.y = s.GetNthExperiment(x).GetGoalY();
            std::cout << "[I] Scen " << x << " " << start << "-" << goal << std::endl;

            if (alg == "csbs" || alg == "all") {
                CSBS<xyLoc, tDirection, MapEnvironment> csbs(csbsWeights, true, 1.0, 0.5);
                timer.StartTimer();
                csbs.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.2fs\n", csbs.GetName().c_str(),
                       me.GetPathLength(solutionPath), csbs.GetNodesExpanded(), csbs.GetNecessaryExpansions(),
                       timer.GetElapsedTime());
            }

            if (alg == "bae" || alg == "all") {
                BAE<xyLoc, tDirection, MapEnvironment> bae(SideCriterion::OptCount, 1.0, 0.5);
                timer.StartTimer();
                bae.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "BAE-FBI",
                       me.GetPathLength(solutionPath), bae.GetNodesExpanded(), bae.GetNecessaryExpansions(),
                       timer.GetElapsedTime());
            }

            if (alg == "astar" || alg == "all") {
                TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
                astar.SetHeuristic(&me);
                timer.StartTimer();
                astar.GetPath(&me, start, goal, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "BAE-FBI",
                       me.GetPathLength(solutionPath), astar.GetNodesExpanded(), astar.GetNecessaryExpansions(),
                       timer.GetElapsedTime());
            }

            if (alg == "dbbs-max-a" || alg == "dbbs-max" || alg == "all") {
                DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MaxTot> dbbs(
                        SideCriterion::Alt, true, 1.0, 0.5);
                timer.StartTimer();
                dbbs.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "DBBS-a-MaxTot",
                       me.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                       dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
            }


            if (alg == "dbbs-max-p" || alg == "dbbs-max" || alg == "all") {
                DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MaxTot> dbbs(
                        SideCriterion::Cardinality, true, 1.0, 0.5);
                timer.StartTimer();
                dbbs.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "DBBS-p-MaxTot",
                       me.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                       dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
            }

            if (alg == "dbbs-max-fbi" || alg == "dbbs-max" || alg == "all") {
                DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MaxTot> dbbs(
                        SideCriterion::OptCount, true, 1.0, 0.5);
                timer.StartTimer();
                dbbs.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "DBBS-fbi-MaxTot",
                       me.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                       dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
            }

            if (alg == "dbbsf-max-a" || alg == "dbbsf-max" || alg == "all") {
                DBBSLB<xyLoc, tDirection, MapEnvironment, MinCriterion::MaxTot> dbbs(
                        SideCriterion::Alt, true, 1.0, 0.5);
                timer.StartTimer();
                dbbs.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "DBBSF-a-MaxTot",
                       me.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                       dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
            }


            if (alg == "dbbsf-max-p" || alg == "dbbsf-max" || alg == "all") {
                DBBSLB<xyLoc, tDirection, MapEnvironment, MinCriterion::MaxTot> dbbs(
                        SideCriterion::Cardinality, true, 1.0, 0.5);
                timer.StartTimer();
                dbbs.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "DBBSF-p-MaxTot",
                       me.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                       dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
            }

            if (alg == "dbbsf-max-fbi" || alg == "dbbsf-max" || alg == "all") {
                DBBSLB<xyLoc, tDirection, MapEnvironment, MinCriterion::MaxTot> dbbs(
                        SideCriterion::OptCount, true, 1.0, 0.5);
                timer.StartTimer();
                dbbs.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "DBBSF-fbi-MaxTot",
                       me.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                       dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
            }

            if (alg == "dbbs-b-fbi" || alg == "dbbs-b" || alg == "all") {
                DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinB> dbbs(
                        SideCriterion::OptCount, true, 1.0, 0.5);
                timer.StartTimer();
                dbbs.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "DBBS-fbi-b",
                       me.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                       dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
            }

            if (alg == "dbbsf-b-fbi" || alg == "dbbsf-b" || alg == "all") {
                DBBSLB<xyLoc, tDirection, MapEnvironment, MinCriterion::MinB> dbbs(
                        SideCriterion::OptCount, true, 1.0, 0.5);
                timer.StartTimer();
                dbbs.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "DBBSF-fbi-b",
                       me.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                       dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
            }

            if (alg == "dbbs-ss-p" || alg == "dbbs-ss" || alg == "all") {
                DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinTot> dbbs(
                        SideCriterion::Cardinality, true, 1.0, 0.5);
                timer.StartTimer();
                dbbs.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "DBBS-p-MinTot",
                       me.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                       dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
            }

            if (alg == "dbbsf-ss-p" || alg == "dbbsf-ss" || alg == "all") {
                DBBSLB<xyLoc, tDirection, MapEnvironment, MinCriterion::MinTot> dbbs(
                        SideCriterion::Cardinality, true, 1.0, 0.5);
                timer.StartTimer();
                dbbs.GetPath(&me, start, goal, &me, &me, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.6fs\n", "DBBSF-p-MinTot",
                       me.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                       dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
            }


            std::vector<MapsBAELBPair > baelbs;
            baelbs.emplace_back("tb-fd-fbi", MapsBAELB(ivF, ivD, SideCriterion::OptCount, 1.0, 0.5));
            baelbs.emplace_back("tb-df-fbi", MapsBAELB(ivD, ivF, SideCriterion::OptCount, 1.0, 0.5));
            baelbs.emplace_back("tb-g-fbi", MapsBAELB(ivG, ivG, SideCriterion::OptCount, 1.0, 0.5));
            baelbs.emplace_back("tb-b-fbi", MapsBAELB(ivB, ivB, SideCriterion::OptCount, 1.0, 0.5));
            baelbs.emplace_back("tb-gfgd-fbi", MapsBAELB(ivGF, ivGD, SideCriterion::OptCount, 1.0, 0.5));
            baelbs.emplace_back("tb-gdgf-fbi", MapsBAELB(ivGD, ivGF, SideCriterion::OptCount, 1.0, 0.5));
            baelbs.emplace_back("tb-gb-fbi", MapsBAELB(ivGB, ivGB, SideCriterion::OptCount, 1.0, 0.5));
            baelbs.emplace_back("tb-fd-d", MapsBAELB(ivF, ivD, SideCriterion::Decay));
            baelbs.emplace_back("tb-df-d", MapsBAELB(ivD, ivF, SideCriterion::Decay));
            baelbs.emplace_back("tb-g-d", MapsBAELB(ivG, ivG, SideCriterion::Decay));
            baelbs.emplace_back("tb-b-d", MapsBAELB(ivB, ivB, SideCriterion::Decay));
            baelbs.emplace_back("tb-gfgd-d", MapsBAELB(ivGF, ivGD, SideCriterion::Decay));
            baelbs.emplace_back("tb-gdgf-d", MapsBAELB(ivGD, ivGF, SideCriterion::Decay));
            baelbs.emplace_back("tb-gb-d", MapsBAELB(ivGB, ivGB, SideCriterion::Decay));

            for (MapsBAELBPair &solver: baelbs) {
                if (alg == solver.first || alg == "tb" || alg == "all") {
                    timer.StartTimer();
                    solver.second.GetPath(&me, start, goal, &me, &me, solutionPath);
                    timer.EndTimer();
                    printf("[R] %s; solution %1.1f; expanded %llu; necessary %llu; time %1.2fs\n", solver.first.c_str(),
                           me.GetPathLength(solutionPath), solver.second.GetNodesExpanded(),
                           solver.second.GetNecessaryExpansions(), timer.GetElapsedTime());
                }
            }
        }
    }
}

void TestMaps(const struct ArgParameters &ap) {


    std::vector<std::string> maps;
    if (ap.heuristic == "all") {
        maps = daoMapsNames;
    } else {
        if (std::find(daoMapsNames.begin(), daoMapsNames.end(), ap.heuristic) == daoMapsNames.end()) {
            throw std::invalid_argument("Unknown map name");
        }
        maps.push_back(ap.heuristic);
    }

    for (auto &mapName: maps) {
        std::string maps_path = "../../maps/dao/" + mapName + ".map";
        std::string scen_path = "../../scenarios/dao/" + mapName + ".map.scen";
        TestSpecificMap(maps_path.c_str(), scen_path.c_str(), ap.instanceId, ap.numOfInstances, ap.alg, ap.weights);
    }

}
