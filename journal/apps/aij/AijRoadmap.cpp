//
// Created by Lior Siag on 21/07/2024.
//

#include "AijRoadmap.h"
#include "RoadMap.h"
#include "BAE.h"
#include "CSBS.h"
#include "BAELB.h"
#include "DBBS.h"
#include "DBBSLB.h"

#define RoadBAELB BAELB<intersection, neighbor, RoadMap>
#define RoadBAELBPair std::pair<std::string, RoadBAELB>

void TestSpecificRoadmap(const char *graph_path, const char *coordinates_path, int instanceId, int numOfInstances,
                         const std::string &alg, const std::vector<double> &csbsWeights) {
    RoadMap rm(graph_path, coordinates_path, false);
    std::vector<intersection> solutionPath;
    Timer timer;
    std::cout << "[D] Col MD" << std::endl;
    for (int i = instanceId; i < instanceId + numOfInstances; ++i) {
        srandom(i * 16);
        node *n1 = rm.GetGraph()->GetRandomNode();
        node *n2 = rm.GetGraph()->GetRandomNode();
        std::cout << "[I] ID " << i << " ("
                  << n1->GetLabelF(GraphSearchConstants::kXCoordinate) << ","
                  << n1->GetLabelF(GraphSearchConstants::kYCoordinate)
                  << ") (" << n2->GetLabelF(GraphSearchConstants::kXCoordinate) << ","
                  << n2->GetLabelF(GraphSearchConstants::kYCoordinate) << ")\n";

        intersection start = rm.GetGraph()->GetRandomNode()->GetNum();
        intersection goal = rm.GetGraph()->GetRandomNode()->GetNum();

        if (alg == "csbs" || alg == "all") {
            CSBS<intersection, neighbor, RoadMap> csbs(csbsWeights, true);
            timer.StartTimer();
            csbs.GetPath(&rm, start, goal, &rm, &rm, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", csbs.GetName().c_str(),
                   rm.GetPathLength(solutionPath), csbs.GetNodesExpanded(), csbs.GetNecessaryExpansions(),
                   timer.GetElapsedTime());
            solutionPath.clear();
        }

        if (alg == "bae" || alg == "all") {
            BAE<intersection, neighbor, RoadMap> bae(SideCriterion::OptCount);
            timer.StartTimer();
            bae.GetPath(&rm, start, goal, &rm, &rm, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "BAE-FBI",
                   rm.GetPathLength(solutionPath), bae.GetNodesExpanded(), bae.GetNecessaryExpansions(),
                   timer.GetElapsedTime());
            solutionPath.clear();
        }

        if (alg == "dbbs-max-a" || alg == "dbbs-max" || alg == "all") {
            DBBS<intersection, neighbor, RoadMap, MinCriterion::MaxTot> dbbs(SideCriterion::Alt);
            timer.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-a-MaxTot",
                   rm.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }


        if (alg == "dbbs-max-p" || alg == "dbbs-max" || alg == "all") {
            DBBS<intersection, neighbor, RoadMap, MinCriterion::MaxTot> dbbs(SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-p-MaxTot",
                   rm.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbs-max-fbi" || alg == "dbbs-max" || alg == "all") {
            DBBSLB<intersection, neighbor, RoadMap, MinCriterion::MaxTot> dbbs(SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBS-fbi-MaxTot",
                   rm.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbsf-max-a" || alg == "dbbsf-max" || alg == "all") {
            DBBSLB<intersection, neighbor, RoadMap, MinCriterion::MaxTot> dbbs(SideCriterion::Alt);
            timer.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-a-MaxTot",
                   rm.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }


        if (alg == "dbbsf-max-p" || alg == "dbbsf-max" || alg == "all") {
            DBBSLB<intersection, neighbor, RoadMap, MinCriterion::MaxTot> dbbs(SideCriterion::Cardinality);
            timer.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-p-MaxTot",
                   rm.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        if (alg == "dbbsf-max-fbi" || alg == "dbbsf-max" || alg == "all") {
            DBBS<intersection, neighbor, RoadMap, MinCriterion::MaxTot> dbbs(SideCriterion::OptCount);
            timer.StartTimer();
            dbbs.GetPath(&rm, start, goal, &rm, &rm, solutionPath);
            timer.EndTimer();
            printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", "DBBSF-fbi-MaxTot",
                   rm.GetPathLength(solutionPath), dbbs.GetNodesExpanded(),
                   dbbs.GetNecessaryExpansions(), timer.GetElapsedTime());
        }

        std::vector<RoadBAELBPair > baelbs;
        baelbs.emplace_back("tb-fd-fbi", RoadBAELB(ivF, ivD, SideCriterion::OptCount));
        baelbs.emplace_back("tb-df-fbi", RoadBAELB(ivD, ivF, SideCriterion::OptCount));
        baelbs.emplace_back("tb-g-fbi", RoadBAELB(ivG, ivG, SideCriterion::OptCount));
        baelbs.emplace_back("tb-b-fbi", RoadBAELB(ivB, ivB, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gfgd-fbi", RoadBAELB(ivGF, ivGD, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gdgf-fbi", RoadBAELB(ivGD, ivGF, SideCriterion::OptCount));
        baelbs.emplace_back("tb-gb-fbi", RoadBAELB(ivGB, ivGB, SideCriterion::OptCount));
        baelbs.emplace_back("tb-fd-d", RoadBAELB(ivF, ivD, SideCriterion::Decay));
        baelbs.emplace_back("tb-df-d", RoadBAELB(ivD, ivF, SideCriterion::Decay));
        baelbs.emplace_back("tb-g-d", RoadBAELB(ivG, ivG, SideCriterion::Decay));
        baelbs.emplace_back("tb-b-d", RoadBAELB(ivB, ivB, SideCriterion::Decay));
        baelbs.emplace_back("tb-gfgd-d", RoadBAELB(ivGF, ivGD, SideCriterion::Decay));
        baelbs.emplace_back("tb-gdgf-d", RoadBAELB(ivGD, ivGF, SideCriterion::Decay));
        baelbs.emplace_back("tb-gb-d", RoadBAELB(ivGB, ivGB, SideCriterion::Decay));

        for (RoadBAELBPair &solver: baelbs) {
            if (alg == solver.first || alg == "tb" || alg == "all") {
                timer.StartTimer();
                solver.second.GetPath(&rm, start, goal, &rm, &rm, solutionPath);
                timer.EndTimer();
                printf("[R] %s; solution %1.0f; expanded %llu; necessary %llu; time %1.2fs\n", solver.first.c_str(),
                       rm.GetPathLength(solutionPath), solver.second.GetNodesExpanded(),
                       solver.second.GetNecessaryExpansions(), timer.GetElapsedTime());
            }
        }
    }
}

void TestRoadmap(const ArgParameters &ap) {
    const std::string graphStr = "../../maps/roadmaps/USA-road-t.col.gr";
    const std::string coordStr = "../../maps/roadmaps/USA-road-d.col.co";
    TestSpecificRoadmap(graphStr.c_str(), coordStr.c_str(), ap.instanceId, ap.numOfInstances, ap.alg, ap.weights);
}
