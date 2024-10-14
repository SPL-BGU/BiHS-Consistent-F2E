#ifndef BidirMAPS_h
#define BidirMAPS_h

#include <stdio.h>

#include "ScenarioLoader.h"
#include "Map2DEnvironment.h"
#include "MapGenerators.h"
#include "MapOverlay.h"
#include "NBSQueueGF.h"
#include "NBS.h"
#include "DVCBS.h"
#include "Baseline.h"
#include "GBFHS.h"
#include "DBBS.h"
#include "CDBBS.h"
#include "BTB.h"
#include "TemplateAStar.h"
#include "BSStar.h"
#include "BAE.h"
#include "BAELB.h"

class MapExperiment {

    const std::string folder;
    const std::vector <std::string> mapFiles;
    const std::vector <std::string> scenarioFiles;
    const double weight;

    void runMap(const char *map, const char *scenario, double weight);

public:
    MapExperiment(std::string folder_,
                  std::vector <std::string> mapFiles_,
                  std::vector <std::string> scenarioFiles_,
                  double weight_) :
            folder(folder_), mapFiles(mapFiles_), scenarioFiles(scenarioFiles_), weight(weight_) {}

    void run() {
        for (int i = 0; i < mapFiles.size(); i++) {
            runMap(("hog2-private/maps/" + folder + "/" + mapFiles[i]).c_str(),
                   ("hog2-private/scenarios/" + folder + "/" + scenarioFiles[i]).c_str(), 1.0);
        }
        exit(0);
    }
};


#endif /* BidirMAPS_hpp */
