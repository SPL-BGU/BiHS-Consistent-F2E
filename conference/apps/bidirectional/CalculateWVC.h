#ifndef CALCULATEWVC_H
#define CALCULATEWVC_H

#include <cassert>
#include <vector>
#include <ext/hash_map>
#include <stdint.h>

template <class state>
class CalculateWVC {
public:
	CalculateWVC() {}
	~CalculateWVC() {}
	double getOptimalP() { return optimalP; }
	double CalcWVC(std::vector<AStarOpenClosedData<state>> &astarOpenClosedList,
				   std::vector<AStarOpenClosedData<state>> &reverstAstarOpenClosedList,
				   int C, int epsilon, bool isAllMustExpand) {
		std::map<int,int> gCountMapForward = initGCountMap(astarOpenClosedList,C);
		std::map<int,int> gCountMapBackward = initGCountMap(reverstAstarOpenClosedList,C);	  
		return CalcWVC(gCountMapForward,gCountMapBackward,C,epsilon,isAllMustExpand);
	}
	
	double CalcWVC(std::map<int,int> &gCountMapForward,
				   std::map<int,int> &gCountMapBackward,
				   int C, int epsilon, bool isAllMustExpand) {
		if (isAllMustExpand){
			C++;
		}		   
		_C = C;
		
		// *** print vectors and map to check map is built correctly ***
		// printOpenClosedDataG(astarOpenClosedList);
		// printOpenClosedDataG(reverstAstarOpenClosedList);
		// printMap(gCountMapForward);
		// printMap(gCountMapBackward);
		
		//printf("C: %d\n", _C);
		
		int i = 0; int j= C - i - epsilon;;
		double wvc = allWVC(gCountMapBackward, C - epsilon);
		//printf("init WVC: %f\n", wvc);
		double minWVC = wvc;

		while (i < C - epsilon)
		{
			wvc = wvc + gCountMapForward[i];
			// printf("i before is: %d\n", i);
			i = nextG(gCountMapForward, i);
			// printf("i after is: %d\n", i);
			int oldj = j;
			j = C - i - epsilon;
			for(int jTag = j; jTag < oldj; jTag = nextG(gCountMapBackward, jTag)) {
				wvc = wvc - gCountMapBackward[jTag];
			}
			if(wvc < minWVC) {
				minWVC = wvc;
				//printf("i %d, j%d\n",i-1,j);
				optimalP = (double)i / (double)C ;
			}		
		}	
		
		return minWVC;
	}
	
		std::map<int,int> initGCountMap(std::vector<AStarOpenClosedData<state>> openClosedList, double CStar) {
		std::map<int,int> ngMap;
		for(int i = 0; i < openClosedList.size(); i++) {
			AStarOpenClosedData<state> item = openClosedList[i];
			int g = item.g;
			if(item.where == kClosedList && (item.g+item.h < CStar) ) {
				if(ngMap.find(g) == ngMap.end()) {
					//Element not found
					ngMap.insert(std::pair<int, int>(g, 1));
				} else {
					//Element found
					ngMap[g] += 1;
				}
			}
		}
		return ngMap;
	}
	
private:

	double optimalP = 0;
	int _C;


	
	double allWVC(std::map<int,int> map, int C) {
		int count = 0;
		
		for(int j = 0; j < map.size() && j < C; j++) {
			count = count + map[j];
		}
		
		return count;
	}
	
	int nextG(std::map<int,int> map, int nextOf) {
		for(int i = 1; i < map.size(); i++) {
			if(map.find(nextOf+i) != map.end()) {
				//Element found
				return (nextOf+i);
			}
		}
		return _C;
	}
	
	void printMap(std::map<int,int> map) {
		printf("*** print map *** \n");
		for (std::map<int,int>::iterator it=map.begin(); it!=map.end(); ++it)
			std::cout << it->first << " => " << it->second << '\n';
	}
	
	void printOpenClosedDataG(std::vector<AStarOpenClosedData<state>> openClosedDataVec) {
		printf("*** print closed list G values *** \n");
		for(int i = 0; i < openClosedDataVec.size(); i++) {
			AStarOpenClosedData<state> item = openClosedDataVec[i];
			if(item.where == kClosedList) {
				printf("%1.0f ", item.g);
			}
		}
		printf("\n");
	}
};

#endif
