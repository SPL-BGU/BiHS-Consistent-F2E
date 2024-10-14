//
// Created by Lior Siag on 06/05/2024.
//

#ifndef HOG2_AIJ_DRIVER_H
#define HOG2_AIJ_DRIVER_H

void Help() {
    std::cout << "Possible arguments:" << std::endl;
    std::cout << "-d: the problem domain" << std::endl;
    std::cout << "-a: algorithm" << std::endl;
    std::cout << "-i: the instance number (ID)" << std::endl;
    std::cout << "-n: how many instances to do from that instance (default 1)" << std::endl;
    std::cout << "-h: the heuristic (in maps and roadmaps, this specifies the map)" << std::endl;
    std::cout << "-w: weights of the lb function" << std::endl;
}

int main(int argc, char *argv[]);

#endif //HOG2_AIJ_DRIVER_H
