
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>

#include <iostream>

#include "PolePlacementControl.h"


int main(){

    ControlSystemZ zC = ControlSystemZ(0,0,0,0,0,0,0,0,0,0,0,0,0);

    printf("%f", zC.calculateZ(0.2, 0.5));

    return 0;

}